#!/usr/bin/env python3

import math
import statistics
import subprocess

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

from tf2_ros import Buffer, TransformListener, TransformException


class WallFollowNode(Node):

    def __init__(self):
        super().__init__('wall_follow_node')

        # ============================================================
        # Wall Follow Parameters
        # ============================================================

        # 1차 Map-Run 서비스가 시작되면 주행 허용
        self.declare_parameter(
            'start_enabled',
            True
        )

        # 오른쪽 벽과 30 cm 유지
        self.declare_parameter(
            'target_wall_distance',
            0.30
        )

        self.declare_parameter(
            'front_turn_distance',
            0.55
        )

        self.declare_parameter(
            'emergency_distance',
            0.25
        )

        self.declare_parameter(
            'wall_lost_distance',
            1.20
        )

        self.declare_parameter(
            'linear_speed',
            0.15
        )

        self.declare_parameter(
            'kp',
            1.2
        )

        self.declare_parameter(
            'max_angular_speed',
            0.60
        )

        self.declare_parameter(
            'corner_turn_speed',
            0.60
        )

        self.declare_parameter(
            'lidar_yaw_offset_deg',
            0.0
        )

        self.declare_parameter(
            'vision_timeout',
            2.0
        )

        # ============================================================
        # Lap Detection Parameters
        # ============================================================

        # 최소 총 주행거리
        self.declare_parameter(
            'min_lap_distance',
            4.0
        )

        # 시작 위치에서 이 거리 이상 벗어나야
        # "한 바퀴를 돌고 돌아왔다"고 판단할 자격을 얻음
        self.declare_parameter(
            'leave_start_radius',
            0.70
        )

        # 시작 위치로 이 거리 이내 복귀하면 완주 후보
        self.declare_parameter(
            'return_radius',
            0.40
        )

        # 시작 직후 오판 방지
        self.declare_parameter(
            'min_lap_time',
            25.0
        )

        # ============================================================
        # Map Save Script
        # ============================================================

        self.declare_parameter(
            'save_script',
            '/home/mice/ugv02_slam_test/save_mapping_result.py'
        )

        # ============================================================
        # Publishers
        # ============================================================

        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.map_run_finished_pub = self.create_publisher(
            Bool,
            '/map_run_finished',
            10
        )

        # ============================================================
        # Subscribers
        # ============================================================

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile_sensor_data
        )

        self.vision_stop_sub = self.create_subscription(
            Bool,
            '/vision_stop',
            self.vision_stop_callback,
            10
        )

        self.vision_ready_sub = self.create_subscription(
            Bool,
            '/vision_ready',
            self.vision_ready_callback,
            10
        )

        # ============================================================
        # TF
        # ============================================================

        self.tf_buffer = Buffer()

        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )

        # ============================================================
        # LiDAR States
        # ============================================================

        self.front = float('inf')
        self.front_right = float('inf')
        self.right = float('inf')

        self.last_scan_time = None

        # ============================================================
        # Vision Fail-safe
        # ============================================================

        self.vision_stop = True
        self.vision_ready = False
        self.last_vision_time = None

        # ============================================================
        # Lap Tracking
        # ============================================================

        self.start_pose = None
        self.previous_pose = None

        self.travelled_distance = 0.0

        self.start_time = None

        # 반드시 시작점을 충분히 벗어난 뒤에만
        # 복귀 판정을 허용
        self.left_start_area = False

        self.lap_finished = False

        # ============================================================
        # Saving State
        # ============================================================

        self.save_process = None
        self.save_started = False
        self.save_finished = False

        # ============================================================
        # Logging State
        # ============================================================

        self.previous_state = None
        self.log_counter = 0

        # ============================================================
        # Control Loop : 10 Hz
        # ============================================================

        self.control_timer = self.create_timer(
            0.1,
            self.control_loop
        )

        self.get_logger().info(
            '=========================================='
        )

        self.get_logger().info(
            '1st MAP-RUN WALL FOLLOW NODE READY'
        )

        self.get_logger().info(
            'Wall target distance = 0.30 m'
        )

        self.get_logger().info(
            'Automatic lap detection = ENABLED'
        )

        self.get_logger().info(
            'Automatic map/path save = ENABLED'
        )

        self.get_logger().info(
            '=========================================='
        )

    # ================================================================
    # Utility
    # ================================================================

    @staticmethod
    def normalize_angle_deg(angle):

        while angle > 180.0:
            angle -= 360.0

        while angle < -180.0:
            angle += 360.0

        return angle

    # ================================================================
    # LiDAR Sector
    # ================================================================

    def get_sector(
        self,
        scan,
        center_deg,
        width_deg,
        mode='median'
    ):

        values = []

        yaw_offset = float(
            self.get_parameter(
                'lidar_yaw_offset_deg'
            ).value
        )

        for i, distance in enumerate(scan.ranges):

            if not math.isfinite(distance):
                continue

            if distance < scan.range_min:
                continue

            if distance > scan.range_max:
                continue

            angle_rad = (
                scan.angle_min
                + i * scan.angle_increment
            )

            angle_deg = math.degrees(
                angle_rad
            )

            robot_angle = self.normalize_angle_deg(
                angle_deg + yaw_offset
            )

            difference = self.normalize_angle_deg(
                robot_angle - center_deg
            )

            if abs(difference) <= width_deg:
                values.append(distance)

        if not values:
            return float('inf')

        values.sort()

        if mode == 'low':

            index = int(
                len(values) * 0.20
            )

            index = min(
                max(index, 0),
                len(values) - 1
            )

            return values[index]

        return statistics.median(values)

    # ================================================================
    # LiDAR Callback
    # ================================================================

    def scan_callback(self, msg):

        self.front = self.get_sector(
            msg,
            center_deg=0.0,
            width_deg=18.0,
            mode='low'
        )

        self.front_right = self.get_sector(
            msg,
            center_deg=-45.0,
            width_deg=15.0,
            mode='low'
        )

        self.right = self.get_sector(
            msg,
            center_deg=-90.0,
            width_deg=15.0,
            mode='median'
        )

        self.last_scan_time = (
            self.get_clock().now()
        )

    # ================================================================
    # Vision
    # ================================================================

    def vision_stop_callback(self, msg):

        self.vision_stop = bool(
            msg.data
        )

        self.last_vision_time = (
            self.get_clock().now()
        )

    def vision_ready_callback(self, msg):

        self.vision_ready = bool(
            msg.data
        )

    # ================================================================
    # Motor Command
    # ================================================================

    def publish_velocity(
        self,
        linear,
        angular
    ):

        msg = Twist()

        msg.linear.x = float(linear)
        msg.angular.z = float(angular)

        self.cmd_pub.publish(msg)

    def stop_robot(self):

        self.publish_velocity(
            0.0,
            0.0
        )

    # ================================================================
    # State Log
    # ================================================================

    def set_state(
        self,
        state,
        text
    ):

        if state != self.previous_state:

            self.previous_state = state

            self.get_logger().info(
                text
            )

    # ================================================================
    # Current Position / Lap Tracking
    # ================================================================

    def update_lap_tracking(self):

        try:

            transform = (
                self.tf_buffer.lookup_transform(
                    'map',
                    'base_link',
                    Time()
                )
            )

        except TransformException:

            return False

        x = (
            transform
            .transform
            .translation
            .x
        )

        y = (
            transform
            .transform
            .translation
            .y
        )

        current_pose = (
            x,
            y
        )

        # ------------------------------------------------------------
        # First valid localization
        # ------------------------------------------------------------

        if self.start_pose is None:

            self.start_pose = current_pose

            self.previous_pose = current_pose

            self.start_time = (
                self.get_clock().now()
            )

            self.get_logger().info(
                (
                    'MAP-RUN START POSE | '
                    f'x={x:.3f}, y={y:.3f}'
                )
            )

            return False

        # ------------------------------------------------------------
        # Travelled Distance
        # ------------------------------------------------------------

        step_distance = math.hypot(
            x - self.previous_pose[0],
            y - self.previous_pose[1]
        )

        # TF 순간 점프는 주행거리에서 제외
        if step_distance < 0.50:

            self.travelled_distance += (
                step_distance
            )

        self.previous_pose = (
            current_pose
        )

        # ------------------------------------------------------------
        # Distance from Start
        # ------------------------------------------------------------

        distance_from_start = math.hypot(
            x - self.start_pose[0],
            y - self.start_pose[1]
        )

        # ------------------------------------------------------------
        # Robot must first leave start area
        # ------------------------------------------------------------

        leave_radius = float(
            self.get_parameter(
                'leave_start_radius'
            ).value
        )

        if (
            not self.left_start_area
            and distance_from_start
            >= leave_radius
        ):

            self.left_start_area = True

            self.get_logger().info(
                (
                    'Robot left start area | '
                    f'distance={distance_from_start:.2f} m'
                )
            )

        # ------------------------------------------------------------
        # Elapsed Time
        # ------------------------------------------------------------

        elapsed = (
            (
                self.get_clock().now()
                - self.start_time
            ).nanoseconds
            / 1e9
        )

        # ------------------------------------------------------------
        # Lap Conditions
        # ------------------------------------------------------------

        min_distance = float(
            self.get_parameter(
                'min_lap_distance'
            ).value
        )

        return_radius = float(
            self.get_parameter(
                'return_radius'
            ).value
        )

        min_time = float(
            self.get_parameter(
                'min_lap_time'
            ).value
        )

        lap_condition = (
            self.left_start_area
            and elapsed >= min_time
            and self.travelled_distance >= min_distance
            and distance_from_start <= return_radius
        )

        # ------------------------------------------------------------
        # Periodic Log
        # ------------------------------------------------------------

        self.log_counter += 1

        if self.log_counter >= 10:

            self.log_counter = 0

            self.get_logger().info(
                (
                    'MAP-RUN | '
                    f'position=({x:.2f},{y:.2f}) | '
                    f'travel={self.travelled_distance:.2f} m | '
                    f'from_start={distance_from_start:.2f} m | '
                    f'time={elapsed:.1f} s'
                )
            )

        return lap_condition

    # ================================================================
    # Save Current Map + Current Trajectory
    # ================================================================

    def start_mapping_save(self):

        if self.save_started:
            return

        self.save_started = True

        save_script = str(
            self.get_parameter(
                'save_script'
            ).value
        )

        self.get_logger().info(
            '=========================================='
        )

        self.get_logger().info(
            '1st Run Lap Completed!'
        )

        self.get_logger().info(
            'ROBOT STOPPED'
        )

        self.get_logger().info(
            'Saving CURRENT Cartographer map + trajectory...'
        )

        self.get_logger().info(
            '=========================================='
        )

        try:

            self.save_process = subprocess.Popen(
                [
                    'python3',
                    save_script
                ]
            )

        except Exception as exc:

            self.get_logger().error(
                f'Failed to start save_mapping_result.py: {exc}'
            )

            self.save_process = None

    # ================================================================
    # Check Save Process
    # ================================================================

    def check_save_process(self):

        if self.save_process is None:
            return

        result = self.save_process.poll()

        # 아직 실행 중
        if result is None:
            return

        if self.save_finished:
            return

        self.save_finished = True

        if result == 0:

            self.get_logger().info(
                '=========================================='
            )

            self.get_logger().info(
                'MAP-RUN SAVE COMPLETE'
            )

            self.get_logger().info(
                'Map: /home/mice/ugv02_slam_test/maps/factory_map.pbstream'
            )

            self.get_logger().info(
                'Path: /home/mice/ugv02_slam_test/maps/patrol_path.csv'
            )

            self.get_logger().info(
                'This patrol_path.csv is from THIS run.'
            )

            self.get_logger().info(
                '=========================================='
            )

            finished_msg = Bool()

            finished_msg.data = True

            self.map_run_finished_pub.publish(
                finished_msg
            )

        else:

            self.get_logger().error(
                (
                    'save_mapping_result.py failed | '
                    f'return code={result}'
                )
            )

    # ================================================================
    # Main Control Loop
    # ================================================================

    def control_loop(self):

        # ------------------------------------------------------------
        # If lap already finished, stay stopped
        # ------------------------------------------------------------

        if self.lap_finished:

            self.stop_robot()

            self.check_save_process()

            return

        # ------------------------------------------------------------
        # Start Enable
        # ------------------------------------------------------------

        if not bool(
            self.get_parameter(
                'start_enabled'
            ).value
        ):

            self.set_state(
                'DISABLED',
                'MAP-RUN DISABLED -> STOP'
            )

            self.stop_robot()

            return

        # ------------------------------------------------------------
        # Vision Ready
        # ------------------------------------------------------------

        if (
            not self.vision_ready
            or self.last_vision_time is None
        ):

            self.set_state(
                'VISION_WAIT',
                'WAITING FOR VISION -> STOP'
            )

            self.stop_robot()

            return

        vision_age = (
            (
                self.get_clock().now()
                - self.last_vision_time
            ).nanoseconds
            / 1e9
        )

        vision_timeout = float(
            self.get_parameter(
                'vision_timeout'
            ).value
        )

        if vision_age > vision_timeout:

            self.set_state(
                'VISION_TIMEOUT',
                'VISION TIMEOUT -> STOP'
            )

            self.stop_robot()

            return

        if self.vision_stop:

            self.set_state(
                'VISION_STOP',
                'YOLO HAZARD -> STOP'
            )

            self.stop_robot()

            return

        # ------------------------------------------------------------
        # LiDAR Ready
        # ------------------------------------------------------------

        if self.last_scan_time is None:

            self.set_state(
                'SCAN_WAIT',
                'WAITING FOR LIDAR -> STOP'
            )

            self.stop_robot()

            return

        scan_age = (
            (
                self.get_clock().now()
                - self.last_scan_time
            ).nanoseconds
            / 1e9
        )

        if scan_age > 0.8:

            self.set_state(
                'SCAN_TIMEOUT',
                'LIDAR TIMEOUT -> STOP'
            )

            self.stop_robot()

            return

        # ------------------------------------------------------------
        # Lap Detection
        # ------------------------------------------------------------

        if self.update_lap_tracking():

            self.lap_finished = True

            self.stop_robot()

            self.start_mapping_save()

            return

        # ------------------------------------------------------------
        # Wall Follow Parameters
        # ------------------------------------------------------------

        target = float(
            self.get_parameter(
                'target_wall_distance'
            ).value
        )

        front_turn = float(
            self.get_parameter(
                'front_turn_distance'
            ).value
        )

        emergency = float(
            self.get_parameter(
                'emergency_distance'
            ).value
        )

        wall_lost = float(
            self.get_parameter(
                'wall_lost_distance'
            ).value
        )

        cruise_speed = float(
            self.get_parameter(
                'linear_speed'
            ).value
        )

        kp = float(
            self.get_parameter(
                'kp'
            ).value
        )

        max_angular = float(
            self.get_parameter(
                'max_angular_speed'
            ).value
        )

        corner_turn = float(
            self.get_parameter(
                'corner_turn_speed'
            ).value
        )

        # ------------------------------------------------------------
        # Wall Follow Control
        # ------------------------------------------------------------

        if self.front < emergency:

            linear = 0.0
            angular = corner_turn

            state = 'EMERGENCY_TURN'

        elif (
            self.front < front_turn
            or self.front_right < 0.35
        ):

            linear = 0.03
            angular = corner_turn

            state = 'CORNER_TURN'

        elif (
            not math.isfinite(self.right)
            or self.right > wall_lost
        ):

            linear = 0.08
            angular = -0.35

            state = 'SEARCH_WALL'

        else:

            error = (
                target - self.right
            )

            angular = (
                kp * error
            )

            angular = max(
                -max_angular,
                min(
                    max_angular,
                    angular
                )
            )

            speed_scale = (
                1.0
                - 0.55
                * abs(angular)
                / max_angular
            )

            linear = max(
                0.07,
                cruise_speed
                * speed_scale
            )

            state = 'WALL_FOLLOW'

        self.previous_state = state

        self.publish_velocity(
            linear,
            angular
        )

    # ================================================================
    # Shutdown
    # ================================================================

    def close(self):

        for _ in range(5):

            self.stop_robot()


def main(args=None):

    rclpy.init(
        args=args
    )

    node = None

    try:

        node = WallFollowNode()

        rclpy.spin(
            node
        )

    except KeyboardInterrupt:

        pass

    finally:

        if node is not None:

            node.close()

            node.destroy_node()

        if rclpy.ok():

            rclpy.shutdown()


if __name__ == '__main__':
    main()