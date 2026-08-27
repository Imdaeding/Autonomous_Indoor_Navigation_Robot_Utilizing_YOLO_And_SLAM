#!/usr/bin/env python3

import csv
import math
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path as NavPath
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

from tf2_ros import Buffer, TransformListener, TransformException


class PathFollowNode(Node):

    def __init__(self):
        super().__init__('path_follow_node')

        # ============================================================
        # Parameters
        # ============================================================

        # 직전 1차 Map-Run에서 저장된 실제 주행 경로
        self.declare_parameter(
            'path_file',
            '/home/mice/ugv02_slam_test/maps/patrol_path.csv'
        )

        # 안전상 Patrol 시작 시 OFF
        self.declare_parameter(
            'start_enabled',
            False
        )

        # 기본 직진 속도
        self.declare_parameter(
            'linear_speed',
            0.10
        )

        # Pure Pursuit Lookahead Distance
        self.declare_parameter(
            'lookahead_distance',
            0.35
        )

        # 최대 각속도
        self.declare_parameter(
            'max_angular_speed',
            0.60
        )

        # 최종 waypoint 도착 판정
        self.declare_parameter(
            'goal_tolerance',
            0.25
        )

        # 최초 위치 검증
        self.declare_parameter(
            'max_start_distance',
            0.75
        )

        # 최초 방향 검증
        self.declare_parameter(
            'max_start_yaw_error_deg',
            50.0
        )

        # 방향이 너무 크게 틀어진 경우
        # 전진하지 않고 제자리 회전
        self.declare_parameter(
            'heading_stop_angle_deg',
            55.0
        )

        # LiDAR 정면 긴급정지
        self.declare_parameter(
            'emergency_distance',
            0.25
        )

        self.declare_parameter(
            'lidar_yaw_offset_deg',
            0.0
        )

        # YOLO heartbeat timeout
        self.declare_parameter(
            'vision_timeout',
            2.0
        )

        # 한 바퀴 후 반복 여부
        self.declare_parameter(
            'loop_path',
            False
        )

        # ============================================================
        # Load Patrol Path
        # ============================================================

        path_file = str(
            self.get_parameter(
                'path_file'
            ).value
        )

        self.waypoints = self.load_path(
            path_file
        )

        if len(self.waypoints) < 2:

            raise RuntimeError(
                'Patrol path must contain at least 2 waypoints.'
            )

        self.current_index = 0

        self.path_finished = False

        self.start_validated = False

        # ============================================================
        # LiDAR State
        # ============================================================

        self.front_distance = float('inf')

        self.last_scan_time = None

        # ============================================================
        # Vision Fail-safe
        # ============================================================

        # YOLO가 준비되기 전까지 무조건 STOP
        self.vision_ready = False

        self.vision_stop = True

        self.last_vision_time = None

        # ============================================================
        # Publishers
        # ============================================================

        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.path_pub = self.create_publisher(
            NavPath,
            '/patrol_path',
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
        # Timers
        # ============================================================

        # 주행 제어 10 Hz
        self.control_timer = self.create_timer(
            0.1,
            self.control_loop
        )

        # RViz Path 표시 1 Hz
        self.path_timer = self.create_timer(
            1.0,
            self.publish_path
        )

        # ============================================================
        # Logging
        # ============================================================

        self.previous_state = None

        self.log_counter = 0

        self.get_logger().info(
            '=========================================='
        )

        self.get_logger().info(
            '2nd PATROL PURE PURSUIT NODE READY'
        )

        self.get_logger().info(
            f'Waypoints loaded: {len(self.waypoints)}'
        )

        self.get_logger().info(
            f'Path: {path_file}'
        )

        self.get_logger().info(
            'start_enabled = FALSE'
        )

        self.get_logger().info(
            'ROBOT IS DISARMED'
        )

        self.get_logger().info(
            '=========================================='
        )

    # ================================================================
    # Load CSV Path
    # ================================================================

    def load_path(self, filename):

        path = Path(filename)

        if not path.exists():

            raise FileNotFoundError(
                f'Path file not found: {filename}'
            )

        points = []

        with open(
            path,
            'r',
            newline='',
            encoding='utf-8'
        ) as file:

            reader = csv.DictReader(file)

            if (
                reader.fieldnames is None
                or 'x' not in reader.fieldnames
                or 'y' not in reader.fieldnames
            ):

                raise RuntimeError(
                    'CSV requires x and y columns.'
                )

            for row in reader:

                try:

                    x = float(
                        row['x']
                    )

                    y = float(
                        row['y']
                    )

                    yaw = 0.0

                    if (
                        'yaw' in row
                        and row['yaw'] != ''
                    ):

                        yaw = float(
                            row['yaw']
                        )

                    if (
                        not math.isfinite(x)
                        or not math.isfinite(y)
                        or not math.isfinite(yaw)
                    ):

                        continue

                    points.append(
                        (
                            x,
                            y,
                            yaw
                        )
                    )

                except Exception:

                    continue

        return points

    # ================================================================
    # Utility
    # ================================================================

    @staticmethod
    def normalize_angle(angle):

        return math.atan2(
            math.sin(angle),
            math.cos(angle)
        )

    @staticmethod
    def quaternion_to_yaw(q):

        siny_cosp = (
            2.0
            * (
                q.w * q.z
                + q.x * q.y
            )
        )

        cosy_cosp = (
            1.0
            -
            2.0
            * (
                q.y * q.y
                + q.z * q.z
            )
        )

        return math.atan2(
            siny_cosp,
            cosy_cosp
        )

    @staticmethod
    def distance(
        x1,
        y1,
        x2,
        y2
    ):

        return math.hypot(
            x2 - x1,
            y2 - y1
        )

    # ================================================================
    # LiDAR
    # ================================================================

    def scan_callback(self, msg):

        values = []

        yaw_offset = float(
            self.get_parameter(
                'lidar_yaw_offset_deg'
            ).value
        )

        for i, distance in enumerate(
            msg.ranges
        ):

            if not math.isfinite(distance):
                continue

            if distance < msg.range_min:
                continue

            if distance > msg.range_max:
                continue

            angle = (
                msg.angle_min
                + i * msg.angle_increment
            )

            angle_deg = (
                math.degrees(angle)
                + yaw_offset
            )

            while angle_deg > 180.0:
                angle_deg -= 360.0

            while angle_deg < -180.0:
                angle_deg += 360.0

            # 정면 ±18도
            if abs(angle_deg) <= 18.0:

                values.append(
                    distance
                )

        if not values:

            self.front_distance = float(
                'inf'
            )

        else:

            values.sort()

            # 단일 노이즈 방지를 위해 하위 20% 지점 사용
            index = int(
                len(values) * 0.20
            )

            index = min(
                max(index, 0),
                len(values) - 1
            )

            self.front_distance = (
                values[index]
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
    # Velocity
    # ================================================================

    def publish_velocity(
        self,
        linear,
        angular
    ):

        msg = Twist()

        msg.linear.x = float(
            linear
        )

        msg.angular.z = float(
            angular
        )

        self.cmd_pub.publish(
            msg
        )

    def stop_robot(self):

        self.publish_velocity(
            0.0,
            0.0
        )

    # ================================================================
    # Logging
    # ================================================================

    def set_state(
        self,
        state,
        message
    ):

        if state != self.previous_state:

            self.previous_state = state

            self.get_logger().info(
                message
            )

    # ================================================================
    # Localization
    # ================================================================

    def get_current_pose(self):

        try:

            transform = (
                self.tf_buffer.lookup_transform(
                    'map',
                    'base_link',
                    Time()
                )
            )

        except TransformException:

            return None

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

        yaw = self.quaternion_to_yaw(
            transform
            .transform
            .rotation
        )

        return (
            x,
            y,
            yaw
        )

    # ================================================================
    # Initial Pose Safety Check
    # ================================================================

    def validate_start_pose(
        self,
        x,
        y,
        yaw
    ):

        if self.start_validated:

            return True

        start_x = (
            self.waypoints[0][0]
        )

        start_y = (
            self.waypoints[0][1]
        )

        start_yaw = (
            self.waypoints[0][2]
        )

        position_error = self.distance(
            x,
            y,
            start_x,
            start_y
        )

        max_distance = float(
            self.get_parameter(
                'max_start_distance'
            ).value
        )

        if position_error > max_distance:

            self.set_state(
                'BAD_START_POSITION',
                (
                    'START POSITION MISMATCH -> STOP | '
                    f'distance={position_error:.2f} m'
                )
            )

            return False

        yaw_error = abs(
            self.normalize_angle(
                start_yaw - yaw
            )
        )

        max_yaw_error = math.radians(
            float(
                self.get_parameter(
                    'max_start_yaw_error_deg'
                ).value
            )
        )

        if yaw_error > max_yaw_error:

            self.set_state(
                'BAD_START_YAW',
                (
                    'START DIRECTION MISMATCH -> STOP | '
                    f'error={math.degrees(yaw_error):.1f} deg'
                )
            )

            return False

        self.start_validated = True

        self.get_logger().info(
            (
                'START POSE VALIDATED | '
                f'position error={position_error:.2f} m | '
                f'yaw error={math.degrees(yaw_error):.1f} deg'
            )
        )

        return True

    # ================================================================
    # Update Current Waypoint Index
    # ================================================================

    def update_current_index(
        self,
        x,
        y
    ):

        total = len(
            self.waypoints
        )

        # ------------------------------------------------------------
        # 중요:
        #
        # 마지막 waypoint가 시작점 근처에 있기 때문에
        # 전체 경로에서 nearest search를 하면
        # 출발하자마자 경로 끝부분으로 점프할 수 있다.
        #
        # 따라서 현재 waypoint 이후 최대 10개만 검색한다.
        # ------------------------------------------------------------

        search_end = min(
            total,
            self.current_index + 10
        )

        nearest_index = (
            self.current_index
        )

        nearest_distance = float(
            'inf'
        )

        for i in range(
            self.current_index,
            search_end
        ):

            wx = (
                self.waypoints[i][0]
            )

            wy = (
                self.waypoints[i][1]
            )

            d = self.distance(
                x,
                y,
                wx,
                wy
            )

            if d < nearest_distance:

                nearest_distance = d

                nearest_index = i

        if nearest_index > self.current_index:

            self.current_index = (
                nearest_index
            )

    # ================================================================
    # Pure Pursuit Lookahead Point
    # ================================================================

    def select_lookahead_target(
        self,
        x,
        y
    ):

        self.update_current_index(
            x,
            y
        )

        lookahead = float(
            self.get_parameter(
                'lookahead_distance'
            ).value
        )

        total = len(
            self.waypoints
        )

        target_index = (
            self.current_index
        )

        # ------------------------------------------------------------
        # 현재 waypoint부터 경로를 앞으로 따라가면서
        # 약 lookahead_distance 만큼 떨어진 waypoint를 찾는다.
        # ------------------------------------------------------------

        accumulated_distance = 0.0

        for i in range(
            self.current_index,
            total - 1
        ):

            x1 = (
                self.waypoints[i][0]
            )

            y1 = (
                self.waypoints[i][1]
            )

            x2 = (
                self.waypoints[i + 1][0]
            )

            y2 = (
                self.waypoints[i + 1][1]
            )

            segment = self.distance(
                x1,
                y1,
                x2,
                y2
            )

            accumulated_distance += (
                segment
            )

            target_index = (
                i + 1
            )

            if accumulated_distance >= lookahead:

                break

        return target_index

    # ================================================================
    # Goal Check
    # ================================================================

    def check_goal(
        self,
        x,
        y
    ):

        last_index = (
            len(self.waypoints) - 1
        )

        # ------------------------------------------------------------
        # 시작점과 마지막 점이 가까우므로
        # 실제로 경로 끝부분까지 진행하지 않았다면
        # 최종 도착 판정을 하지 않는다.
        # ------------------------------------------------------------

        if self.current_index < (
            last_index - 2
        ):

            return False

        goal_x = (
            self.waypoints[
                last_index
            ][0]
        )

        goal_y = (
            self.waypoints[
                last_index
            ][1]
        )

        distance_to_goal = (
            self.distance(
                x,
                y,
                goal_x,
                goal_y
            )
        )

        tolerance = float(
            self.get_parameter(
                'goal_tolerance'
            ).value
        )

        if distance_to_goal > tolerance:

            return False

        loop_path = bool(
            self.get_parameter(
                'loop_path'
            ).value
        )

        if loop_path:

            self.current_index = 0

            self.start_validated = False

            self.get_logger().info(
                'PATROL LOOP COMPLETED -> NEXT LOOP'
            )

            return False

        self.path_finished = True

        self.get_logger().info(
            '=========================================='
        )

        self.get_logger().info(
            '2nd PATROL PATH COMPLETED'
        )

        self.get_logger().info(
            'ROBOT STOPPED'
        )

        self.get_logger().info(
            '=========================================='
        )

        return True

    # ================================================================
    # Publish RViz Path
    # ================================================================

    def publish_path(self):

        msg = NavPath()

        msg.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        msg.header.frame_id = (
            'map'
        )

        for x, y, yaw in self.waypoints:

            pose = PoseStamped()

            pose.header = (
                msg.header
            )

            pose.pose.position.x = (
                float(x)
            )

            pose.pose.position.y = (
                float(y)
            )

            pose.pose.position.z = (
                0.0
            )

            pose.pose.orientation.z = (
                math.sin(
                    yaw / 2.0
                )
            )

            pose.pose.orientation.w = (
                math.cos(
                    yaw / 2.0
                )
            )

            msg.poses.append(
                pose
            )

        self.path_pub.publish(
            msg
        )

    # ================================================================
    # Main Control Loop
    # ================================================================

    def control_loop(self):

        # ------------------------------------------------------------
        # 1. Manual ARM
        # ------------------------------------------------------------

        enabled = bool(
            self.get_parameter(
                'start_enabled'
            ).value
        )

        if not enabled:

            self.set_state(
                'DISARMED',
                'PATROL DISARMED -> STOP'
            )

            self.stop_robot()

            return

        # ------------------------------------------------------------
        # 2. Already Finished
        # ------------------------------------------------------------

        if self.path_finished:

            self.stop_robot()

            return

        # ------------------------------------------------------------
        # 3. Vision Ready
        # ------------------------------------------------------------

        if not self.vision_ready:

            self.set_state(
                'VISION_NOT_READY',
                'VISION NOT READY -> STOP'
            )

            self.stop_robot()

            return

        if self.last_vision_time is None:

            self.set_state(
                'VISION_WAIT',
                'WAITING FOR VISION SIGNAL -> STOP'
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
                'VISION SIGNAL TIMEOUT -> STOP'
            )

            self.stop_robot()

            return

        # ------------------------------------------------------------
        # 4. YOLO Hazard
        # ------------------------------------------------------------

        if self.vision_stop:

            self.set_state(
                'VISION_HAZARD',
                'YOLO HAZARD -> STOP'
            )

            self.stop_robot()

            return

        # ------------------------------------------------------------
        # 5. LiDAR Ready
        # ------------------------------------------------------------

        if self.last_scan_time is None:

            self.set_state(
                'NO_SCAN',
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
        # 6. Emergency Obstacle
        # ------------------------------------------------------------

        emergency = float(
            self.get_parameter(
                'emergency_distance'
            ).value
        )

        if (
            self.front_distance
            < emergency
        ):

            self.set_state(
                'LIDAR_EMERGENCY',
                (
                    'LIDAR OBSTACLE -> STOP | '
                    f'front={self.front_distance:.2f} m'
                )
            )

            self.stop_robot()

            return

        # ------------------------------------------------------------
        # 7. Localization
        # ------------------------------------------------------------

        pose = self.get_current_pose()

        if pose is None:

            self.set_state(
                'NO_LOCALIZATION',
                'map -> base_link unavailable -> STOP'
            )

            self.stop_robot()

            return

        x, y, robot_yaw = pose

        # ------------------------------------------------------------
        # 8. Initial Pose Validation
        # ------------------------------------------------------------

        if not self.validate_start_pose(
            x,
            y,
            robot_yaw
        ):

            self.stop_robot()

            return

        # ------------------------------------------------------------
        # 9. Update waypoint position
        # ------------------------------------------------------------

        self.update_current_index(
            x,
            y
        )

        # ------------------------------------------------------------
        # 10. Goal
        # ------------------------------------------------------------

        if self.check_goal(
            x,
            y
        ):

            self.stop_robot()

            return

        # ------------------------------------------------------------
        # 11. Select Pure Pursuit target
        # ------------------------------------------------------------

        target_index = (
            self.select_lookahead_target(
                x,
                y
            )
        )

        target_x = (
            self.waypoints[
                target_index
            ][0]
        )

        target_y = (
            self.waypoints[
                target_index
            ][1]
        )

        dx = (
            target_x - x
        )

        dy = (
            target_y - y
        )

        distance_to_target = (
            math.hypot(
                dx,
                dy
            )
        )

        # ============================================================
        # Gemini 코드에서 가져온 핵심:
        # Pure Pursuit
        # ============================================================

        target_angle = math.atan2(
            dy,
            dx
        )

        alpha = self.normalize_angle(
            target_angle
            - robot_yaw
        )

        speed = float(
            self.get_parameter(
                'linear_speed'
            ).value
        )

        lookahead = float(
            self.get_parameter(
                'lookahead_distance'
            ).value
        )

        max_angular = float(
            self.get_parameter(
                'max_angular_speed'
            ).value
        )

        # ------------------------------------------------------------
        # Pure Pursuit Angular Velocity
        #
        # w = 2 * v * sin(alpha) / Ld
        # ------------------------------------------------------------

        angular = (
            2.0
            * speed
            * math.sin(alpha)
            / max(
                lookahead,
                0.10
            )
        )

        angular = max(
            -max_angular,
            min(
                max_angular,
                angular
            )
        )

        # ------------------------------------------------------------
        # 방향이 너무 틀어져 있으면 먼저 제자리 회전
        # ------------------------------------------------------------

        heading_stop_angle = math.radians(
            float(
                self.get_parameter(
                    'heading_stop_angle_deg'
                ).value
            )
        )

        if abs(alpha) > heading_stop_angle:

            linear = 0.0

            # Pure Pursuit 공식은 v=0이면 w도 0이 되므로
            # 제자리 회전용 각속도를 별도로 생성
            rotate_gain = 1.2

            angular = (
                rotate_gain
                * alpha
            )

            angular = max(
                -max_angular,
                min(
                    max_angular,
                    angular
                )
            )

            state = (
                'ROTATE_TO_PATH'
            )

        else:

            # --------------------------------------------------------
            # Gemini 코드의 코너링 감속 사용
            # --------------------------------------------------------

            if max_angular > 0.0:

                speed_scale = (
                    1.0
                    - 0.50
                    * abs(angular)
                    / max_angular
                )

            else:

                speed_scale = 1.0

            speed_scale = max(
                0.40,
                speed_scale
            )

            linear = (
                speed
                * speed_scale
            )

            state = (
                'PURE_PURSUIT'
            )

        # ------------------------------------------------------------
        # 12. Publish cmd_vel
        # ------------------------------------------------------------

        self.publish_velocity(
            linear,
            angular
        )

        self.previous_state = (
            state
        )

        # ------------------------------------------------------------
        # Status Log
        # ------------------------------------------------------------

        self.log_counter += 1

        if self.log_counter >= 10:

            self.log_counter = 0

            self.get_logger().info(
                (
                    f'{state:15s} | '
                    f'pose=({x:.2f},{y:.2f}) | '
                    f'wp={target_index}/'
                    f'{len(self.waypoints)-1} | '
                    f'target=({target_x:.2f},'
                    f'{target_y:.2f}) | '
                    f'd={distance_to_target:.2f} | '
                    f'alpha={math.degrees(alpha):.1f} deg | '
                    f'front={self.front_distance:.2f} | '
                    f'v={linear:.2f} | '
                    f'w={angular:.2f}'
                )
            )

    # ================================================================
    # Shutdown
    # ================================================================

    def close(self):

        for _ in range(5):

            self.stop_robot()

            time.sleep(
                0.05
            )


def main(args=None):

    rclpy.init(
        args=args
    )

    node = None

    try:

        node = PathFollowNode()

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