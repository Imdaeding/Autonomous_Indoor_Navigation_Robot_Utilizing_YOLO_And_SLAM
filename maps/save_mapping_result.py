#!/usr/bin/env python3

import csv
import math
import os
from pathlib import Path

import rclpy
from rclpy.node import Node

from cartographer_ros_msgs.srv import (
    FinishTrajectory,
    GetTrajectoryStates,
    TrajectoryQuery,
    WriteState,
)


class MappingResultSaver(Node):

    def __init__(self):

        super().__init__('mapping_result_saver')

        # ============================================================
        # Parameters
        # ============================================================

        self.declare_parameter(
            'output_dir',
            '/home/mice/ugv02_slam_test/maps'
        )

        self.declare_parameter(
            'map_name',
            'factory_map'
        )

        # -1 = 현재 ACTIVE trajectory 자동 선택
        self.declare_parameter(
            'trajectory_id',
            -1
        )

        # waypoint 간 최소 거리
        self.declare_parameter(
            'min_waypoint_spacing',
            0.20
        )

        self.output_dir = Path(
            self.get_parameter(
                'output_dir'
            ).value
        )

        self.map_name = str(
            self.get_parameter(
                'map_name'
            ).value
        )

        self.requested_trajectory_id = int(
            self.get_parameter(
                'trajectory_id'
            ).value
        )

        self.min_spacing = float(
            self.get_parameter(
                'min_waypoint_spacing'
            ).value
        )

        self.output_dir.mkdir(
            parents=True,
            exist_ok=True
        )

        self.pbstream_path = (
            self.output_dir
            / f'{self.map_name}.pbstream'
        )

        self.path_csv_path = (
            self.output_dir
            / 'patrol_path.csv'
        )

        # ============================================================
        # Service Clients
        # ============================================================

        self.states_client = self.create_client(
            GetTrajectoryStates,
            '/get_trajectory_states'
        )

        self.finish_client = self.create_client(
            FinishTrajectory,
            '/finish_trajectory'
        )

        self.query_client = self.create_client(
            TrajectoryQuery,
            '/trajectory_query'
        )

        self.write_client = self.create_client(
            WriteState,
            '/write_state'
        )

    # ================================================================
    # Wait for Service
    # ================================================================

    def wait_for_service(
        self,
        client,
        service_name,
        timeout=10.0
    ):

        self.get_logger().info(
            f'Waiting for {service_name}...'
        )

        if not client.wait_for_service(
            timeout_sec=timeout
        ):

            raise RuntimeError(
                f'Service not available: '
                f'{service_name}'
            )

    # ================================================================
    # Call Service
    # ================================================================

    def call_service(
        self,
        client,
        request
    ):

        future = client.call_async(
            request
        )

        rclpy.spin_until_future_complete(
            self,
            future
        )

        if future.result() is None:

            raise RuntimeError(
                'Service call failed'
            )

        return future.result()

    # ================================================================
    # Quaternion -> Yaw
    # ================================================================

    @staticmethod
    def quaternion_to_yaw(q):

        siny_cosp = (
            2.0
            * (
                q.w * q.z
                +
                q.x * q.y
            )
        )

        cosy_cosp = (
            1.0
            -
            2.0
            * (
                q.y * q.y
                +
                q.z * q.z
            )
        )

        return math.atan2(
            siny_cosp,
            cosy_cosp
        )

    # ================================================================
    # Find Trajectory
    # ================================================================

    def find_trajectory(self):

        self.wait_for_service(
            self.states_client,
            '/get_trajectory_states'
        )

        request = (
            GetTrajectoryStates.Request()
        )

        response = self.call_service(
            self.states_client,
            request
        )

        ids = list(
            response
            .trajectory_states
            .trajectory_id
        )

        states = list(
            response
            .trajectory_states
            .trajectory_state
        )

        if len(ids) == 0:

            raise RuntimeError(
                'No Cartographer trajectory found.'
            )

        self.get_logger().info(
            f'Trajectory IDs: {ids}'
        )

        self.get_logger().info(
            f'Trajectory states: {states}'
        )

        # ------------------------------------------------------------
        # 사용자가 ID를 직접 지정한 경우
        # ------------------------------------------------------------

        if self.requested_trajectory_id >= 0:

            if (
                self.requested_trajectory_id
                not in ids
            ):

                raise RuntimeError(
                    'Requested trajectory ID '
                    'does not exist.'
                )

            index = ids.index(
                self.requested_trajectory_id
            )

            return (
                ids[index],
                states[index]
            )

        # ------------------------------------------------------------
        # ACTIVE trajectory 우선 선택
        #
        # ACTIVE = 0
        # ------------------------------------------------------------

        for trajectory_id, state in zip(
            ids,
            states
        ):

            if state == 0:

                return (
                    trajectory_id,
                    state
                )

        # ACTIVE가 없으면 마지막 trajectory 사용
        return (
            ids[-1],
            states[-1]
        )

    # ================================================================
    # Finish Trajectory
    # ================================================================

    def finish_trajectory(
        self,
        trajectory_id,
        trajectory_state
    ):

        # ACTIVE = 0
        if trajectory_state != 0:

            self.get_logger().info(
                f'Trajectory {trajectory_id} '
                f'is already not ACTIVE. '
                f'Skip finish.'
            )

            return

        self.wait_for_service(
            self.finish_client,
            '/finish_trajectory'
        )

        request = (
            FinishTrajectory.Request()
        )

        request.trajectory_id = (
            trajectory_id
        )

        response = self.call_service(
            self.finish_client,
            request
        )

        message = getattr(
            response.status,
            'message',
            ''
        )

        self.get_logger().info(
            f'Finish trajectory response: '
            f'{message}'
        )

    # ================================================================
    # Get Final Optimized Trajectory
    # ================================================================

    def get_trajectory(
        self,
        trajectory_id
    ):

        self.wait_for_service(
            self.query_client,
            '/trajectory_query'
        )

        request = (
            TrajectoryQuery.Request()
        )

        request.trajectory_id = (
            trajectory_id
        )

        response = self.call_service(
            self.query_client,
            request
        )

        poses = list(
            response.trajectory
        )

        if len(poses) < 2:

            raise RuntimeError(
                'Trajectory contains '
                'too few poses.'
            )

        self.get_logger().info(
            f'Optimized trajectory poses: '
            f'{len(poses)}'
        )

        return poses

    # ================================================================
    # Downsample Trajectory
    # ================================================================

    def make_waypoints(
        self,
        poses
    ):

        raw_points = []

        for pose_stamped in poses:

            pose = (
                pose_stamped.pose
            )

            x = float(
                pose.position.x
            )

            y = float(
                pose.position.y
            )

            yaw = (
                self.quaternion_to_yaw(
                    pose.orientation
                )
            )

            raw_points.append(
                (
                    x,
                    y,
                    yaw
                )
            )

        waypoints = [
            raw_points[0]
        ]

        last_x = raw_points[0][0]
        last_y = raw_points[0][1]

        for point in raw_points[1:-1]:

            x = point[0]
            y = point[1]

            distance = math.hypot(
                x - last_x,
                y - last_y
            )

            if distance >= self.min_spacing:

                waypoints.append(
                    point
                )

                last_x = x
                last_y = y

        # 마지막 위치는 반드시 저장
        last_point = raw_points[-1]

        if (
            waypoints[-1][0]
            != last_point[0]
            or
            waypoints[-1][1]
            != last_point[1]
        ):

            waypoints.append(
                last_point
            )

        self.get_logger().info(
            f'Downsampled waypoints: '
            f'{len(waypoints)}'
        )

        return waypoints

    # ================================================================
    # Save CSV
    # ================================================================

    def save_csv(
        self,
        waypoints
    ):

        with open(
            self.path_csv_path,
            'w',
            newline='',
            encoding='utf-8'
        ) as file:

            writer = csv.writer(
                file
            )

            writer.writerow(
                [
                    'index',
                    'x',
                    'y',
                    'yaw'
                ]
            )

            for index, point in enumerate(
                waypoints
            ):

                x = point[0]
                y = point[1]
                yaw = point[2]

                writer.writerow(
                    [
                        index,
                        f'{x:.6f}',
                        f'{y:.6f}',
                        f'{yaw:.6f}'
                    ]
                )

        self.get_logger().info(
            f'Patrol path saved: '
            f'{self.path_csv_path}'
        )

    # ================================================================
    # Save PBStream
    # ================================================================

    def save_pbstream(self):

        self.wait_for_service(
            self.write_client,
            '/write_state'
        )

        if self.pbstream_path.exists():

            self.get_logger().warning(
                f'Existing file will be '
                f'overwritten: '
                f'{self.pbstream_path}'
            )

            self.pbstream_path.unlink()

        request = (
            WriteState.Request()
        )

        request.filename = str(
            self.pbstream_path
        )

        request.include_unfinished_submaps = (
            True
        )

        response = self.call_service(
            self.write_client,
            request
        )

        message = getattr(
            response.status,
            'message',
            ''
        )

        self.get_logger().info(
            f'Write state response: '
            f'{message}'
        )

        if not self.pbstream_path.exists():

            raise RuntimeError(
                'PBStream file was '
                'not created.'
            )

        self.get_logger().info(
            f'PBStream saved: '
            f'{self.pbstream_path}'
        )

    # ================================================================
    # Main Save Process
    # ================================================================

    def run(self):

        self.get_logger().info(
            '======================================'
        )

        self.get_logger().info(
            'Saving Map-Run Result'
        )

        self.get_logger().info(
            '======================================'
        )

        trajectory_id, state = (
            self.find_trajectory()
        )

        self.get_logger().info(
            f'Selected trajectory: '
            f'{trajectory_id}'
        )

        # Final optimization
        self.finish_trajectory(
            trajectory_id,
            state
        )

        # Final optimized trajectory
        poses = self.get_trajectory(
            trajectory_id
        )

        waypoints = self.make_waypoints(
            poses
        )

        self.save_csv(
            waypoints
        )

        self.save_pbstream()

        self.get_logger().info(
            '======================================'
        )

        self.get_logger().info(
            'MAP-RUN SAVE COMPLETE'
        )

        self.get_logger().info(
            f'MAP : {self.pbstream_path}'
        )

        self.get_logger().info(
            f'PATH: {self.path_csv_path}'
        )

        self.get_logger().info(
            '======================================'
        )


def main(args=None):

    rclpy.init(
        args=args
    )

    node = MappingResultSaver()

    try:

        node.run()

    except Exception as e:

        node.get_logger().error(
            f'Save failed: {e}'
        )

        raise

    finally:

        node.destroy_node()

        if rclpy.ok():

            rclpy.shutdown()


if __name__ == '__main__':

    main()