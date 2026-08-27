#!/usr/bin/env python3

import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import cv2
import torch
from ultralytics import YOLO

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


# ================================================================
# Jetson IMX219 GStreamer Pipeline
# ================================================================

def gstreamer_pipeline(
    sensor_id,
    capture_width=1280,
    capture_height=720,
    output_width=640,
    output_height=480,
    framerate=30,
    flip_method=0,
):

    return (
        f"nvarguscamerasrc sensor-id={sensor_id} ! "
        f"video/x-raw(memory:NVMM), "
        f"width=(int){capture_width}, "
        f"height=(int){capture_height}, "
        f"format=(string)NV12, "
        f"framerate=(fraction){framerate}/1 ! "
        f"nvvidconv flip-method={flip_method} ! "
        f"video/x-raw, "
        f"width=(int){output_width}, "
        f"height=(int){output_height}, "
        f"format=(string)BGRx ! "
        f"videoconvert ! "
        f"video/x-raw, format=(string)BGR ! "
        f"appsink drop=true max-buffers=1 sync=false"
    )


class YoloSafetyNode(Node):

    def __init__(self):

        super().__init__(
            'yolo_safety_node'
        )

        # ============================================================
        # 1. Parameters
        # ============================================================

        self.declare_parameter(
            'model_path',
            '/home/mice/ugv02_slam_test/test.pt'
        )

        # ------------------------------------------------------------
        # YOLO 1차 confidence
        #
        # cable threshold가 0.40이므로
        # 1차 필터 역시 0.40 이하로 설정해야 함
        # ------------------------------------------------------------

        self.declare_parameter(
            'confidence',
            0.40
        )

        # BOX 최종 threshold
        self.declare_parameter(
            'box_threshold',
            0.50
        )

        # CABLE 최종 threshold
        self.declare_parameter(
            'cable_threshold',
            0.40
        )

        # 몇 프레임 연속 탐지 시 위험 확정?
        self.declare_parameter(
            'hazard_confirm_frames',
            2
        )

        # 위험물이 사라진 후 재출발 대기시간
        self.declare_parameter(
            'clear_wait_sec',
            2.0
        )

        # ============================================================
        # 2. YOLO Model Load
        # ============================================================

        model_path = self.get_parameter(
            'model_path'
        ).value

        if not Path(
            model_path
        ).exists():

            raise FileNotFoundError(
                f'YOLO model not found: '
                f'{model_path}'
            )

        self.get_logger().info(
            f'Loading YOLO model: '
            f'{model_path}'
        )

        self.model = YOLO(
            model_path
        )

        self.get_logger().info(
            f'YOLO classes: '
            f'{self.model.names}'
        )

        # ============================================================
        # 3. GPU / CPU
        # ============================================================

        if torch.cuda.is_available():

            self.device = 0

        else:

            self.device = 'cpu'

        self.get_logger().info(
            f'YOLO device: '
            f'{self.device}'
        )

        # ============================================================
        # 4. Target Classes
        # ============================================================

        self.target_class_ids = set()

        for (
            class_id,
            class_name
        ) in self.model.names.items():

            name = str(
                class_name
            ).strip().lower()

            if name in {
                'box',
                'cable'
            }:

                self.target_class_ids.add(
                    int(class_id)
                )

        if not self.target_class_ids:

            raise RuntimeError(
                'box / cable class '
                'not found in test.pt'
            )

        self.get_logger().info(
            f'Target class IDs: '
            f'{sorted(self.target_class_ids)}'
        )

        # ============================================================
        # 5. Current Settings Log
        # ============================================================

        base_conf = float(
            self.get_parameter(
                'confidence'
            ).value
        )

        box_conf = float(
            self.get_parameter(
                'box_threshold'
            ).value
        )

        cable_conf = float(
            self.get_parameter(
                'cable_threshold'
            ).value
        )

        confirm_frames = int(
            self.get_parameter(
                'hazard_confirm_frames'
            ).value
        )

        clear_wait = float(
            self.get_parameter(
                'clear_wait_sec'
            ).value
        )

        self.get_logger().info(
            '=========================================='
        )

        self.get_logger().info(
            'YOLO SAFETY FILTER SETTINGS'
        )

        self.get_logger().info(
            f'YOLO base confidence : '
            f'{base_conf:.2f}'
        )

        self.get_logger().info(
            f'BOX threshold        : '
            f'{box_conf:.2f}'
        )

        self.get_logger().info(
            f'CABLE threshold      : '
            f'{cable_conf:.2f}'
        )

        self.get_logger().info(
            f'Confirm frames       : '
            f'{confirm_frames}'
        )

        self.get_logger().info(
            f'Clear wait           : '
            f'{clear_wait:.1f} sec'
        )

        self.get_logger().info(
            '=========================================='
        )

        # ============================================================
        # 6. Open Cameras
        # ============================================================

        self.get_logger().info(
            'Opening CAM0...'
        )

        self.cap0 = cv2.VideoCapture(
            gstreamer_pipeline(
                sensor_id=0
            ),
            cv2.CAP_GSTREAMER
        )

        self.get_logger().info(
            'Opening CAM1...'
        )

        self.cap1 = cv2.VideoCapture(
            gstreamer_pipeline(
                sensor_id=1
            ),
            cv2.CAP_GSTREAMER
        )

        # ============================================================
        # 7. Check Cameras
        # ============================================================

        if not self.cap0.isOpened():

            self.cap0.release()
            self.cap1.release()

            raise RuntimeError(
                'CAM0 open failed'
            )

        if not self.cap1.isOpened():

            self.cap0.release()
            self.cap1.release()

            raise RuntimeError(
                'CAM1 open failed'
            )

        # ============================================================
        # 8. Camera Thread Pool
        # ============================================================

        self.camera_executor = (
            ThreadPoolExecutor(
                max_workers=2
            )
        )

        # ============================================================
        # 9. First Frame Test
        # ============================================================

        self.get_logger().info(
            'Testing first frames '
            'from CAM0 and CAM1...'
        )

        future0 = (
            self.camera_executor.submit(
                self.cap0.read
            )
        )

        future1 = (
            self.camera_executor.submit(
                self.cap1.read
            )
        )

        ret0, frame0 = (
            future0.result()
        )

        ret1, frame1 = (
            future1.result()
        )

        if (
            not ret0
            or frame0 is None
        ):

            self.close_cameras()

            raise RuntimeError(
                'CAM0 first frame '
                'read failed'
            )

        if (
            not ret1
            or frame1 is None
        ):

            self.close_cameras()

            raise RuntimeError(
                'CAM1 first frame '
                'read failed'
            )

        self.get_logger().info(
            f'CAM0 first frame OK: '
            f'{frame0.shape[1]}x'
            f'{frame0.shape[0]}'
        )

        self.get_logger().info(
            f'CAM1 first frame OK: '
            f'{frame1.shape[1]}x'
            f'{frame1.shape[0]}'
        )

        # ============================================================
        # 10. First YOLO Inference
        # ============================================================

        confidence = float(
            self.get_parameter(
                'confidence'
            ).value
        )

        self.get_logger().info(
            'Testing first YOLO inference...'
        )

        test_results = (
            self.model.predict(
                source=[
                    frame0,
                    frame1
                ],
                conf=confidence,
                device=self.device,
                verbose=False
            )
        )

        if len(
            test_results
        ) != 2:

            self.close_cameras()

            raise RuntimeError(
                'Initial YOLO '
                'inference failed'
            )

        self.get_logger().info(
            'Initial YOLO inference OK'
        )

        # ============================================================
        # 11. ROS Publishers
        # ============================================================

        self.stop_pub = (
            self.create_publisher(
                Bool,
                '/vision_stop',
                10
            )
        )

        self.ready_pub = (
            self.create_publisher(
                Bool,
                '/vision_ready',
                10
            )
        )

        # ============================================================
        # 12. Safety States
        # ============================================================

        self.stop_active = True

        self.vision_ready = True

        self.clear_start_time = None

        self.hazard_streak = 0

        self.hazard_confirmed = False

        self.camera_error_active = False

        self.yolo_error_active = False

        # ============================================================
        # 13. Timer
        # ============================================================

        self.control_timer = (
            self.create_timer(
                0.05,
                self.inference_loop
            )
        )

        self.get_logger().info(
            '=========================================='
        )

        self.get_logger().info(
            'YOLO SAFETY NODE READY'
        )

        self.get_logger().info(
            'CAM0 READY'
        )

        self.get_logger().info(
            'CAM1 READY'
        )

        self.get_logger().info(
            'BOX >= 0.50'
        )

        self.get_logger().info(
            'CABLE >= 0.40'
        )

        self.get_logger().info(
            '2 consecutive frames required'
        )

        self.get_logger().info(
            'Hazard clear delay: 2.0 sec'
        )

        self.get_logger().info(
            '=========================================='
        )

    # ================================================================
    # Publish STOP
    # ================================================================

    def publish_stop(
        self,
        stop
    ):

        msg = Bool()

        msg.data = bool(
            stop
        )

        self.stop_pub.publish(
            msg
        )

    # ================================================================
    # Publish READY
    # ================================================================

    def publish_ready(
        self,
        ready
    ):

        msg = Bool()

        msg.data = bool(
            ready
        )

        self.ready_pub.publish(
            msg
        )

    # ================================================================
    # Class Threshold
    # ================================================================

    def get_class_threshold(
        self,
        class_name
    ):

        name = str(
            class_name
        ).strip().lower()

        if name == 'box':

            return float(
                self.get_parameter(
                    'box_threshold'
                ).value
            )

        if name == 'cable':

            return float(
                self.get_parameter(
                    'cable_threshold'
                ).value
            )

        return 1.0

    # ================================================================
    # Extract Hazard Objects
    # ================================================================

    def get_hazard_objects(
        self,
        result,
        camera_name
    ):

        hazards = []

        if result.boxes is None:

            return hazards

        if len(
            result.boxes
        ) == 0:

            return hazards

        classes = (
            result.boxes.cls
            .cpu()
            .numpy()
            .astype(int)
        )

        confidences = (
            result.boxes.conf
            .cpu()
            .numpy()
        )

        for (
            class_id,
            confidence
        ) in zip(
            classes,
            confidences
        ):

            if (
                class_id
                not in self.target_class_ids
            ):

                continue

            class_name = str(
                self.model.names[
                    class_id
                ]
            ).strip().lower()

            class_threshold = (
                self.get_class_threshold(
                    class_name
                )
            )

            if (
                float(confidence)
                < class_threshold
            ):

                continue

            hazards.append(
                (
                    camera_name,
                    class_name,
                    float(confidence)
                )
            )

        return hazards

    # ================================================================
    # Main Inference Loop
    # ================================================================

    def inference_loop(self):

        # ============================================================
        # Read CAM0 + CAM1
        # ============================================================

        future0 = (
            self.camera_executor.submit(
                self.cap0.read
            )
        )

        future1 = (
            self.camera_executor.submit(
                self.cap1.read
            )
        )

        ret0, frame0 = (
            future0.result()
        )

        ret1, frame1 = (
            future1.result()
        )

        # ============================================================
        # Camera Fail-safe
        # ============================================================

        if (
            not ret0
            or frame0 is None
            or
            not ret1
            or frame1 is None
        ):

            self.vision_ready = False

            self.stop_active = True

            self.clear_start_time = None

            self.hazard_streak = 0

            self.hazard_confirmed = False

            self.publish_ready(
                False
            )

            self.publish_stop(
                True
            )

            if not self.camera_error_active:

                self.get_logger().error(
                    'CAM0/CAM1 frame '
                    'read failed -> STOP'
                )

            self.camera_error_active = True

            return

        if self.camera_error_active:

            self.get_logger().info(
                'CAM0/CAM1 frame recovered'
            )

            self.camera_error_active = False

        # ============================================================
        # YOLO Inference
        # ============================================================

        confidence = float(
            self.get_parameter(
                'confidence'
            ).value
        )

        try:

            results = (
                self.model.predict(
                    source=[
                        frame0,
                        frame1
                    ],
                    conf=confidence,
                    device=self.device,
                    verbose=False
                )
            )

        except Exception as e:

            self.vision_ready = False

            self.stop_active = True

            self.clear_start_time = None

            self.hazard_streak = 0

            self.hazard_confirmed = False

            self.publish_ready(
                False
            )

            self.publish_stop(
                True
            )

            if not self.yolo_error_active:

                self.get_logger().error(
                    f'YOLO inference error '
                    f'-> STOP: {e}'
                )

            self.yolo_error_active = True

            return

        # ============================================================
        # Result Check
        # ============================================================

        if len(
            results
        ) != 2:

            self.vision_ready = False

            self.stop_active = True

            self.clear_start_time = None

            self.hazard_streak = 0

            self.hazard_confirmed = False

            self.publish_ready(
                False
            )

            self.publish_stop(
                True
            )

            return

        if self.yolo_error_active:

            self.get_logger().info(
                'YOLO inference recovered'
            )

            self.yolo_error_active = False

        self.vision_ready = True

        self.publish_ready(
            True
        )

        # ============================================================
        # Extract Hazards
        # ============================================================

        hazards0 = (
            self.get_hazard_objects(
                results[0],
                'CAM0'
            )
        )

        hazards1 = (
            self.get_hazard_objects(
                results[1],
                'CAM1'
            )
        )

        hazards = (
            hazards0
            + hazards1
        )

        strong_hazard_detected = (
            len(hazards) > 0
        )

        # ============================================================
        # Hazard detected
        # ============================================================

        if strong_hazard_detected:

            self.clear_start_time = None

            self.hazard_streak += 1

            required_frames = int(
                self.get_parameter(
                    'hazard_confirm_frames'
                ).value
            )

            if required_frames < 1:

                required_frames = 1

            # 이미 위험 확정
            if self.hazard_confirmed:

                self.stop_active = True

                self.publish_stop(
                    True
                )

                return

            # 아직 연속 프레임 수 부족
            if (
                self.hazard_streak
                < required_frames
            ):

                self.publish_stop(
                    self.stop_active
                )

                return

            # ========================================================
            # Hazard Confirmed
            # ========================================================

            self.hazard_confirmed = True

            self.stop_active = True

            self.publish_stop(
                True
            )

            self.get_logger().warning(
                '=========================================='
            )

            self.get_logger().warning(
                'HAZARD CONFIRMED -> ROBOT STOP'
            )

            self.get_logger().warning(
                f'Consecutive frames: '
                f'{self.hazard_streak}'
            )

            for (
                camera_name,
                class_name,
                confidence_value
            ) in hazards:

                threshold = (
                    self.get_class_threshold(
                        class_name
                    )
                )

                self.get_logger().warning(
                    f'{camera_name} | '
                    f'{class_name} | '
                    f'confidence='
                    f'{confidence_value:.2f} | '
                    f'threshold='
                    f'{threshold:.2f}'
                )

            self.get_logger().warning(
                '=========================================='
            )

            return

        # ============================================================
        # No Hazard
        # ============================================================

        self.hazard_streak = 0

        # ------------------------------------------------------------
        # 기존에 위험 확정 상태였다면
        # clear timer 시작
        # ------------------------------------------------------------

        if self.hazard_confirmed:

            self.hazard_confirmed = False

            self.stop_active = True

            self.clear_start_time = (
                time.monotonic()
            )

            clear_wait_sec = float(
                self.get_parameter(
                    'clear_wait_sec'
                ).value
            )

            self.publish_stop(
                True
            )

            self.get_logger().info(
                'Hazard disappeared'
            )

            self.get_logger().info(
                f'Waiting '
                f'{clear_wait_sec:.1f} '
                f'seconds before restart...'
            )

            return

        # ============================================================
        # Already Driving
        # ============================================================

        if not self.stop_active:

            self.publish_stop(
                False
            )

            return

        # ============================================================
        # Startup / Clear Wait
        # ============================================================

        if self.clear_start_time is None:

            self.clear_start_time = (
                time.monotonic()
            )

            self.publish_stop(
                True
            )

            return

        elapsed = (
            time.monotonic()
            - self.clear_start_time
        )

        clear_wait_sec = float(
            self.get_parameter(
                'clear_wait_sec'
            ).value
        )

        if elapsed < clear_wait_sec:

            self.publish_stop(
                True
            )

            return

        # ============================================================
        # Driving Allowed
        # ============================================================

        self.stop_active = False

        self.clear_start_time = None

        self.publish_stop(
            False
        )

        self.get_logger().info(
            '=========================================='
        )

        self.get_logger().info(
            f'Hazard clear for '
            f'{clear_wait_sec:.1f} seconds'
        )

        self.get_logger().info(
            'DRIVING ALLOWED'
        )

        self.get_logger().info(
            '=========================================='
        )

    # ================================================================
    # Close Cameras
    # ================================================================

    def close_cameras(self):

        try:

            if hasattr(
                self,
                'camera_executor'
            ):

                self.camera_executor.shutdown(
                    wait=True
                )

        except Exception:

            pass

        try:

            if hasattr(
                self,
                'cap0'
            ):

                self.cap0.release()

        except Exception:

            pass

        try:

            if hasattr(
                self,
                'cap1'
            ):

                self.cap1.release()

        except Exception:

            pass

    # ================================================================
    # Close Node
    # ================================================================

    def close(self):

        try:

            for _ in range(5):

                self.publish_stop(
                    True
                )

                self.publish_ready(
                    False
                )

                time.sleep(
                    0.05
                )

        except Exception:

            pass

        self.close_cameras()


def main(args=None):

    rclpy.init(
        args=args
    )

    node = None

    try:

        node = (
            YoloSafetyNode()
        )

        rclpy.spin(
            node
        )

    except KeyboardInterrupt:

        pass

    except Exception as e:

        print(
            f'YOLO Safety Node ERROR: '
            f'{e}'
        )

        raise

    finally:

        if node is not None:

            node.close()

            node.destroy_node()

        if rclpy.ok():

            rclpy.shutdown()


if __name__ == '__main__':

    main()