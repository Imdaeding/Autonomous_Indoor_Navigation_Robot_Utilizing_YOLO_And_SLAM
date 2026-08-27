#!/usr/bin/env python3

import json
import threading
import time

import serial

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist


class UGV02SerialBridge(Node):

    def __init__(self):

        super().__init__('ugv02_serial_bridge')

        # ============================================================
        # 1. Serial Parameters
        # ============================================================

        # 실제 테스트에서 확인된 UGV02 USB Serial 포트
        self.declare_parameter(
            'port',
            '/dev/ttyACM0'
        )

        # UGV02 기본 Serial baud rate
        self.declare_parameter(
            'baudrate',
            115200
        )

        # /cmd_vel이 이 시간 이상 끊기면 정지
        self.declare_parameter(
            'cmd_timeout',
            0.5
        )

        # ESP32가 보내는 데이터를 터미널에 출력할지
        self.declare_parameter(
            'print_feedback',
            False
        )

        port = self.get_parameter(
            'port'
        ).value

        baudrate = self.get_parameter(
            'baudrate'
        ).value

        # ============================================================
        # 2. USB Serial 연결
        # ============================================================

        self.ser = serial.Serial(
            port,
            baudrate=baudrate,
            timeout=0.1
        )

        self.get_logger().info(
            f'UGV02 USB Serial connected: '
            f'{port} @ {baudrate}'
        )

        # ============================================================
        # 3. 현재 이동 명령
        # ============================================================

        self.linear_x = 0.0
        self.angular_z = 0.0

        self.last_cmd_time = None

        # ============================================================
        # 4. /cmd_vel Subscriber
        # ============================================================

        self.cmd_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            10
        )

        # ============================================================
        # 5. ESP32에 10Hz로 명령 전송
        # ============================================================

        self.timer = self.create_timer(
            0.1,
            self.send_control
        )

        # ============================================================
        # 6. ESP32 수신 Thread
        # ============================================================

        self.running = True

        self.recv_thread = threading.Thread(
            target=self.read_serial,
            daemon=True
        )

        self.recv_thread.start()

        # 시작 시 정지 명령
        self.send_json(
            0.0,
            0.0
        )

    # ================================================================
    # /cmd_vel callback
    # ================================================================

    def cmd_callback(self, msg):

        self.linear_x = msg.linear.x
        self.angular_z = msg.angular.z

        self.last_cmd_time = self.get_clock().now()

    # ================================================================
    # ESP32 JSON 명령 전송
    # ================================================================

    def send_json(
        self,
        x,
        z
    ):

        # UGV02 ROS velocity control command
        command = {
            'T': 13,
            'X': round(float(x), 3),
            'Z': round(float(z), 3)
        }

        message = (
            json.dumps(
                command,
                separators=(',', ':')
            )
            + '\n'
        )

        try:

            self.ser.write(
                message.encode('utf-8')
            )

            self.ser.flush()

        except serial.SerialException as e:

            self.get_logger().error(
                f'Serial write error: {e}'
            )

    # ================================================================
    # 10Hz 주행 명령 전송
    # ================================================================

    def send_control(self):

        # 아직 /cmd_vel을 받은 적이 없으면 정지
        if self.last_cmd_time is None:

            self.send_json(
                0.0,
                0.0
            )

            return

        timeout = self.get_parameter(
            'cmd_timeout'
        ).value

        age = (
            self.get_clock().now()
            - self.last_cmd_time
        ).nanoseconds / 1e9

        # /cmd_vel이 끊긴 경우
        if age > timeout:

            self.send_json(
                0.0,
                0.0
            )

            return

        # 정상 주행
        self.send_json(
            self.linear_x,
            self.angular_z
        )

    # ================================================================
    # ESP32 → Jetson Serial 수신
    # ================================================================

    def read_serial(self):

        while self.running:

            try:

                line = self.ser.readline()

                if not line:
                    continue

                print_feedback = self.get_parameter(
                    'print_feedback'
                ).value

                if print_feedback:

                    text = line.decode(
                        'utf-8',
                        errors='ignore'
                    ).strip()

                    if text:

                        print(
                            f'ESP32: {text}'
                        )

            except serial.SerialException:

                break

    # ================================================================
    # 종료 처리
    # ================================================================

    def close(self):

        self.running = False

        # 종료 시 확실하게 정지
        for _ in range(5):

            try:

                self.send_json(
                    0.0,
                    0.0
                )

                time.sleep(0.05)

            except Exception:

                pass

        if self.ser.is_open:
            self.ser.close()


def main(args=None):

    rclpy.init(args=args)

    node = UGV02SerialBridge()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.close()

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':
    main()