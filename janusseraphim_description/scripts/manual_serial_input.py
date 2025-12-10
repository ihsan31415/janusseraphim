#!/usr/bin/env python3
"""Manual joint-state publisher that mimics the serial ADC interface."""

import sys
from typing import List

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class ManualSerialInput(Node):
    def __init__(self) -> None:
        super().__init__('manual_serial_input')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.joint_names = [
            'base_to_rotator_joint',
            'rotator_to_shoulder_joint',
            'shoulder_to_elbow_joint',
            'elbow_to_wrist_joint',
            'wrist_to_ee_joint',
            'syringe_slider_joint',
            'elbow_to_elbow_servo_gear_joint',
            'elbow_to_elbow_gear_joint',
            'ee_gear_joint',
        ]
        self.get_logger().info(
            'Manual serial input is ready. Enter six numbers (0-1023) separated by spaces or commas.\n'
            "Type 'quit' to exit."
        )

    @staticmethod
    def _map_val(x: float, in_min: float, in_max: float, out_min: float, out_max: float) -> float:
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    def publish_values(self, values: List[float]) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names

        j1 = self._map_val(values[0], 0.0, 1023.0, -3.14, 3.14)
        j2 = self._map_val(values[1], 0.0, 1023.0, 0.0, 3.14)
        j3 = self._map_val(values[2], 0.0, 1023.0, -3.14, 0.0)
        j4 = self._map_val(values[3], 0.0, 1023.0, -1.57, 1.57)
        j5 = self._map_val(values[4], 0.0, 1023.0, -1.57, 1.57)
        j6 = self._map_val(values[5], 0.0, 1023.0, -0.05, 0.05)

        j_gear1 = j4
        j_gear2 = j4
        j_ee_gear = self._map_val(j6, -0.05, 0.05, -1.57, 1.57)

        msg.position = [j1, j2, j3, j4, j5, j6, j_gear1, j_gear2, j_ee_gear]
        self.publisher_.publish(msg)
        self.get_logger().info(
            'Published joint_states: [%s]' % ', '.join(f'{v:.3f}' for v in msg.position)
        )


def parse_line(line: str) -> List[float]:
    tokens = line.replace(',', ' ').split()
    return [float(tok) for tok in tokens]


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ManualSerialInput()

    try:
        while rclpy.ok():
            try:
                line = input('adc values> ').strip()
            except EOFError:
                break

            if not line:
                continue

            if line.lower() in {'q', 'quit', 'exit'}:
                break

            try:
                raw_values = parse_line(line)
            except ValueError:
                node.get_logger().warn('Could not parse the numbers, please retry.')
                continue

            if len(raw_values) != 6:
                node.get_logger().warn('Expecting exactly 6 numbers (got %d).', len(raw_values))
                continue

            if not all(0.0 <= v <= 1023.0 for v in raw_values):
                node.get_logger().warn('Values must be between 0 and 1023. Rejecting input.')
                continue

            node.publish_values(raw_values)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
