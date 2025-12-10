#!/usr/bin/env python3
"""Subscribe to joint_states and stream corresponding ADC values to a serial device."""

import argparse
import sys
from typing import Dict, List, Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

try:
    import serial
except ImportError:  # pragma: no cover - serial is expected in runtime env
    serial = None

JOINT_CONFIG: List[Tuple[str, Tuple[float, float]]] = [
    ('base_to_rotator_joint', (-3.14, 3.14)),
    ('rotator_to_shoulder_joint', (0.0, 3.14)),
    ('shoulder_to_elbow_joint', (-3.14, 0.0)),
    ('elbow_to_wrist_joint', (-1.57, 1.57)),
    ('wrist_to_ee_joint', (-1.57, 1.57)),
    ('syringe_slider_joint', (-0.05, 0.05)),
]


class JointStateSerialWriter(Node):
    def __init__(self, port: str, baud: int, dry_run: bool) -> None:
        super().__init__('joint_state_serial_writer')
        self.port = port
        self.baud = baud
        self.dry_run = dry_run or serial is None
        self.serial_conn = None
        if not self.dry_run:
            try:
                self.serial_conn = serial.Serial(self.port, self.baud, timeout=1)
                self.get_logger().info(f'Connected to {self.port} @ {self.baud} baud')
            except Exception as exc:  # pragma: no cover - serial errors happen at runtime
                self.get_logger().error(f'Failed to open serial port {self.port}: {exc}')
                self.dry_run = True
        if self.dry_run:
            if serial is None:
                self.get_logger().warn('pyserial is not installed. Running in dry-run mode.')
            else:
                self.get_logger().warn('Running in dry-run mode (no serial writes).')

        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10,
        )
        self.last_publish = None

    @staticmethod
    def encode_adc(value: float, low: float, high: float) -> int:
        """Map from a physical range (rad or meters) back to 0-1023 ADC units."""
        if high == low:
            return 0
        ratio = (value - low) / (high - low)
        ratio = max(0.0, min(1.0, ratio))
        return int(round(ratio * 1023.0))

    def joint_state_callback(self, msg: JointState) -> None:
        name_to_index: Dict[str, int] = {name: idx for idx, name in enumerate(msg.name)}
        if not all(name in name_to_index for name, _ in JOINT_CONFIG):
            missing = [name for name, _ in JOINT_CONFIG if name not in name_to_index]
            self.get_logger().debug(f'Missing joints in message: {missing}')
            return

        raw_values: List[int] = []
        for joint_name, limits in JOINT_CONFIG:
            idx = name_to_index[joint_name]
            raw = self.encode_adc(msg.position[idx], limits[0], limits[1])
            raw_values.append(raw)

        line = ','.join(str(v) for v in raw_values) + '\n'
        if self.dry_run:
            if line != self.last_publish:
                self.get_logger().info(f'[dry-run] would send: {line.strip()}')
                self.last_publish = line
            return

        try:
            self.serial_conn.write(line.encode('utf-8'))
        except Exception as exc:  # pragma: no cover - depends on hardware
            self.get_logger().error(f'Failed to write serial data: {exc}')


def main() -> None:
    parser = argparse.ArgumentParser(description='Forward joint_states to serial hardware.')
    parser.add_argument('--port', default='/dev/ttyACM0', help='Serial device path')
    parser.add_argument('--baud', type=int, default=115200, help='Serial baud rate')
    parser.add_argument('--dry-run', action='store_true', help='Do not write to serial, just log lines')
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = JointStateSerialWriter(args.port, args.baud, args.dry_run)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.serial_conn:
            node.serial_conn.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
