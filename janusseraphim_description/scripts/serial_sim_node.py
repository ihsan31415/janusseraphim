#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class SimulatedSerialNode(Node):
    def __init__(self):
        super().__init__('simulated_serial_node')
        rate = 20.0
        self.amp = 1.0
        self.prismatic_amp = 0.03
        self.center_offset = 0.0
        self.timer = self.create_timer(1.0 / rate, self.publish_state)
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.start_time = self.get_clock().now().nanoseconds / 1e9
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
            "Simulated serial publishing at 20.0 Hz with default amplitudes"
        )

    def map_val(self, x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    def publish_state(self):
        t = self.get_clock().now().nanoseconds / 1e9
        dt = t - self.start_time
        a = self.amp
        p = self.prismatic_amp
        positions = [
            0.5 * a * math.sin(dt),
            1.57 + 0.6 * a * math.sin(dt + 0.7),
            -1.3 + 0.5 * a * math.sin(dt + 1.4),
            0.4 * a * math.sin(dt + 2.1),
            0.3 * a * math.sin(dt + 2.8),
            0.5 * p * math.sin(dt + 3.5) + self.center_offset,
        ]
        j4 = positions[3]
        j6 = positions[5]
        j_gear1 = j4
        j_gear2 = j4
        j_ee_gear = self.map_val(j6, -0.05, 0.05, -1.57, 1.57)

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = positions + [j_gear1, j_gear2, j_ee_gear]
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimulatedSerialNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
