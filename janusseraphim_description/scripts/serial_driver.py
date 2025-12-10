#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import serial
import math
import sys

class SerialJointDriver(Node):
    def __init__(self):
        super().__init__('serial_joint_driver')
        
        self.publish_rate = self.declare_parameter('publish_rate_hz', 60.0).value
        self.publish_rate = max(1.0, float(self.publish_rate))
        self.latest_values = None

        self.use_dummy = False
        # Simple argument parsing
        if '--use-dummy' in sys.argv:
            try:
                idx = sys.argv.index('--use-dummy')
                if idx + 1 < len(sys.argv) and sys.argv[idx+1].lower() == 'true':
                    self.use_dummy = True
            except:
                pass

        if self.use_dummy:
            self.get_logger().info("Simulation Mode: Generating dummy data")
            self.ser = None
            self.start_time = 0.0 # Will be set in first callback or just use current time
        else:
            # Configure Serial Port (Adjust '/dev/ttyACM0' as needed)
            try:
                self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
                self.get_logger().info("Connected to Arduino on /dev/ttyACM0")
            except Exception as e:
                self.get_logger().error(f"Failed to connect to serial: {e}")
                self.ser = None

        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)

        # Define the joints you want to control
        self.joint_names = [
            'base_to_rotator_joint', 
            'rotator_to_shoulder_joint', 
            'shoulder_to_elbow_joint', 
            'elbow_to_wrist_joint',
            'wrist_to_ee_joint',
            'syringe_slider_joint',
            'elbow_to_elbow_servo_gear_joint',
            'elbow_to_elbow_gear_joint',
            'ee_gear_joint'
        ]

    def map_val(self, x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    def timer_callback(self):
        values = []
        if self.use_dummy:
            t = self.get_clock().now().nanoseconds / 1e9
            values = [
                512 + 400 * math.sin(t),
                512 + 400 * math.sin(t + 1),
                512 + 400 * math.sin(t + 2),
                512 + 400 * math.sin(t + 3),
                512 + 400 * math.sin(t + 4),
                512 + 400 * math.sin(t + 5)
            ]
        else:
            serial_values = self._drain_serial_buffer()
            if serial_values:
                self.latest_values = serial_values
            values = self.latest_values or []

        if len(values) >= 6:
            try:
                msg = JointState()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.name = self.joint_names
                
                # Map 0-1023 (Arduino) to Radians (URDF limits)
                # Adjust min/max radians based on your URDF limits
                j1 = self.map_val(float(values[0]), 0, 1023, -3.14, 3.14)
                j2 = self.map_val(float(values[1]), 0, 1023, 0.0, 3.14)
                j3 = self.map_val(float(values[2]), 0, 1023, -3.14, 0.0)
                j4 = self.map_val(float(values[3]), 0, 1023, -1.57, 1.57)
                j5 = self.map_val(float(values[4]), 0, 1023, -1.57, 1.57)
                j6 = self.map_val(float(values[5]), 0, 1023, -0.05, 0.05) # Prismatic joint
                
                # Coupled joints (move with elbow_to_wrist_joint)
                j_gear1 = j4
                j_gear2 = j4
                
                # Coupled ee_gear (moves with syringe_slider_joint)
                # Map slider range (-0.05 to 0.05) to gear range (-1.57 to 1.57)
                j_ee_gear = self.map_val(j6, -0.05, 0.05, -1.57, 1.57)

                msg.position = [j1, j2, j3, j4, j5, j6, j_gear1, j_gear2, j_ee_gear]
                self.publisher_.publish(msg)
                    
            except ValueError:
                pass
            except Exception as e:
                self.get_logger().warn(f"Serial read error: {e}")

    def _drain_serial_buffer(self):
        if self.ser is None:
            return None
        latest = None
        try:
            while self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if not line:
                    continue
                parts = line.split(',')
                if len(parts) >= 6:
                    latest = parts
        except Exception as exc:
            self.get_logger().warn(f"Serial drain error: {exc}")
        return latest

def main(args=None):
    rclpy.init(args=args)
    node = SerialJointDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
