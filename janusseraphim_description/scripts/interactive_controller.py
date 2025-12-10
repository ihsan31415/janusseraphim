#!/usr/bin/env python3
"""Interactive Marker controller for drag-and-drop end-effector manipulation."""

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, TransformStamped
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker, InteractiveMarkerFeedback
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from rclpy.duration import Duration
import tf2_ros
import serial

class InteractiveController(Node):
    def __init__(self) -> None:
        super().__init__('interactive_controller')

        self.target_link: str = self.declare_parameter('target_link', 'tool0').value
        self.reference_frame: str = self.declare_parameter('reference_frame', 'base_link').value
        self.serial_port: str = self.declare_parameter('serial_port', '/dev/ttyACM0').value
        self.serial_baud: int = int(self.declare_parameter('serial_baud', 115200).value)

        self.tf_buffer = Buffer(cache_time=Duration(seconds=30))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.marker_name = f'{self.target_link}_interactive_marker'
        self.child_frame = f'{self.target_link}_interactive'
        # Default position within arm's reachable workspace (not at origin!)
        self.marker_pose: Pose = Pose()
        self.marker_pose.position.x = 0.15  # forward
        self.marker_pose.position.y = 0.0
        self.marker_pose.position.z = 0.15  # up
        self.marker_pose.orientation.w = 1.0
        self.user_dragging = False

        self.serial_conn: Optional[serial.Serial] = None

        self.server = InteractiveMarkerServer(self, 'interactive_controller_server')
        self.create_marker()
        self.server.applyChanges()

        # Only sync once at startup, then let user control
        self.initial_sync_done = False
        self.sync_timer = self.create_timer(1.0, self.sync_with_robot_pose)
        self.serial_timer = self.create_timer(2.0, self.ensure_serial_connection)
        # Continuously broadcast TF so IK solver can track marker position
        self.tf_timer = self.create_timer(0.02, self.broadcast_current_tf)  # 50 Hz

    def create_marker(self) -> None:
        pose = self.lookup_current_pose()
        if pose:
            self.marker_pose = pose
        marker = InteractiveMarker()
        marker.header.frame_id = self.reference_frame
        marker.name = self.marker_name
        marker.description = f'{self.target_link} interactive control'
        marker.scale = 0.2
        marker.pose.position.x = float(self.marker_pose.position.x)
        marker.pose.position.y = float(self.marker_pose.position.y)
        marker.pose.position.z = float(self.marker_pose.position.z)
        marker.pose.orientation.x = float(self.marker_pose.orientation.x)
        marker.pose.orientation.y = float(self.marker_pose.orientation.y)
        marker.pose.orientation.z = float(self.marker_pose.orientation.z)
        marker.pose.orientation.w = float(self.marker_pose.orientation.w) if self.marker_pose.orientation.w else 1.0

        marker.controls.append(self._make_visual_control())
        self._add_6dof_controls(marker)

        self.server.insert(marker)
        self.server.setCallback(marker.name, self.process_feedback)
        self.publish_interactive_tf(self.marker_pose)

    def _make_visual_control(self) -> InteractiveMarkerControl:
        control = InteractiveMarkerControl()
        control.always_visible = True
        marker = Marker()
        marker.type = Marker.SPHERE
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.r = 0.1
        marker.color.g = 0.6
        marker.color.b = 0.9
        marker.color.a = 0.9
        control.markers.append(marker)
        return control

    def _add_6dof_controls(self, marker: InteractiveMarker) -> None:
        axes = [
            ('rotate_x', (1, 1, 0, 0), InteractiveMarkerControl.ROTATE_AXIS),
            ('move_x',   (1, 1, 0, 0), InteractiveMarkerControl.MOVE_AXIS),
            ('rotate_y', (0, 1, 1, 0), InteractiveMarkerControl.ROTATE_AXIS),
            ('move_y',   (0, 1, 1, 0), InteractiveMarkerControl.MOVE_AXIS),
            ('rotate_z', (0, 0, 1, 1), InteractiveMarkerControl.ROTATE_AXIS),
            ('move_z',   (0, 0, 1, 1), InteractiveMarkerControl.MOVE_AXIS),
        ]
        for name, orientation, mode in axes:
            control = InteractiveMarkerControl()
            control.name = name
            control.orientation.w = float(orientation[0])
            control.orientation.x = float(orientation[1])
            control.orientation.y = float(orientation[2])
            control.orientation.z = float(orientation[3])
            control.interaction_mode = mode
            marker.controls.append(control)

        move_control = InteractiveMarkerControl()
        move_control.name = 'move_3d'
        move_control.interaction_mode = InteractiveMarkerControl.MOVE_3D
        marker.controls.append(move_control)

        rotate_control = InteractiveMarkerControl()
        rotate_control.name = 'rotate_3d'
        rotate_control.interaction_mode = InteractiveMarkerControl.ROTATE_3D
        marker.controls.append(rotate_control)

    def lookup_current_pose(self) -> Optional[Pose]:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.reference_frame,
                self.target_link,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1) # Reduced timeout to avoid blocking
            )
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException, tf2_ros.ConnectivityException):
            # Suppress warning if just starting up, as IK solver will bootstrap it
            # self.get_logger().warn(
            #     f'Could not find transform from {self.reference_frame} to {self.target_link}; using origin.'
            # )
            return None

        pose = Pose()
        pose.position.x = transform.transform.translation.x
        pose.position.y = transform.transform.translation.y
        pose.position.z = transform.transform.translation.z
        pose.orientation.x = transform.transform.rotation.x
        pose.orientation.y = transform.transform.rotation.y
        pose.orientation.z = transform.transform.rotation.z
        pose.orientation.w = transform.transform.rotation.w or 1.0
        return pose

    def process_feedback(self, feedback: InteractiveMarkerFeedback) -> None:
        if feedback.event_type == InteractiveMarkerFeedback.MOUSE_DOWN:
            self.user_dragging = True
        elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            self.user_dragging = False

        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            self.marker_pose = feedback.pose
            self.publish_interactive_tf(self.marker_pose)
            self.send_serial_pose(self.marker_pose)
            self.server.setPose(self.marker_name, self.marker_pose)
            self.server.applyChanges()

    def broadcast_current_tf(self) -> None:
        """Continuously broadcast the marker's current TF so IK solver can track it."""
        self.publish_interactive_tf(self.marker_pose)

    def publish_interactive_tf(self, pose: Pose) -> None:
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.reference_frame
        transform.child_frame_id = self.child_frame
        transform.transform.translation.x = pose.position.x
        transform.transform.translation.y = pose.position.y
        transform.transform.translation.z = pose.position.z
        transform.transform.rotation.x = float(pose.orientation.x)
        transform.transform.rotation.y = float(pose.orientation.y)
        transform.transform.rotation.z = float(pose.orientation.z)
        transform.transform.rotation.w = float(pose.orientation.w) or 1.0
        self.tf_broadcaster.sendTransform(transform)

    def send_serial_pose(self, pose: Pose) -> None:
        if self.serial_conn is None:
            return
        line = f"{pose.position.x:.5f},{pose.position.y:.5f},{pose.position.z:.5f},"
        line += f"{pose.orientation.x:.6f},{pose.orientation.y:.6f},{pose.orientation.z:.6f},{pose.orientation.w:.6f}\n"
        try:
            self.serial_conn.write(line.encode('utf-8'))
        except Exception as exc:
            self.get_logger().error(f'Failed to write serial data: {exc}')
            self.close_serial()

    def ensure_serial_connection(self) -> None:
        if not self.serial_port:
            return  # serial disabled
        if self.serial_conn and self.serial_conn.is_open:
            return
        try:
            self.serial_conn = serial.Serial(self.serial_port, self.serial_baud, timeout=1)
            self.get_logger().info(f'Connected to serial device {self.serial_port} @ {self.serial_baud}')
        except Exception as exc:
            self.serial_conn = None
            self.get_logger().warn(f'Serial connection failed: {exc}')

    def close_serial(self) -> None:
        if self.serial_conn:
            try:
                self.serial_conn.close()
            except Exception:
                pass
            self.serial_conn = None

    def sync_with_robot_pose(self) -> None:
        # Only sync once at startup to get initial position, then stop
        if self.initial_sync_done:
            return
        if self.user_dragging:
            return
        pose = self.lookup_current_pose()
        if not pose:
            return
        self.marker_pose = pose
        self.server.setPose(self.marker_name, pose)
        self.server.applyChanges()
        self.publish_interactive_tf(pose)
        self.initial_sync_done = True
        self.get_logger().info('Initial sync complete, interactive control active')

    def destroy_node(self):  # type: ignore[override]
        self.close_serial()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = InteractiveController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
