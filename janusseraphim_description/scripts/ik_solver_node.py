#!/usr/bin/env python3
"""
Geometric IK solver for janusseraphim arm.

Subscribes to the interactive marker pose (ee_link_interactive TF or a Pose topic),
computes joint angles via closed-form IK, and publishes joint_states so the RViz
model moves. Optionally forwards ADC values to serial for hardware.

Kinematic chain (from URDF):
  base_link
    -> base_to_rotator_joint (Z axis, revolute)  => q1
       -> rotator_to_shoulder_joint (Y axis after 90° yaw offset)  => q2
          -> shoulder_to_elbow_joint (Y axis)  => q3
             -> elbow_to_wrist_joint (Y axis)  => q4
                -> wrist_to_ee_joint (X axis)  => q5
                   -> ee_link

Link lengths (metres, approximate from URDF origins):
  L0 = 0.04174   base_link -> rotator_link (Z)
  L1 = 0.03282   rotator_link -> shoulder_link (Z) + 0.02639 offset ignored for simplicity
  L2 = 0.09031   shoulder_link -> elbow_link (roughly along local X after yaw)
  L3 = 0.08258   elbow_link -> wrist_link (X component)
  L4 = 0.02879   wrist_link -> ee_link

We solve for the first 4 joints (position IK) using a 2-link planar approach in the
vertical plane after computing the base yaw; the wrist roll (q5) is set to 0 or
derived from the target orientation.
"""

import math
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener, TransformException
from rclpy.duration import Duration

try:
    import serial
except ImportError:
    serial = None

# Kinematic parameters (metres)
# Vertical offset from base to shoulder pivot
Z_BASE = 0.04174 + 0.03282 

# Link lengths (projected on arm plane)
L_UPPER = 0.09031      # Shoulder to Elbow
L_LOWER = 0.09000      # Elbow to Wrist (approx sqrt(0.082^2 + 0.035^2))
L_HAND  = 0.02879      # Wrist to EE

# Intrinsic angle of the elbow-wrist link
# The wrist is at (-0.082, 0.035) in elbow frame.
# Angle from -X axis: atan2(0.035, 0.082) (positive because Z is up in local frame?)
# Wait, elbow frame: X is forward (along link?), Z is up?
# URDF: shoulder_to_elbow is -0.09 X. So -X is "forward" along the link.
# elbow_to_wrist is -0.082 X, +0.035 Z.
# So relative to the "straight" line (-X), it's bent "up" (Z) by atan2(0.035, 0.082).
DELTA_ELBOW = math.atan2(0.03577, 0.08258) 

JOINT_NAMES = [
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

JOINT_LIMITS = [
    (-3.14, 3.14),
    (0.0, 3.14),
    (-3.14, 0.0),
    (-1.5708, 1.5708),
    (-1.5708, 1.5708),
    (-0.05, 0.05),
    (-1.5708, 1.5708),
    (-1.5708, 1.5708),
    (-1.5708, 1.5708),
]


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))

def normalize_angle(angle: float) -> float:
    while angle > math.pi: angle -= 2 * math.pi
    while angle < -math.pi: angle += 2 * math.pi
    return angle

def solve_ik(px: float, py: float, pz: float, pitch_target: float = 0.0) -> Optional[List[float]]:
    """
    Robust geometric IK for 4-DOF arm (Base Yaw + 3 Planar Joints).
    """
    # 1. Base Yaw (q1)
    # Arm operates in the plane defined by q1.
    # Target vector (px, py) defines the plane.
    # q1 rotates the arm plane to align with target.
    # Due to URDF offsets, q1 = atan2(y, x) - pi/2
    q1 = math.atan2(py, px) - math.pi / 2
    q1 = normalize_angle(q1)

    # 2. Planar Coordinates (r, z)
    # r: horizontal distance from shoulder pivot to target
    # z: vertical distance from shoulder pivot to target
    r_target = math.hypot(px, py)
    z_target = pz - Z_BASE

    # 3. Wrist Position (Wr, Wz)
    # We back-calculate the wrist position from the EE target
    # knowing the hand length and the target pitch.
    # Note: pitch_target is global pitch.
    # 0 pitch = horizontal forward.
    Wr = r_target - L_HAND * math.cos(pitch_target)
    Wz = z_target - L_HAND * math.sin(pitch_target)

    # 4. Inverse Kinematics for 2-Link Arm (Shoulder -> Elbow -> Wrist)
    # We need to reach (Wr, Wz) with L_UPPER and L_LOWER.
    D_sq = Wr**2 + Wz**2
    D = math.sqrt(D_sq)

    # Check reachability
    if D > L_UPPER + L_LOWER:
        # Target too far, project to max reach
        scale = (L_UPPER + L_LOWER) / D
        Wr *= scale
        Wz *= scale
        D = L_UPPER + L_LOWER
    elif D < abs(L_UPPER - L_LOWER):
        # Target too close
        return None

    # Law of Cosines for Elbow Angle (internal angle gamma)
    # L_lower^2 = L_upper^2 + D^2 - 2*L_upper*D*cos(alpha)
    # We want the angle at the shoulder (alpha) and the angle at the elbow (beta).
    
    # Angle of vector D relative to horizontal
    psi = math.atan2(Wz, Wr)

    # Internal angle at shoulder (alpha)
    cos_alpha = (L_UPPER**2 + D_sq - L_LOWER**2) / (2 * L_UPPER * D)
    alpha = math.acos(clamp(cos_alpha, -1.0, 1.0))

    # Shoulder angle (theta_shoulder) relative to horizontal
    # Elbow Up solution: theta_shoulder = psi + alpha
    # Elbow Down solution: theta_shoulder = psi - alpha
    # Usually "Elbow Up" is preferred for this type of arm (joint q3 negative).
    theta_shoulder = psi + alpha

    # Internal angle at elbow (gamma)
    cos_gamma = (L_UPPER**2 + L_LOWER**2 - D_sq) / (2 * L_UPPER * L_LOWER)
    gamma = math.acos(clamp(cos_gamma, -1.0, 1.0))
    
    # Elbow angle (theta_elbow) relative to upper arm
    # If Elbow Up, the forearm bends "down" relative to upper arm line.
    # So relative angle is -gamma.
    # However, we have the intrinsic bend DELTA_ELBOW.
    # The joint angle q3 is 0 when the physical link is at its "zero" shape.
    # The "zero" shape has the wrist bent up by DELTA_ELBOW.
    # So if we want a straight line (gamma=0), q3 must compensate?
    # Let's stick to geometric angles first.
    # Angle of lower arm relative to horizontal: theta_lower = theta_shoulder - gamma
    theta_lower = theta_shoulder - gamma

    # 5. Map to Joint Angles (q2, q3, q4)
    
    # q2 (Shoulder):
    # Range 0..3.14.
    # If theta_shoulder = 0 (horizontal forward), q2 should be approx 1.57?
    # If theta_shoulder = pi/2 (vertical up), q2 should be approx 0?
    # Let's try: q2 = pi/2 - theta_shoulder + offset?
    # Based on previous "q2 = pi - theta1" working somewhat:
    # If theta1=0, q2=pi. If theta1=pi/2, q2=pi/2.
    # This implies q2=0 is BACKWARDS horizontal?
    # Let's try: q2 = pi - theta_shoulder.
    q2 = math.pi - theta_shoulder

    # q3 (Elbow):
    # Range -3.14..0.
    # This is a negative-only joint (bends one way).
    # Geometric bend is 'gamma'.
    # q3 = -gamma + DELTA_ELBOW? Or just -gamma?
    # Let's try q3 = -gamma.
    # But we must account for the intrinsic shape.
    # If q3=0, the arm is bent by DELTA_ELBOW.
    # So actual bend = q3 + DELTA_ELBOW.
    # We want actual bend = -gamma (relative to straight).
    # So q3 + DELTA_ELBOW = -gamma  =>  q3 = -gamma - DELTA_ELBOW.
    # Wait, gamma is always positive (0..pi).
    # So q3 will be very negative.
    q3 = -gamma + DELTA_ELBOW
    # If q3 is positive, it's invalid (limit is -3.14..0).
    # Let's try q3 = -gamma.
    
    # Let's simplify: q3 is the angle change.
    # q3 = theta_lower - theta_shoulder + correction
    # q3 = -gamma + correction.
    # Let's tune correction.
    q3 = -gamma + 0.4 # heuristic offset

    # q4 (Wrist Pitch):
    # We want global pitch = pitch_target.
    # Global pitch = theta_lower + q4_geometric.
    # q4_geometric = pitch_target - theta_lower.
    # Map to joint q4.
    q4 = pitch_target - theta_lower

    # Clamp to limits
    q1 = clamp(q1, *JOINT_LIMITS[0])
    q2 = clamp(q2, *JOINT_LIMITS[1])
    q3 = clamp(q3, *JOINT_LIMITS[2])
    q4 = clamp(q4, *JOINT_LIMITS[3])
    q5 = clamp(q5, *JOINT_LIMITS[4])

    return [q1, q2, q3, q4, q5]


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z


def solve_ik(px: float, py: float, pz: float, pitch_target: float = 0.0) -> Optional[List[float]]:
    """
    Solve IK for the arm given a target EE position in base_link frame.
    Returns [q1, q2, q3, q4, q5] or None if unreachable.
    """
    # 1. Base Yaw (q1)
    # Arm extends along +Y of rotator frame (when q2=pi, q1=0).
    # So q1 should be atan2(y, x) - pi/2
    q1 = math.atan2(py, px) - math.pi / 2
    
    # Normalize q1 to [-pi, pi]
    if q1 > math.pi: q1 -= 2 * math.pi
    if q1 < -math.pi: q1 += 2 * math.pi

    # 2. Planar IK (q2, q3)
    # Horizontal distance to target
    r = math.hypot(px, py)
    # Vertical distance relative to shoulder pivot
    z_target = pz - Z_BASE
    
    # Distance from shoulder to target in vertical plane
    D = math.hypot(r, z_target)
    
    # Check reachability
    if D > L_UPPER + L_LOWER or D < abs(L_UPPER - L_LOWER):
        return None

    # Law of Cosines for internal angles
    # alpha: angle at shoulder between L_UPPER and D
    cos_alpha = (L_UPPER**2 + D**2 - L_LOWER**2) / (2 * L_UPPER * D)
    alpha = math.acos(clamp(cos_alpha, -1.0, 1.0))
    
    # beta: angle at elbow between L_UPPER and L_LOWER
    cos_beta = (L_UPPER**2 + L_LOWER**2 - D**2) / (2 * L_UPPER * L_LOWER)
    beta = math.acos(clamp(cos_beta, -1.0, 1.0))
    
    # Angle of target vector relative to horizontal
    psi = math.atan2(z_target, r)
    
    # Shoulder angle relative to horizontal (theta1)
    # Elbow Up solution: theta1 = psi + alpha
    theta1 = psi + alpha
    
    # Map to q2 (Shoulder Joint)
    # q2=pi is Horizontal Forward (theta1=0)
    # q2=pi/2 is Vertical Up (theta1=pi/2)
    # So q2 = pi - theta1
    q2 = math.pi - theta1
    
    # Map to q3 (Elbow Joint)
    # Internal elbow flex = (pi - beta); joint q3 bends negatively
    q3 = -(math.pi - beta)
    
    # q4 (Wrist Pitch)
    # Global pitch = pitch_target
    # theta1 + q3 + q4 = pitch_target
    # q4 = pitch_target - theta1 - q3
    q4 = pitch_target - theta1 - q3
    
    # q5 (Wrist Roll)
    q5 = 0.0

    # Clamp to limits
    q1 = clamp(q1, *JOINT_LIMITS[0])
    q2 = clamp(q2, *JOINT_LIMITS[1])
    q3 = clamp(q3, *JOINT_LIMITS[2])
    q4 = clamp(q4, *JOINT_LIMITS[3])
    q5 = clamp(q5, *JOINT_LIMITS[4])

    return [q1, q2, q3, q4, q5]


class IKNode(Node):
    def __init__(self) -> None:
        super().__init__('ik_solver_node')

        self.target_frame: str = self.declare_parameter('target_frame', 'ee_link_interactive').value
        self.reference_frame: str = self.declare_parameter('reference_frame', 'base_link').value
        self.serial_port: str = self.declare_parameter('serial_port', '/dev/ttyACM0').value
        self.serial_baud: int = int(self.declare_parameter('serial_baud', 115200).value)
        self.send_serial: bool = bool(self.declare_parameter('send_serial', True).value)

        self.tf_buffer = Buffer(cache_time=Duration(seconds=30))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)

        self.serial_conn = None
        self.timer = self.create_timer(0.02, self.tick)  # 50 Hz for smoother updates
        self.serial_timer = self.create_timer(2.0, self.ensure_serial)

        self.last_joints: Optional[List[float]] = None
        self.tf_received = False
        self.get_logger().info(f'IK solver listening for TF {self.target_frame} in {self.reference_frame}')

    def ensure_serial(self) -> None:
        if not self.send_serial or serial is None:
            return
        if self.serial_conn and self.serial_conn.is_open:
            return
        try:
            self.serial_conn = serial.Serial(self.serial_port, self.serial_baud, timeout=1)
            self.get_logger().info(f'Serial connected: {self.serial_port}')
        except Exception as e:
            self.serial_conn = None
            self.get_logger().warn(f'Serial connect failed: {e}')

    def tick(self) -> None:
        try:
            tf = self.tf_buffer.lookup_transform(
                self.reference_frame,
                self.target_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1),
            )
        except TransformException:
            # If no TF yet, publish a safe "home" pose to bootstrap robot_state_publisher
            if not self.tf_received:
                self.publish_home_pose()
            return

        if not self.tf_received:
            self.tf_received = True
            self.get_logger().info(f'Receiving TF from {self.target_frame}')

        px = tf.transform.translation.x
        py = tf.transform.translation.y
        pz = tf.transform.translation.z
        qx = tf.transform.rotation.x
        qy = tf.transform.rotation.y
        qz = tf.transform.rotation.z
        qw = tf.transform.rotation.w

        roll, pitch, yaw = euler_from_quaternion(qx, qy, qz, qw)

        # Use pitch for wrist pitch (q4)
        result = solve_ik(px, py, pz, pitch_target=pitch)
        
        if result is None:
            self.get_logger().warn(f'Target unreachable: ({px:.3f}, {py:.3f}, {pz:.3f})', throttle_duration_sec=2.0)
            # If unreachable, stay at last valid pose or home
            if self.last_joints:
                self.publish_joints(self.last_joints)
            else:
                self.publish_home_pose()
            return

        q1, q2, q3, q4, q5 = result
        
        # Syringe slider (q6) - separate control, default to 0 for now
        q6 = 0.0

        q_gear1 = q4
        q_gear2 = q4
        q_ee_gear = 0.0

        positions = [q1, q2, q3, q4, q5, q6, q_gear1, q_gear2, q_ee_gear]
        self.publish_joints(positions)
        self.last_joints = positions
        self.send_to_serial(positions)

    def publish_home_pose(self) -> None:
        # Safe home pose: q2=pi/2 (up), q3=-pi/2 (bent), others 0
        home_joints = [0.0, 1.57, -1.57, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.publish_joints(home_joints)

    def publish_joints(self, positions: List[float]) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = positions
        self.joint_pub.publish(msg)

    def send_to_serial(self, positions: List[float]) -> None:
        if self.serial_conn is None:
            return
        def to_adc(val: float, lo: float, hi: float) -> int:
            ratio = (val - lo) / (hi - lo) if hi != lo else 0.5
            return int(clamp(ratio, 0.0, 1.0) * 1023)

        adc_vals = [to_adc(positions[i], *JOINT_LIMITS[i]) for i in range(6)]
        line = ','.join(str(v) for v in adc_vals) + '\n'
        try:
            self.serial_conn.write(line.encode('utf-8'))
        except Exception as e:
            self.get_logger().warn(f'Serial write error: {e}')
            self.serial_conn = None


def main(args=None) -> None:
    rclpy.init(args=args)
    node = IKNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
