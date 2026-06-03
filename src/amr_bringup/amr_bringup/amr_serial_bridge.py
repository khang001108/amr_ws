#!/usr/bin/env python3
"""Bridge ESP32 AMR serial telemetry and ROS 2 topics."""

import math

from geometry_msgs.msg import Quaternion, TransformStamped, Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
import serial
from tf2_ros import TransformBroadcaster


def euler_to_quat(roll: float, pitch: float, yaw: float) -> Quaternion:
    """Convert Euler orientation angles to a ROS quaternion."""
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


class AMRSerialBridge(Node):
    """Publish AMR state from serial data and forward velocity commands."""

    def __init__(self):
        """Initialize serial communications and ROS interfaces."""
        super().__init__('amr_serial_bridge')

        self.declare_parameter('port', '/dev/esp32')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('wheel_radius', 0.0435)
        self.declare_parameter('wheel_base', 0.20)
        self.declare_parameter('encoder_ppr', 4400)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('footprint_frame', 'base_footprint')
        self.declare_parameter('left_joint_name', 'wheel_left_joint')
        self.declare_parameter('right_joint_name', 'wheel_right_joint')
        self.declare_parameter('cmd_linear_gain', 1.5)
        self.declare_parameter('cmd_angular_gain', 1.3)
        self.declare_parameter('cmd_min_linear', 0.12)
        self.declare_parameter('cmd_min_angular', 0.35)
        self.declare_parameter('cmd_max_linear', 0.6)
        self.declare_parameter('cmd_max_angular', 1.5)
        self.declare_parameter('cmd_deadband', 0.01)

        self.port = self.get_parameter('port').value
        self.baud = self.get_parameter('baud').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.encoder_ppr = self.get_parameter('encoder_ppr').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.footprint_frame = self.get_parameter('footprint_frame').value
        self.left_joint_name = self.get_parameter('left_joint_name').value
        self.right_joint_name = self.get_parameter('right_joint_name').value
        self.cmd_linear_gain = self.get_parameter('cmd_linear_gain').value
        self.cmd_angular_gain = self.get_parameter('cmd_angular_gain').value
        self.cmd_min_linear = self.get_parameter('cmd_min_linear').value
        self.cmd_min_angular = self.get_parameter('cmd_min_angular').value
        self.cmd_max_linear = self.get_parameter('cmd_max_linear').value
        self.cmd_max_angular = self.get_parameter('cmd_max_angular').value
        self.cmd_deadband = self.get_parameter('cmd_deadband').value

        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.01)
            self.get_logger().info(
                f'Serial opened: {self.port} @ {self.baud}')
        except serial.SerialException as error:
            self.get_logger().fatal(f'Cannot open serial port: {error}')
            raise SystemExit(1) from error

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.js_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.tf_br = TransformBroadcaster(self)

        self.create_subscription(Twist, '/cmd_vel', self._cmd_vel_cb, 10)

        self._prev_enc_l = None
        self._prev_enc_r = None
        self._prev_odom_time = None
        self._x = 0.0
        self._y = 0.0
        self._th = 0.0
        self._wheel_pos_l = 0.0
        self._wheel_pos_r = 0.0

        self.create_timer(0.01, self._read_serial)

        self.get_logger().info('AMR serial bridge ready.')
        self.get_logger().info(
            'Publishing joint_states for joints: '
            f'[{self.left_joint_name}, {self.right_joint_name}]')

    def _cmd_vel_cb(self, msg: Twist):
        """Forward a ROS velocity command to the ESP32."""
        linear_x = self._shape_cmd(
            msg.linear.x,
            self.cmd_linear_gain,
            self.cmd_min_linear,
            self.cmd_max_linear,
        )
        angular_z = self._shape_cmd(
            msg.angular.z,
            self.cmd_angular_gain,
            self.cmd_min_angular,
            self.cmd_max_angular,
        )
        line = f'{linear_x:.4f},{angular_z:.4f}\n'
        try:
            self.ser.write(line.encode())
        except serial.SerialException as error:
            self.get_logger().error(f'Serial write error: {error}')

    def _shape_cmd(
            self, value: float, gain: float, min_abs: float,
            max_abs: float) -> float:
        """Scale commands and lift small non-zero values above motor stiction."""
        if abs(value) <= self.cmd_deadband:
            return 0.0

        shaped = value * gain
        if abs(shaped) < min_abs:
            shaped = math.copysign(min_abs, shaped)

        return max(-max_abs, min(max_abs, shaped))

    def _read_serial(self):
        """Read and dispatch complete telemetry lines."""
        try:
            while self.ser.in_waiting:
                raw = self.ser.readline().decode(errors='replace').strip()
                if raw.startswith('ODOM,'):
                    self._handle_odom(raw)
                elif raw.startswith('IMU,'):
                    self._handle_imu(raw)
        except serial.SerialException as error:
            self.get_logger().error(f'Serial read error: {error}')

    def _handle_odom(self, line: str):
        """Parse wheel telemetry and publish odometry, joints, and TF."""
        try:
            parts = line.split(',')
            enc_l = int(parts[1])
            enc_r = int(parts[2])
        except (IndexError, ValueError):
            self.get_logger().warn(f'Bad ODOM line: {line}')
            return

        now = self.get_clock().now()
        if self._prev_enc_l is None:
            self._prev_enc_l = enc_l
            self._prev_enc_r = enc_r
            self._prev_odom_time = now
            return

        d_enc_l = enc_l - self._prev_enc_l
        d_enc_r = enc_r - self._prev_enc_r
        dt = (now - self._prev_odom_time).nanoseconds / 1.0e9
        self._prev_enc_l = enc_l
        self._prev_enc_r = enc_r
        self._prev_odom_time = now
        if dt <= 0.0:
            return

        wheel_scale = 2.0 * math.pi * self.wheel_radius / self.encoder_ppr
        d_left = d_enc_l * wheel_scale
        d_right = d_enc_r * wheel_scale
        d_centre = (d_left + d_right) / 2.0
        d_theta = (d_right - d_left) / self.wheel_base

        self._th += d_theta
        self._x += d_centre * math.cos(self._th)
        self._y += d_centre * math.sin(self._th)

        radians_per_count = 2.0 * math.pi / self.encoder_ppr
        self._wheel_pos_l += d_enc_l * radians_per_count
        self._wheel_pos_r += d_enc_r * radians_per_count

        v_left = d_left / dt
        v_right = d_right / dt
        vx = (v_left + v_right) / 2.0
        wz = (v_right - v_left) / self.wheel_base
        vel_l = d_enc_l * radians_per_count / dt
        vel_r = d_enc_r * radians_per_count / dt
        orientation = euler_to_quat(0.0, 0.0, self._th)

        joint_state = JointState()
        joint_state.header.stamp = now.to_msg()
        joint_state.name = [self.left_joint_name, self.right_joint_name]
        joint_state.position = [self._wheel_pos_l, self._wheel_pos_r]
        joint_state.velocity = [vel_l, vel_r]
        self.js_pub.publish(joint_state)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.footprint_frame
        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.orientation = orientation
        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = wz
        odom.pose.covariance[0] = 0.01
        odom.pose.covariance[7] = 0.01
        odom.pose.covariance[35] = 0.05
        self.odom_pub.publish(odom)

        transform = TransformStamped()
        transform.header.stamp = now.to_msg()
        transform.header.frame_id = self.odom_frame
        transform.child_frame_id = self.footprint_frame
        transform.transform.translation.x = self._x
        transform.transform.translation.y = self._y
        transform.transform.rotation = orientation
        self.tf_br.sendTransform(transform)

    def _handle_imu(self, line: str):
        """Parse yaw telemetry and publish an orientation measurement."""
        try:
            yaw_deg = float(line.split(',')[1])
        except (IndexError, ValueError):
            self.get_logger().warn(f'Bad IMU line: {line}')
            return

        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = self.base_frame
        imu_msg.orientation = euler_to_quat(
            0.0, 0.0, math.radians(yaw_deg))
        imu_msg.linear_acceleration_covariance[0] = -1.0
        imu_msg.angular_velocity_covariance[0] = -1.0
        imu_msg.orientation_covariance[0] = 0.01
        imu_msg.orientation_covariance[4] = 0.01
        imu_msg.orientation_covariance[8] = 0.01
        self.imu_pub.publish(imu_msg)


def main(args=None):
    """Run the AMR serial bridge node."""
    rclpy.init(args=args)
    node = AMRSerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
