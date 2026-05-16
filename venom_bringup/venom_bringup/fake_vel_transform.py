"""Transform Nav2 velocity commands through a fake base frame.

This mirrors the fake_vel_transform node used by the reference navigation
stack: Nav2 plans in ``base_link_fake`` while the chassis receives commands in
the real ``base_link`` frame.
"""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Path
from rclpy.node import Node
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Return yaw from a quaternion."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    """Build a z-axis quaternion from yaw."""
    half_yaw = yaw * 0.5
    return 0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)


class FakeVelTransform(Node):
    """Publish ``base_link_fake`` and rotate velocity commands for the chassis."""

    def __init__(self) -> None:
        super().__init__('fake_vel_transform')

        self.declare_parameter('input_cmd_vel_topic', '/cmd_vel_raw')
        self.declare_parameter('output_cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('local_plan_topic', '/local_plan')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('fake_base_frame', 'base_link_fake')
        self.declare_parameter('spin_speed', 0.0)
        self.declare_parameter('publish_frequency_hz', 20.0)

        self.input_cmd_vel_topic = str(self.get_parameter('input_cmd_vel_topic').value)
        self.output_cmd_vel_topic = str(self.get_parameter('output_cmd_vel_topic').value)
        self.local_plan_topic = str(self.get_parameter('local_plan_topic').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.fake_base_frame = str(self.get_parameter('fake_base_frame').value)
        self.spin_speed = float(self.get_parameter('spin_speed').value)

        publish_frequency_hz = float(self.get_parameter('publish_frequency_hz').value)
        publish_period = 1.0 / max(publish_frequency_hz, 1.0)

        self._base_link_yaw = 0.0
        self._current_angle = 0.0

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._tf_broadcaster = TransformBroadcaster(self)

        self._cmd_vel_pub = self.create_publisher(Twist, self.output_cmd_vel_topic, 10)
        self.create_subscription(Twist, self.input_cmd_vel_topic, self._cmd_vel_callback, 10)
        self.create_subscription(Path, self.local_plan_topic, self._local_plan_callback, 10)
        self.create_timer(publish_period, self._publish_transform)

        self.get_logger().info(
            'fake_vel_transform active: '
            f'{self.input_cmd_vel_topic} -> {self.output_cmd_vel_topic}, '
            f'{self.base_frame} -> {self.fake_base_frame}'
        )

    def _local_plan_callback(self, msg: Path) -> None:
        if not msg.poses:
            return

        index = min(len(msg.poses) // 4, len(msg.poses) - 1)
        orientation = msg.poses[index].pose.orientation
        teb_angle = yaw_from_quaternion(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )
        self._current_angle = teb_angle - self._base_link_yaw

    def _cmd_vel_callback(self, msg: Twist) -> None:
        try:
            transform = self._tf_buffer.lookup_transform(
                self.odom_frame,
                self.base_frame,
                rclpy.time.Time(),
            )
            rotation = transform.transform.rotation
            self._base_link_yaw = yaw_from_quaternion(
                rotation.x,
                rotation.y,
                rotation.z,
                rotation.w,
            )
        except TransformException as exc:
            self.get_logger().debug(
                f'Unable to read {self.odom_frame}->{self.base_frame}: {exc}'
            )

        angle_diff = -self._current_angle
        transformed = Twist()
        transformed.angular.z = self.spin_speed if abs(msg.angular.z) > 1e-6 else 0.0
        transformed.linear.x = (
            msg.linear.x * math.cos(angle_diff) + msg.linear.y * math.sin(angle_diff)
        )
        transformed.linear.y = (
            -msg.linear.x * math.sin(angle_diff) + msg.linear.y * math.cos(angle_diff)
        )
        transformed.linear.z = msg.linear.z
        self._cmd_vel_pub.publish(transformed)

    def _publish_transform(self) -> None:
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.base_frame
        transform.child_frame_id = self.fake_base_frame

        qx, qy, qz, qw = quaternion_from_yaw(self._current_angle)
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self._tf_broadcaster.sendTransform(transform)


def main() -> None:
    rclpy.init()
    node = FakeVelTransform()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
