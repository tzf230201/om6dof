"""Logitech F710 -> isolated additive preview Twist, no ownership/motor commands."""
import math

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node

from .teleop_node import _LinuxJoystick, GAMEPAD_NAME_HINTS


def joystick_twist(axes, buttons):
    if len(axes) < 6 or not all(math.isfinite(v) for v in axes):
        raise ValueError('need six finite Logitech axes')
    def deadzone(value):
        if abs(value) < 0.08:
            return 0.0
        return math.copysign(min(1.0, (abs(value) - 0.08) / 0.92), value)
    values = [-deadzone(axes[1]) * 0.03, -deadzone(axes[0]) * 0.03,
              -deadzone(axes[4]) * 0.03, deadzone(axes[3]) * 0.35,
              ((0 in buttons) - (3 in buttons)) * 0.35,
              (max(0.0, min(1.0, (axes[5] + 1) / 2)) -
               max(0.0, min(1.0, (axes[2] + 1) / 2))) * 0.35]
    for start, bound in ((0, 0.05), (3, 0.5)):
        length = math.sqrt(sum(x*x for x in values[start:start+3]))
        if length > bound:
            values[start:start+3] = [x * bound / length for x in values[start:start+3]]
    return values


class PreviewGamepad(Node):
    def __init__(self):
        super().__init__('combined_preview_gamepad')
        self.device = _LinuxJoystick(GAMEPAD_NAME_HINTS)
        self.armed = False
        self.publisher = self.create_publisher(
            TwistStamped, '/om6dof/combined_preview/teleop_twist', 1)
        self.create_timer(0.02, self._tick)

    def _tick(self):
        connected, axes, buttons = self.device.snapshot()
        if not connected:
            self.armed = False
            return  # absence expires input; it is not synthetic neutral
        try:
            values = joystick_twist(axes, buttons)
        except ValueError:
            self.armed = False
            return
        if not self.armed:
            self.armed = not any(abs(x) > 1e-9 for x in values)
            values = [0.0] * 6
        msg = TwistStamped()
        msg.header.frame_id = 'world'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = values[:3]
        msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z = values[3:]
        self.publisher.publish(msg)

    def destroy_node(self):
        self.device._close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PreviewGamepad()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
