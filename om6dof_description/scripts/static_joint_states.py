#!/usr/bin/env python3
# Copyright 2026 OM6DOF maintainers.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Hold every movable joint at a fixed pose on /joint_states.

Enough to give robot_state_publisher something to build TF from, so the arm can
be looked at without the real robot and without joint_state_publisher_gui, which
is not installed on every machine this repo runs on.

It is deliberately dumb: no GUI, no interpolation, one pose. Use `pose` to move
it, or leave this out entirely and let the live robot publish instead.
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from urdf_parser_py.urdf import URDF

MOVABLE = ('revolute', 'continuous', 'prismatic', 'planar', 'floating')


class StaticJointStates(Node):

    def __init__(self):
        super().__init__('static_joint_states')

        self.declare_parameter('robot_description', '')
        # Declared by type with no default: rcl cannot infer a type for an empty
        # list, and declaring one throws. Left unset it reads back as None.
        self.declare_parameter('pose', Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('publish_rate', 20.0)

        description = self.get_parameter('robot_description').value
        if not description:
            raise RuntimeError("parameter 'robot_description' is empty")

        robot = URDF.from_xml_string(description)
        names = [j.name for j in robot.joints if j.type in MOVABLE]

        try:
            pose = list(self.get_parameter('pose').value or [])
        except Exception:
            pose = []
        if pose and len(pose) != len(names):
            raise RuntimeError(
                "parameter 'pose' has %d values but the model has %d movable joints: %s"
                % (len(pose), len(names), ', '.join(names)))
        if not pose:
            pose = [0.0] * len(names)

        self._message = JointState()
        self._message.name = names
        self._message.position = [float(v) for v in pose]

        self._publisher = self.create_publisher(JointState, 'joint_states', 10)
        rate = float(self.get_parameter('publish_rate').value)
        self.create_timer(1.0 / rate if rate > 0.0 else 0.05, self._publish)

        self.get_logger().info('holding %d joints: %s' % (len(names), ', '.join(names)))

    def _publish(self):
        self._message.header.stamp = self.get_clock().now().to_msg()
        self._publisher.publish(self._message)


def main():
    rclpy.init()
    node = StaticJointStates()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
