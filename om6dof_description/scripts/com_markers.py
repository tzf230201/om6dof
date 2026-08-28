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
Label each link's centre of mass with its name, mass and lever arm.

Text only. RViz's own RobotModel display already draws the masses -- turn on
Mass Properties -> Mass -- and it does it better than a hand-rolled marker ever
will, so drawing them here as well was duplication that only cost frame rate.
What RViz does not tell you is which link this is, what it weighs, and how far
its centre of mass sits from its own origin, which is exactly the number that
matters when the value is a placeholder somebody never filled in.

Markers are published once, latched, in each link's own frame: robot_state_
publisher's TF carries them wherever the arm goes, so republishing them would
buy nothing and cost a great deal of rendering.
"""

import math

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import ColorRGBA
from urdf_parser_py.urdf import URDF
from visualization_msgs.msg import Marker, MarkerArray

# A centre of mass sitting exactly on its link's origin is almost always a
# placeholder nobody filled in rather than a part that genuinely balances there.
PLACEHOLDER_EPS = 1e-9


class ComLabels(Node):

    def __init__(self):
        super().__init__('com_markers')

        self.declare_parameter('robot_description', '')
        self.declare_parameter('label_scale', 0.010)
        # RViz draws TEXT_VIEW_FACING above the point it is given, by roughly the
        # height of the text, so the gap grows with label_scale. Shrinking the
        # text closes it; this nudges the label along the link's own axes when
        # that is not enough. Both take effect without a restart.
        self.declare_parameter('label_offset', [0.03, 0.0, 0.0])
        # The label is the mass and nothing else by default. RViz centres text on
        # the point it is given, so a long string sprawls away from the dot in
        # both directions and stops reading as belonging to it. The link name is
        # obvious from where the label sits; the lever arm is in the log.
        self.declare_parameter('show_name', False)
        self.declare_parameter('show_lever', False)

        description = self.get_parameter('robot_description').value
        if not description:
            raise RuntimeError(
                "parameter 'robot_description' is empty; pass the URDF the same way "
                'robot_state_publisher gets it')

        self._robot = URDF.from_xml_string(description)
        robot = self._robot

        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._publisher = self.create_publisher(MarkerArray, '~/com_markers', latched)

        array = self._build()
        self._publisher.publish(array)
        self.add_on_set_parameters_callback(self._on_parameters)

        total = 0.0
        placeholders = []
        for link in robot.links:
            if link.inertial is None or link.inertial.mass <= 0.0:
                continue
            total += link.inertial.mass
            xyz = list(link.inertial.origin.xyz) if link.inertial.origin else [0.0, 0.0, 0.0]
            if math.sqrt(sum(v * v for v in xyz)) < PLACEHOLDER_EPS:
                placeholders.append(link.name)

        self.get_logger().info(
            'labelled %d links, %.4f kg in total' % (len(array.markers), total))
        if placeholders:
            self.get_logger().warn(
                'centre of mass still on the link origin, so probably never filled in: %s'
                % ', '.join(placeholders))

    def _build(self, scale=None, offset=None, show_name=None, show_lever=None):
        if scale is None:
            scale = float(self.get_parameter('label_scale').value)
        if offset is None:
            offset = list(self.get_parameter('label_offset').value)
        if show_name is None:
            show_name = bool(self.get_parameter('show_name').value)
        if show_lever is None:
            show_lever = bool(self.get_parameter('show_lever').value)
        robot = self._robot

        array = MarkerArray()
        for index, link in enumerate(robot.links):
            inertial = link.inertial
            if inertial is None or inertial.mass <= 0.0:
                continue
            xyz = list(inertial.origin.xyz) if inertial.origin else [0.0, 0.0, 0.0]
            lever = math.sqrt(sum(v * v for v in xyz))
            placeholder = lever < PLACEHOLDER_EPS

            marker = Marker()
            marker.header.frame_id = link.name
            # The markers are published once, but their link frames move as the
            # joints move.  Ask RViz to resolve the frame on every render frame
            # instead of transforming this pose only when the message arrives.
            marker.frame_locked = True
            marker.ns = 'label'
            marker.id = index
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            marker.pose.position.x = float(xyz[0]) + offset[0]
            marker.pose.position.y = float(xyz[1]) + offset[1]
            marker.pose.position.z = float(xyz[2]) + offset[2]
            marker.pose.orientation.w = 1.0
            marker.scale.z = scale
            marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            text = '%.4f' % inertial.mass
            if show_name:
                text = '%s  %s' % (link.name, text)
            if show_lever:
                text += '  (CoM on origin)' if placeholder else '  %.1f mm' % (lever * 1000.0)
            marker.text = text
            array.markers.append(marker)

        return array

    def _on_parameters(self, parameters):
        watched = {p.name: p.value for p in parameters
                   if p.name in ('label_scale', 'label_offset', 'show_name', 'show_lever')}
        if watched:
            # The framework applies these only after this returns, so the new
            # values have to be read out of the request rather than off the node.
            self._publisher.publish(
                self._build(
                    scale=watched.get('label_scale'),
                    offset=list(watched['label_offset']) if 'label_offset' in watched else None,
                    show_name=watched.get('show_name'),
                    show_lever=watched.get('show_lever')))
        return SetParametersResult(successful=True)


def main():
    rclpy.init()
    node = ComLabels()
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
