#!/usr/bin/env python3
"""Run a conservative multisine identification trajectory and log joint states.

This program deliberately does *not* move the arm unless ``--execute`` is
given.  It sends one FollowJointTrajectory goal to the existing trajectory
controller, while saving measured position, velocity and Present Current
(``JointState.effort``) to a CSV file for the later gravity/friction fit.
"""

import argparse
import csv
import math
import os
import sys
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint


ARM_JOINTS = ('joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6')
READY_POSE = (0.0, -0.6806, 1.3613, 0.0, 0.8901, 0.0)
# Each profile uses three incommensurate angular frequencies (rad/s).  The
# final coefficient is selected so sum(A_k * omega_k) is zero: the robot starts
# with zero velocity rather than a step-like kick.
PROFILES = {
    # First-hardware-run profile: deliberately very slow.
    'conservative': {
        'omegas': (0.10, 0.16, 0.24),
        'coefficients': {
            'joint2': (0.060, -0.020, -0.011666666666666667),
            'joint3': (0.040, -0.015, -0.006666666666666667),
        },
    },
    # Teaching identification profile: about 7 deg/s (joint2) and 5 deg/s
    # (joint3) at nominal scale.  This is still much slower than the paper's
    # roughly 50--60 deg/s plots, keeping omitted inertial torque modest.
    'teaching': {
        'omegas': (0.50, 0.80, 1.20),
        'coefficients': {
            'joint2': (0.120, -0.040, -0.023333333333333334),
            'joint3': (0.080, -0.030, -0.013333333333333334),
        },
    },
    # Same teaching frequencies on all arm axes.  Axis 1/4/6 have negligible
    # gravity loading, but their bidirectional motion identifies Coulomb
    # friction and current bias; axis 5 gets its own gravity/friction data.
    'teaching_all': {
        'omegas': (0.50, 0.80, 1.20),
        'coefficients': {
            'joint1': (0.100, -0.040, -0.015000000000000000),
            'joint2': (0.120, -0.040, -0.023333333333333334),
            'joint3': (0.080, -0.030, -0.013333333333333334),
            'joint4': (0.080, -0.030, -0.013333333333333334),
            'joint5': (0.100, -0.035, -0.018333333333333333),
            'joint6': (0.060, -0.0225, -0.010000000000000000),
        },
    },
}
# Keep a 0.02 rad buffer from the URDF limits during a hardware experiment.
# The trajectory command itself is still validated by arm_controller; this is
# an earlier, human-readable guard before any goal is sent.
JOINT_LIMITS = {
    'joint1': (-math.pi * 0.90, math.pi * 0.90),
    'joint2': (-math.pi * 0.65, math.pi * 0.67),
    'joint3': (-math.pi * 0.60, math.pi * 0.68),
    'joint4': (-math.pi * 0.90, math.pi * 0.90),
    'joint5': (-math.pi * 0.63, math.pi * 0.67),
    'joint6': (-math.pi * 0.90, math.pi * 0.90),
}
LIMIT_MARGIN = 0.020
SETTLE_TIME = 1.0
MAX_AMPLITUDE_SCALE = 1.5
MAX_FREQUENCY_SCALE = 1.5


def multisine(coefficients, omegas, t):
    """Return position offset and velocity for the three-sine trajectory."""
    position = sum(a * math.sin(w * t) for a, w in zip(coefficients, omegas))
    velocity = sum(a * w * math.cos(w * t) for a, w in zip(coefficients, omegas))
    return position, velocity


class IdentificationRun(Node):
    def __init__(self, args):
        super().__init__('multisine_identification')
        self._args = args
        self._latest = None
        self._recording = False
        self._csv_file = None
        self._writer = None
        self._samples = 0
        self._start_monotonic = None
        self._client = ActionClient(self, FollowJointTrajectory, args.action)
        self._active_handle = None
        self._active_result = None
        self.create_subscription(JointState, args.joint_states, self._on_joint_state, 50)
        self._simulation_publisher = None
        self._simulation_start = None
        if args.simulate:
            self._simulation_publisher = self.create_publisher(JointState, args.joint_states, 10)
            self._simulation_start = time.monotonic()
            self.create_timer(0.02, self._publish_simulation)

    def _coefficients(self, name):
        coefficients = PROFILES[self._args.profile]['coefficients'][name]
        return tuple(value * self._args.amplitude_scale for value in coefficients)

    def _omegas(self):
        return tuple(value * self._args.frequency_scale
                     for value in PROFILES[self._args.profile]['omegas'])

    def _on_joint_state(self, message):
        values = {name: index for index, name in enumerate(message.name)}
        if not all(name in values for name in ARM_JOINTS):
            missing = ', '.join(name for name in ARM_JOINTS if name not in values)
            self.get_logger().warn('ignoring incomplete /joint_states; missing: %s' % missing)
            return
        self._latest = message
        if not self._recording:
            return

        def field(sequence, name):
            index = values[name]
            return sequence[index] if index < len(sequence) else float('nan')

        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1.0e-9
        row = [stamp, time.monotonic() - self._start_monotonic]
        for name in ARM_JOINTS:
            row.extend((field(message.position, name), field(message.velocity, name),
                        field(message.effort, name)))
        self._writer.writerow(row)
        self._samples += 1

    def _publish_simulation(self):
        """Publish the exact planned motion for RViz, without any hardware I/O."""
        elapsed = time.monotonic() - self._simulation_start
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'base_link'
        # The gripper is not actuated by this experiment, but robot_state_
        # publisher still needs its prismatic joint to publish the two finger
        # frames.  Omitting it left RobotModel with a missing-TF error in RViz.
        message.name = list(ARM_JOINTS) + ['gripper_left_joint']
        # The known, collision-free ready pose used by the ros2_control fake rig.
        message.position = list(READY_POSE) + [0.0157]
        message.velocity = [0.0] * len(message.name)
        message.effort = [0.0] * len(message.name)
        for name in PROFILES[self._args.profile]['coefficients']:
            coefficients = self._coefficients(name)
            index = ARM_JOINTS.index(name)
            offset, velocity = multisine(coefficients, self._omegas(), elapsed)
            message.position[index] += offset
            message.velocity[index] = velocity
        self._simulation_publisher.publish(message)

    def wait_for_state(self, timeout):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and self._latest is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self._latest is not None

    def preview(self):
        if not self.wait_for_state(15.0):
            raise RuntimeError('no complete JointState received within 15 seconds')
        indices = {name: index for index, name in enumerate(self._latest.name)}
        starts = {name: self._latest.position[indices[name]] for name in ARM_JOINTS}
        self.get_logger().info('current pose: ' + ', '.join(
            '%s=%.4f' % (name, starts[name]) for name in ARM_JOINTS))
        unsafe = []
        steps = int(math.ceil(self._args.duration / 0.01)) + 1
        for name in PROFILES[self._args.profile]['coefficients']:
            coefficients = self._coefficients(name)
            offsets = [multisine(
                coefficients, self._omegas(), min(index * 0.01, self._args.duration))[0]
                       for index in range(steps)]
            offset_min, offset_max = min(offsets), max(offsets)
            _, initial_velocity = multisine(coefficients, self._omegas(), 0.0)
            self.get_logger().info(
                '%s: planned range [%.4f, %.4f] rad, initial velocity %.3g rad/s'
                % (name, starts[name] + offset_min, starts[name] + offset_max,
                   initial_velocity))
            lower, upper = JOINT_LIMITS[name]
            if starts[name] + offset_min < lower + LIMIT_MARGIN:
                unsafe.append('%s would approach lower limit %.4f rad (needs >= %.4f)'
                              % (name, lower, lower + LIMIT_MARGIN))
            if starts[name] + offset_max > upper - LIMIT_MARGIN:
                unsafe.append('%s would approach upper limit %.4f rad (needs <= %.4f)'
                              % (name, upper, upper - LIMIT_MARGIN))
        if unsafe:
            raise RuntimeError('unsafe start pose: ' + '; '.join(unsafe))

    def _trajectory(self):
        indices = {name: index for index, name in enumerate(self._latest.name)}
        starts = [self._latest.position[indices[name]] for name in ARM_JOINTS]
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM_JOINTS)
        count = int(round(self._args.duration / self._args.period)) + 1
        for point_index in range(count):
            t = min(point_index * self._args.period, self._args.duration)
            point = JointTrajectoryPoint()
            point.positions = list(starts)
            point.velocities = [0.0] * len(ARM_JOINTS)
            for name in PROFILES[self._args.profile]['coefficients']:
                coefficients = self._coefficients(name)
                index = ARM_JOINTS.index(name)
                offset, velocity = multisine(coefficients, self._omegas(), t)
                point.positions[index] += offset
                point.velocities[index] = velocity
            point.time_from_start = Duration(seconds=t).to_msg()
            goal.trajectory.points.append(point)
        # The stock arm_controller rejects a trajectory whose final waypoint
        # has non-zero velocity.  Keep the final multisine pose for one second
        # and end at zero velocity, so it can decelerate and satisfy that rule.
        settle = JointTrajectoryPoint()
        settle.positions = list(goal.trajectory.points[-1].positions)
        settle.velocities = [0.0] * len(ARM_JOINTS)
        settle.time_from_start = Duration(seconds=self._args.duration + SETTLE_TIME).to_msg()
        goal.trajectory.points.append(settle)
        return goal

    def _move_to_ready(self):
        """Move every arm joint to the known ready pose at a conservative speed."""
        indices = {name: index for index, name in enumerate(self._latest.name)}
        starts = [self._latest.position[indices[name]] for name in ARM_JOINTS]
        duration = max(
            3.0,
            max(abs(target - start) for start, target in zip(starts, READY_POSE))
            / self._args.ready_speed)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM_JOINTS)
        start = JointTrajectoryPoint()
        start.positions = starts
        start.velocities = [0.0] * len(ARM_JOINTS)
        start.time_from_start = Duration(seconds=0.0).to_msg()
        target = JointTrajectoryPoint()
        target.positions = list(READY_POSE)
        target.velocities = [0.0] * len(ARM_JOINTS)
        target.time_from_start = Duration(seconds=duration).to_msg()
        goal.trajectory.points = [start, target]
        self.get_logger().warn(
            'moving to ready pose over %.1f seconds before identification' % duration)
        try:
            future = self._client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, future)
            self._active_handle = future.result()
            if self._active_handle is None or not self._active_handle.accepted:
                raise RuntimeError('trajectory controller rejected the ready-pose goal')
            self._active_result = self._active_handle.get_result_async()
            rclpy.spin_until_future_complete(
                self, self._active_result, timeout_sec=duration + 15.0)
            if not self._active_result.done():
                raise RuntimeError('ready-pose trajectory did not finish before timeout')
            result = self._active_result.result().result
            if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
                raise RuntimeError('ready-pose trajectory failed: %s' % result.error_string)
        finally:
            if (self._active_handle is not None and self._active_result is not None and
                    not self._active_result.done()):
                self.get_logger().warn('cancelling the active ready-pose trajectory')
                cancel = self._active_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, cancel, timeout_sec=2.0)
            self._active_handle = None
            self._active_result = None

    def execute(self):
        if not self._client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError('trajectory action is unavailable: %s' % self._args.action)
        if not self.wait_for_state(15.0):
            raise RuntimeError('no complete JointState received within 15 seconds')
        if self._args.move_to_ready or self._args.ready_only:
            self._move_to_ready()
        if self._args.ready_only:
            self.get_logger().info('ready pose reached; no identification trajectory was sent')
            return
        self.preview()
        output = os.path.abspath(os.path.expanduser(self._args.output))
        os.makedirs(os.path.dirname(output), exist_ok=True)
        self._csv_file = open(output, 'w', newline='', encoding='utf-8')
        header = ['ros_stamp_s', 'elapsed_s']
        for name in ARM_JOINTS:
            header.extend((name + '_position_rad', name + '_velocity_rad_s',
                           name + '_effort_raw'))
        self._writer = csv.writer(self._csv_file)
        self._writer.writerow(header)
        self._start_monotonic = time.monotonic()
        self._recording = True
        try:
            future = self._client.send_goal_async(self._trajectory())
            rclpy.spin_until_future_complete(self, future)
            self._active_handle = future.result()
            if self._active_handle is None or not self._active_handle.accepted:
                raise RuntimeError('trajectory controller rejected the identification goal')
            self.get_logger().info('trajectory accepted; recording to %s' % output)
            self._active_result = self._active_handle.get_result_async()
            rclpy.spin_until_future_complete(self, self._active_result,
                                             timeout_sec=self._args.duration + 15.0)
            if not self._active_result.done():
                raise RuntimeError('trajectory did not finish before timeout')
            result = self._active_result.result().result
            if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
                raise RuntimeError('trajectory failed: %s' % result.error_string)
            self.get_logger().info('finished: wrote %d samples to %s' % (self._samples, output))
        finally:
            self._recording = False
            if (self._active_handle is not None and self._active_result is not None and
                    not self._active_result.done()):
                self.get_logger().warn('cancelling the active identification trajectory')
                cancel = self._active_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, cancel, timeout_sec=2.0)
            self._active_handle = None
            self._active_result = None
            if self._csv_file is not None:
                self._csv_file.close()
                self._csv_file = None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true',
                        help='send the trajectory; without it the program only previews limits')
    parser.add_argument('--simulate', action='store_true',
                        help='publish the planned motion on /joint_states for RViz; never contacts hardware')
    parser.add_argument('--move-to-ready', action='store_true',
                        help='move to the known ready pose before running the multisine')
    parser.add_argument('--ready-only', action='store_true',
                        help='move to the known ready pose, then exit without a multisine')
    parser.add_argument('--ready-speed', type=float, default=0.12,
                        help='maximum ready-pose speed in rad/s (default: 0.12)')
    parser.add_argument('--amplitude-scale', type=float, default=1.0,
                        help='scale the tested multisine amplitude, in (0, 1.5] (default: 1.0)')
    parser.add_argument('--frequency-scale', type=float, default=1.0,
                        help='scale the multisine frequency, in (0, 1.5] (default: 1.0)')
    parser.add_argument('--profile', choices=tuple(PROFILES), default='conservative',
                        help='motion profile: conservative, teaching, or teaching_all (default: conservative)')
    parser.add_argument('--duration', type=float, default=180.0,
                        help='trajectory duration in seconds (default: 180)')
    parser.add_argument('--period', type=float, default=0.05,
                        help='trajectory point interval in seconds (default: 0.05)')
    # The standard bringup exposes this action.  The optional custom controller
    # can still be selected explicitly with --action.
    parser.add_argument('--action', default='/arm_controller/follow_joint_trajectory')
    parser.add_argument('--joint-states', default='/joint_states')
    parser.add_argument('--output', default='~/tf_identification/multisine.csv')
    args, ros_args = parser.parse_known_args()
    if args.duration <= 0.0 or args.period <= 0.0 or args.ready_speed <= 0.0:
        parser.error('--duration, --period and --ready-speed must be positive')
    if not 0.0 < args.amplitude_scale <= MAX_AMPLITUDE_SCALE:
        parser.error('--amplitude-scale must be in (0, %.1f]' % MAX_AMPLITUDE_SCALE)
    if not 0.0 < args.frequency_scale <= MAX_FREQUENCY_SCALE:
        parser.error('--frequency-scale must be in (0, %.1f]' % MAX_FREQUENCY_SCALE)
    rclpy.init(args=ros_args)
    node = IdentificationRun(args)
    try:
        if args.execute and args.simulate:
            parser.error('--execute and --simulate cannot be used together')
        if args.execute:
            node.execute()
        elif args.simulate:
            node.get_logger().info('RViz simulation active; publishing planned multisine on %s'
                                   % args.joint_states)
            rclpy.spin(node)
        else:
            node.preview()
            node.get_logger().info('preview only: re-run with --execute to move the arm')
    except RuntimeError as error:
        node.get_logger().error(str(error))
        raise SystemExit(2)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
