#!/usr/bin/env python3
"""Run a conservative multisine identification trajectory and log joint states.

This program deliberately does *not* move the arm unless ``--execute`` is
given. It requests the canonical
Go to READY sequence through TRANSITION, verifies both poses, then sends an
identification trajectory while saving measured position, velocity and Present Current
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
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectoryPoint
from identification_tracking import TrackingRecorder, tracking_path, report_path, write_report


ARM_JOINTS = ('joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6')
SIMULATION_POSE = (0.0,) * 6  # visualization only, never a hardware READY target
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
# Small joint-specific multisine signals. These are an experimental tracking
# profile, NOT a collision-checked trajectory or proven identification design.
_COMBINED_OMEGAS = {
    'joint1': (.43, .71, 1.07), 'joint2': (.51, .83, 1.29),
    'joint3': (.59, .97, 1.43), 'joint4': (.67, 1.09, 1.61),
    'joint5': (.73, 1.19, 1.79), 'joint6': (.79, 1.31, 1.97),
}
_COMBINED_AMPLITUDES = dict(zip(ARM_JOINTS, (.06, .08, .06, .05, .06, .04)))
PROFILES['teaching_combined'] = {
    'joint_omegas': _COMBINED_OMEGAS,
    'coefficients': {
        name: (a, -.35*a, -(a*w[0] - .35*a*w[1])/w[2])
        for name, w in _COMBINED_OMEGAS.items()
        for a in (_COMBINED_AMPLITUDES[name],)
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
READY_POSITION_TOLERANCE = 0.03
READY_VELOCITY_TOLERANCE = 0.03
STATE_MAX_AGE = 0.5


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
        self._latest_received = 0.0
        self._ready_pose = None
        self._transition_pose = None
        self._ready_publisher = None
        self._recording = False
        self._csv_file = None
        self._writer = None
        self._samples = 0
        self._start_monotonic = None
        self._last_recorded = -float('inf')
        self._client = ActionClient(self, FollowJointTrajectory, args.action)
        self._active_handle = None
        self._active_result = None
        self._tracking = None
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

    def _omegas(self, name):
        profile = PROFILES[self._args.profile]
        omegas = (profile['joint_omegas'][name] if 'joint_omegas' in profile
                  else profile['omegas'])
        return tuple(value * self._args.frequency_scale
                     for value in omegas)

    def _excited_joints(self):
        return self._args.joints or tuple(PROFILES[self._args.profile]['coefficients'])

    def _on_joint_state(self, message):
        values = {name: index for index, name in enumerate(message.name)}
        if not all(name in values for name in ARM_JOINTS):
            missing = ', '.join(name for name in ARM_JOINTS if name not in values)
            self.get_logger().warn('ignoring incomplete /joint_states; missing: %s' % missing)
            return
        if any(values[name] >= len(message.position) or
               not math.isfinite(message.position[values[name]]) for name in ARM_JOINTS):
            return
        self._latest = message
        self._latest_received = time.monotonic()
        if not self._recording:
            return
        if self._latest_received - self._last_recorded < 1.0 / self._args.rate:
            return
        self._last_recorded = self._latest_received

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
        # Synthetic visualization origin only; not a hardware-ready target.
        message.position = list(SIMULATION_POSE) + [0.0157]
        message.velocity = [0.0] * len(message.name)
        message.effort = [0.0] * len(message.name)
        for name in self._excited_joints():
            coefficients = self._coefficients(name)
            index = ARM_JOINTS.index(name)
            offset, velocity = multisine(coefficients, self._omegas(name), elapsed)
            message.position[index] += offset
            message.velocity[index] = velocity
        self._simulation_publisher.publish(message)

    def wait_for_state(self, timeout):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            if self._latest is not None and time.monotonic() - self._latest_received < STATE_MAX_AGE:
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        return False

    def preview(self):
        if self._ready_pose is None:
            self._load_ready_configuration()
        if not self.wait_for_state(15.0):
            raise RuntimeError('no complete JointState received within 15 seconds')
        indices = {name: index for index, name in enumerate(self._latest.name)}
        # A dry-run previews the excitation around its mandatory ready start.
        starts = (dict(zip(ARM_JOINTS, self._ready_pose)) if not self._args.execute else
                  {name: self._latest.position[indices[name]] for name in ARM_JOINTS})
        self.get_logger().info('current pose: ' + ', '.join(
            '%s=%.4f' % (name, starts[name]) for name in ARM_JOINTS))
        unsafe = []
        steps = int(math.ceil(self._args.duration / 0.01)) + 1
        for name in self._excited_joints():
            coefficients = self._coefficients(name)
            offsets = [multisine(
                coefficients, self._omegas(name), min(index * 0.01, self._args.duration))[0]
                       for index in range(steps)]
            offset_min, offset_max = min(offsets), max(offsets)
            _, initial_velocity = multisine(coefficients, self._omegas(name), 0.0)
            self.get_logger().info('%s: frequencies=%s rad/s; absolute offset bound=%.4f rad' %
                (name, ','.join('%.3f' % w for w in self._omegas(name)),
                 sum(abs(a) for a in coefficients)))
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
            for name in self._excited_joints():
                coefficients = self._coefficients(name)
                index = ARM_JOINTS.index(name)
                offset, velocity = multisine(coefficients, self._omegas(name), t)
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

    def _load_ready_configuration(self):
        """Read the same live controller configuration used by Go to READY."""
        client = self.create_client(GetParameters, '/om6dof_controller/get_parameters')
        try:
            if not client.wait_for_service(timeout_sec=5.0):
                raise RuntimeError('READY configuration unavailable: start the canonical OM6DOF controller')
            request = GetParameters.Request()
            request.names = ['transition_pose_degrees', 'ready_pose_degrees',
                             'operation_mode_topic', 'pose_profile_duration_seconds', 'arm_action']
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            if not future.done() or future.result() is None:
                raise RuntimeError('timed out reading READY configuration; no motion requested')
            values = future.result().values
            if len(values) != 5:
                raise RuntimeError('incomplete READY configuration; no motion requested')
            poses = []
            for value in values[:2]:
                degrees = list(value.double_array_value)
                if len(degrees) != 6 or not all(math.isfinite(v) for v in degrees):
                    raise RuntimeError('invalid controller TRANSITION/READY pose')
                pose = tuple(math.radians(v) for v in degrees)
                if any(not JOINT_LIMITS[name][0] <= v <= JOINT_LIMITS[name][1]
                       for name, v in zip(ARM_JOINTS, pose)):
                    raise RuntimeError('controller TRANSITION/READY pose exceeds joint limits')
                poses.append(pose)
            topic = values[2].string_value
            duration = values[3].double_value
            if not topic or not math.isfinite(duration) or duration <= 0:
                raise RuntimeError('invalid controller READY topic/duration')
            if values[4].string_value != self._args.action:
                raise RuntimeError('calibration action differs from the Go to READY controller action')
            self._transition_pose, self._ready_pose = poses
            self._ready_timeout = 2.0 * duration + 15.0
            self._ready_topic = topic
        finally:
            self.destroy_client(client)

    def _move_to_ready(self):
        """Use Go to READY's command, letting its controller own both waypoints."""
        self._load_ready_configuration()
        self._ready_publisher = self.create_publisher(String, self._ready_topic, 10)
        deadline = time.monotonic() + 5.0
        while self._ready_publisher.get_subscription_count() < 1:
            if not rclpy.ok() or time.monotonic() >= deadline:
                raise RuntimeError('Go to READY controller is not listening; no motion requested')
            rclpy.spin_once(self, timeout_sec=0.05)
        requested_at = time.monotonic()
        self._ready_publisher.publish(String(data='READY'))
        self.get_logger().info('TRANSITION_THEN_READY: requested the same command as Go to READY')
        deadline = requested_at + self._ready_timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            message = self._latest
            if (message is None or self._latest_received <= requested_at or
                    time.monotonic() - self._latest_received >= STATE_MAX_AGE):
                continue
            indices = {name: i for i, name in enumerate(message.name)}
            if all(abs(message.position[indices[name]] - target) <= READY_POSITION_TOLERANCE
                   for name, target in zip(ARM_JOINTS, self._transition_pose)):
                self.get_logger().info('TRANSITION_CONFIRMED: waiting for READY')
                return
        raise RuntimeError('TRANSITION was not observed after Go to READY request; calibration cancelled')

    def _wait_ready_settled(self, timeout=10.0):
        """Require fresh measured position and velocity at ready for one second."""
        deadline = time.monotonic() + timeout
        stable_since = None
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            now = time.monotonic()
            message = self._latest
            stable = message is not None and now - self._latest_received < STATE_MAX_AGE
            if stable:
                indices = {name: i for i, name in enumerate(message.name)}
                for name, target in zip(ARM_JOINTS, self._ready_pose):
                    i = indices[name]
                    stable = stable and (
                        abs(message.position[i] - target) <= READY_POSITION_TOLERANCE and
                        i < len(message.velocity) and math.isfinite(message.velocity[i]) and
                        abs(message.velocity[i]) <= READY_VELOCITY_TOLERANCE)
            if not stable:
                stable_since = None
            elif stable_since is None:
                stable_since = now
            elif now - stable_since >= SETTLE_TIME:
                self.get_logger().info('READY_CONFIRMED: starting calibration from settled ready pose')
                return
        raise RuntimeError('ready pose was not confirmed by fresh, settled joint feedback; calibration cancelled')

    def _begin_recording(self):
        output = os.path.abspath(os.path.expanduser(self._args.output))
        os.makedirs(os.path.dirname(output), exist_ok=True)
        self._csv_file = open(output, 'x', newline='', encoding='utf-8')
        header = ['ros_stamp_s', 'elapsed_s']
        for name in ARM_JOINTS:
            header.extend((name + '_position_rad', name + '_velocity_rad_s',
                           name + '_effort_raw'))
        self._writer = csv.writer(self._csv_file)
        self._writer.writerow(header)
        self._start_monotonic = time.monotonic()
        self._recording = True
        return output

    def record_only(self):
        """Passive /joint_states capture; never send a trajectory or change torque."""
        if not self.wait_for_state(15.0):
            raise RuntimeError('no complete JointState received within 15 seconds')
        output = self._begin_recording()
        self.get_logger().info('recording joint states to %s; stop with Ctrl-C' % output)
        try:
            rclpy.spin(self)
        finally:
            self._recording = False
            self._csv_file.close()
            self._csv_file = None

    def execute(self):
        # Refuse collisions before requesting READY (which itself moves the arm).
        output_path = os.path.abspath(os.path.expanduser(self._args.output))
        if (os.path.exists(output_path) or tracking_path(output_path).exists() or
                report_path(output_path).exists()):
            raise RuntimeError('recording or tracking output already exists; choose a new path')
        if not self._client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError('trajectory action is unavailable: %s' % self._args.action)
        if not self.wait_for_state(15.0):
            raise RuntimeError('no complete JointState received within 15 seconds')
        self.get_logger().info('MOVING_TO_READY: calibration waits for ready-pose success')
        self._move_to_ready()
        self._wait_ready_settled(timeout=self._ready_timeout)
        if self._args.ready_only:
            self.get_logger().info('ready pose reached; no identification trajectory was sent')
            return
        self.preview()
        output = self._begin_recording()
        try:
            self._tracking = TrackingRecorder(output)
            future = self._client.send_goal_async(
                self._trajectory(), feedback_callback=self._tracking.write)
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
            response = self._active_result.result()
            result = response.result
            if (response.status != GoalStatus.STATUS_SUCCEEDED or
                    result.error_code != FollowJointTrajectory.Result.SUCCESSFUL):
                raise RuntimeError('trajectory failed: %s' % result.error_string)
            self.get_logger().info('finished: wrote %d samples to %s' % (self._samples, output))
            self.get_logger().info('tracking: %d goal-specific feedback samples to %s' %
                                   (self._tracking.count, self._tracking.path))
            if self._tracking.count == 0:
                self.get_logger().warn('NO_TRACKING_FEEDBACK: current recording is not sufficient for tracking analysis')
        finally:
            self._recording = False
            try:
                if (self._active_handle is not None and self._active_result is not None and
                        not self._active_result.done()):
                    self.get_logger().warn('cancelling the active identification trajectory')
                    cancel = self._active_handle.cancel_goal_async()
                    rclpy.spin_until_future_complete(self, cancel, timeout_sec=2.0)
            finally:
                self._active_handle = None
                self._active_result = None
                if self._csv_file is not None:
                    self._csv_file.close()
                    self._csv_file = None
                if getattr(self, '_tracking', None) is not None:
                    self._tracking.close()
                    self._tracking = None
        # Only reached after action success and file closure, never on failure.
        try:
            report = write_report(output)
            self.get_logger().info('TRACKING_REPORT: matched %d/%d feedback samples; %s' %
                (report['matched_samples'], report['feedback_samples'], report_path(output)))
            for joint in (self._args.joints or ARM_JOINTS):
                entry = report['joints'][joint]
                self.get_logger().info('%s tracking RMSE=%s rad, max=%s rad' %
                    (joint, entry['tracking_rmse_rad'], entry['tracking_max_abs_rad']))
            if report['quality_flags']:
                self.get_logger().warn('TRACKING_QUALITY: ' + ', '.join(report['quality_flags']))
        except (OSError, ValueError, KeyError) as error:
            self.get_logger().warn('TRACKING_ANALYSIS_UNAVAILABLE: %s; raw recordings retained' % error)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true',
                        help='send the trajectory; without it the program only previews limits')
    parser.add_argument('--simulate', action='store_true',
                        help='publish the planned motion on /joint_states for RViz; never contacts hardware')
    parser.add_argument('--record-only', action='store_true',
                        help='passively record joint states until interrupted; never moves the arm')
    parser.add_argument('--rate', type=float, default=100.0,
                        help='maximum recording rate in Hz; actual rate depends on joint states')
    parser.add_argument('--move-to-ready', action='store_true',
                        help='compatibility option: moving to ready is now mandatory with --execute')
    parser.add_argument('--ready-only', action='store_true',
                        help='move to the known ready pose, then exit without a multisine')
    parser.add_argument('--ready-speed', type=float, default=0.12,
                        help='legacy option; Go to READY controller now owns transition timing')
    parser.add_argument('--amplitude-scale', type=float, default=1.0,
                        help='scale the tested multisine amplitude, in (0, 1.5] (default: 1.0)')
    parser.add_argument('--frequency-scale', type=float, default=1.0,
                        help='scale the multisine frequency, in (0, 1.5] (default: 1.0)')
    parser.add_argument('--profile', choices=tuple(PROFILES), default='conservative',
                        help='motion profile: conservative, teaching, or teaching_all (default: conservative)')
    parser.add_argument('--joints', nargs='+', choices=ARM_JOINTS,
                        help='excite only these joints from the selected profile')
    parser.add_argument('--duration', type=float, default=180.0,
                        help='trajectory duration in seconds (default: 180)')
    parser.add_argument('--period', type=float, default=0.05,
                        help='trajectory point interval in seconds (default: 0.05)')
    # The standard bringup exposes this action.  The optional custom controller
    # can still be selected explicitly with --action.
    parser.add_argument('--action', default='/arm_controller/follow_joint_trajectory')
    parser.add_argument('--joint-states', default='/joint_states')
    parser.add_argument(
        '--output',
        default='~/ros2_ws/src/om6dof/om6dof_controllers/papers/experiments/'
                'multisine_identification/data/multisine.csv')
    args, ros_args = parser.parse_known_args()
    if sum((args.execute, args.simulate, args.record_only)) > 1:
        parser.error('--execute, --simulate and --record-only are mutually exclusive')
    if not math.isfinite(args.rate) or not 1.0 <= args.rate <= 500.0:
        parser.error('--rate must be between 1 and 500 Hz')
    if args.joints and set(args.joints) - set(PROFILES[args.profile]['coefficients']):
        parser.error('selected joints are not in the selected motion profile')
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
        if args.record_only:
            node.record_only()
        elif args.execute:
            node.execute()
        elif args.simulate:
            node.get_logger().info('RViz simulation active; publishing planned multisine on %s'
                                   % args.joint_states)
            rclpy.spin(node)
        else:
            node.preview()
            node.get_logger().info('preview only: re-run with --execute to move the arm')
    except KeyboardInterrupt:
        pass
    except (RuntimeError, OSError) as error:
        node.get_logger().error(str(error))
        raise SystemExit(2)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
