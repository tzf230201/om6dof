"""Operator-supervised READY -> Mode 0 -> confirmed supported-pose ramp.

No external service is stopped. The port must be free at startup and again
after our own position launch exits. Separate expiring operator confirmations
are mandatory after TRANSITION -> READY and after supported-pose capture in
Mode 0. No target is sent by activation. Failures retain the current
owned stack for supported inspection/stop; they never retry or change mode.
"""
import copy
import argparse
import json
import math
import os
from pathlib import Path
import secrets
import select
import signal
import subprocess
import sys
import tempfile
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.srv import GetParameters, SetParametersAtomically
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from controller_manager_msgs.srv import ListControllers
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Float64MultiArray
from .impedance_faults import fault_message, limit_description
from .force_preflight import force_pose_problem, load_force_limits, FORCE_JOINT_LIMIT_MARGIN
from .impedance_target import exclusive_controller

JOINTS = [f'joint{i}' for i in range(1, 7)]
CONTROLLER = 'om6dof_position_impedance_controller'
ARM_PORT = '/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT5NUUIQ-if00-port0'
PREFIX = 'IMPEDANCE_STACK '


class ReadyNotSettled(RuntimeError):
    """Fresh measured READY did not remain settled for the required interval."""


class PositionRecoveryRequired(RuntimeError):
    """Mode 0 was entered but impedance activation has NOT been requested."""


class ActiveLimitRecoveryRequired(RuntimeError):
    """Only latched limit fault 32; recovery still requires explicit motion consent."""


class RecoveryPoseInvalid(RuntimeError):
    """Read-only recovery capture/consent invalidated; explicit recapture required."""


def preflight():
    if not os.path.exists(ARM_PORT):
        raise RuntimeError('Arm serial device is unavailable.')
    owner = subprocess.run(['fuser', ARM_PORT], capture_output=True, text=True, timeout=3)
    if owner.returncode != 1 or owner.stderr.strip():
        raise RuntimeError('Serial port busy or ownership unknown; stop the other stack first.')
    processes = subprocess.run(['ps', '-eo', 'comm='], check=True,
                               capture_output=True, text=True, timeout=3)
    if 'ros2_control_no' in processes.stdout:
        raise RuntimeError('Another ros2_control_node exists; no second hardware owner allowed.')


def stop_owned(process):
    if process.poll() is not None:
        return
    for sig, timeout in ((signal.SIGINT, 10), (signal.SIGTERM, 3), (signal.SIGKILL, 2)):
        try:
            os.killpg(process.pid, sig)
            process.wait(timeout=timeout)
            return
        except ProcessLookupError:
            return
        except subprocess.TimeoutExpired:
            pass
    raise RuntimeError('Owned launch did not stop; do not start another hardware stack.')


def trial_config(original, ready, auto_force=False):
    """Build an activation profile, with FORCE's 1 N and zero-margin overrides."""
    result = copy.deepcopy(original)
    params = result['/**'][CONTROLLER]['ros__parameters']
    if params.get('joints') != JOINTS or params.get('allow_mock_hardware') is not False:
        raise ValueError('Stack requires the real six-joint profile.')
    if params.get('require_bus_health') is not True:
        raise ValueError('Bus health must remain required.')
    if len(ready) != 6 or not all(math.isfinite(v) for v in ready):
        raise ValueError('Invalid READY pose.')
    # Loading or entering Mode 0 is NEVER consent to start the gravity ramp.
    params['enable_on_activation'] = False
    params['require_activation_pose'] = True
    params['activation_pose'] = [float(v) for v in ready]
    params['activation_pose_tolerance'] = 0.03
    if auto_force:
        params['joint_limit_margin'] = FORCE_JOINT_LIMIT_MARGIN
        params['force_native_current_limits'] = True
        limits = params.get('max_cartesian_wrench', [])
        if (params.get('enable_cartesian_wrench') is not True or len(limits) != 6
                or any(not math.isfinite(v) or v < 1.0 for v in limits[:3])):
            raise ValueError('Force start requires a configured 1 N Cartesian force capability.')
        params['max_cartesian_wrench'] = [1.0, 1.0, 1.0] + list(limits[3:])
    return result


class PoseWatch:
    def __init__(self):
        self.q = self.v = None
        self.received = self.stamp = 0
        self.epoch = 0
        self.disruption = 'No joint feedback yet.'

    def update(self, msg, now, ros_ns):
        try:
            stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            if stamp <= self.stamp:
                raise ValueError('Joint feedback timestamp did not increase.')
            if not 0 <= ros_ns - stamp < 200_000_000:
                raise ValueError(f'Joint feedback source age {(ros_ns - stamp) / 1e9:.3f}s is invalid.')
            if len(set(msg.name)) != len(msg.name):
                raise ValueError('duplicate joints')
            indices = [msg.name.index(j) for j in JOINTS]
            q, v = ([float(values[i]) for i in indices]
                    for values in (msg.position, msg.velocity))
            if not all(math.isfinite(x) for x in q + v):
                raise ValueError('nonfinite state')
            if now - self.received >= .2:
                self.epoch += 1
                self.disruption = f'Joint feedback gap {now - self.received:.3f}s exceeded 0.200s.'
            self.q, self.v, self.received, self.stamp = q, v, now, stamp
        except (ValueError, IndexError, TypeError) as exc:
            self.q = self.v = None
            self.epoch += 1
            self.disruption = f'Invalid joint feedback: {exc}'

    def problem(self, pose, now):
        if self.q is None:
            return self.disruption
        age = now - self.received
        if not 0 <= age < .2:
            return f'Joint feedback receipt age {age:.3f}s is invalid (limit 0.200s).'
        for joint, actual, target in zip(JOINTS, self.q, pose):
            if abs(actual - target) > .03:
                return f'{joint} pose error {math.degrees(actual - target):+.2f}deg exceeds 1.72deg.'
        for joint, velocity in zip(JOINTS, self.v):
            if abs(velocity) > .05:
                return f'{joint} velocity {velocity:+.3f}rad/s exceeds 0.050rad/s.'
        return ''

    def at(self, pose, now, settled=True):
        return (self.q is not None and 0 <= now - self.received < .2
                and all(abs(a - b) <= .03 for a, b in zip(self.q, pose))
                and (not settled or all(abs(v) <= .05 for v in self.v)))


class ImpedanceStack(Node):
    def __init__(self, auto_force=False):
        super().__init__('om6dof_impedance_stack')
        self.auto_force = auto_force
        self.pose = PoseWatch()
        self.children = []
        self.bridge_at = 0.
        self.bridge_ready = False
        self.controller_status = self.controller_fault = -1
        self.controller_at = self.limit_at = 0.
        self.limit_detail = ''
        self.current_limits_ok = False
        self.create_subscription(JointState, '/joint_states', self.on_pose, 1)
        self.create_subscription(String, '/om6dof/impedance_target/status', self.on_bridge, 1)
        self.create_subscription(Float64MultiArray, '/' + CONTROLLER + '/state', self.on_state, 1)
        self.create_subscription(Float64MultiArray, '/' + CONTROLLER + '/limit_event', self.on_limit, 1)
        self.force_mode_pub = self.create_publisher(String, '/om6dof/operation_mode', 1)
        self.force_mode_state_pub = self.create_publisher(
            String, '/om6dof/operation_mode/state', QoSProfile(
                depth=1, reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def phase(self, phase, message, token=''):
        print(PREFIX + json.dumps(dict(phase=phase, message=message, token=token)), flush=True)

    def on_pose(self, msg):
        self.pose.update(msg, time.monotonic(), self.get_clock().now().nanoseconds)

    def on_bridge(self, msg):
        try:
            data = json.loads(msg.data)
            age = self.get_clock().now().nanoseconds - data['stamp_ns']
            key = 'force_ready' if getattr(self, 'auto_force', False) else 'ready'
            self.bridge_ready = data.get(key) is True and 0 <= age < 200_000_000
            self.bridge_at = time.monotonic()
        except (ValueError, KeyError, TypeError):
            self.bridge_ready = False

    def on_limit(self, msg):
        self.limit_detail = limit_description(msg.data, JOINTS)
        self.current_limits_ok = bool(self.limit_detail and msg.data[6] == 1)
        self.limit_at = time.monotonic()

    def recovery_limits_valid(self):
        return (self.current_limits_ok and 0 <= time.monotonic() - self.limit_at < .2)

    def on_state(self, msg):
        if len(msg.data) != 44 or not all(math.isfinite(x) for x in msg.data):
            self.controller_status = self.controller_fault = -1
            self.controller_at = 0.
            return
        self.controller_status, self.controller_fault = msg.data[:2]
        self.controller_at = time.monotonic()

    def limit_fault_fresh(self):
        return (getattr(self, 'controller_status', -1) == 3
                and getattr(self, 'controller_fault', -1) == 32
                and 0 <= time.monotonic() - getattr(self, 'controller_at', 0.) < .2)

    def check_limit_fault(self):
        if self.limit_fault_fresh():
            detail = (self.limit_detail if 0 <= time.monotonic() - self.limit_at < .2 else '')
            raise ActiveLimitRecoveryRequired(fault_message(3, 32, detail))

    def activation_ready(self):
        self.check_limit_fault()
        return self.bridge_ready and 0 <= time.monotonic() - self.bridge_at < .2

    def spawn(self, command):
        process = subprocess.Popen(command, stdin=subprocess.DEVNULL, start_new_session=True)
        self.children.append(process)
        return process

    def pump(self):
        if not rclpy.ok():
            raise KeyboardInterrupt
        # EOF means the owning web process is gone; never leave orphan launches.
        if select.select([sys.stdin], [], [], 0)[0]:
            line = sys.stdin.readline()
            if not line:
                raise KeyboardInterrupt
        else:
            line = ''
        rclpy.spin_once(self, timeout_sec=.02)
        return line.strip()

    def wait(self, predicate, timeout, reason):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.pump()
            if any(p.poll() is not None for p in self.children):
                raise RuntimeError('Owned launch exited unexpectedly; ' + reason)
            if predicate():
                return
        raise RuntimeError(reason)

    def call(self, service, name, request, timeout=15., before_send=None):
        client = self.create_client(service, name)
        try:
            self.wait(client.service_is_ready, timeout, name + ' unavailable')
            if before_send is not None:
                before_send()
            future = client.call_async(request)
            self.wait(future.done, timeout, name + ' timed out')
            result = future.result()
            if result is None:
                raise RuntimeError(name + ' returned no response')
            return result
        finally:
            self.destroy_client(client)

    def stable_ready(self, ready, timeout=12.):
        since = [None]
        epoch = [self.pose.epoch]
        last_problem = ['No continuous one-second READY interval.']
        def stable():
            now = time.monotonic()
            if epoch[0] != self.pose.epoch:
                since[0] = None
                epoch[0] = self.pose.epoch
                last_problem[0] = self.pose.disruption
            if not self.pose.at(ready, now):
                since[0] = None
                last_problem[0] = self.pose.problem(ready, now)
            elif since[0] is None:
                since[0] = now
            return since[0] is not None and now - since[0] >= 1.
        reason = 'Fresh measured READY did not settle for one second.'
        try:
            self.wait(stable, timeout, reason)
        except RuntimeError as exc:
            if str(exc) != reason:
                raise  # Process/service failures must not become pose recovery.
            raise ReadyNotSettled(reason + ' ' + last_problem[0]) from exc

    def prepare(self):
        preflight()
        self.phase('preparing', 'Starting dedicated position stack; will move TRANSITION → READY.')
        self.spawn(['ros2', 'launch', 'om6dof_bringup', 'hardware.launch.py',
                    'current_control:=false', 'arm_operating_mode:=3'])
        self.spawn(['ros2', 'launch', 'om6dof_controllers', 'controller.launch.py'])
        position = list(self.children)
        req = GetParameters.Request(names=['transition_pose_degrees', 'ready_pose_degrees',
                                          'operation_mode_topic', 'pose_profile_duration_seconds'])
        values = self.call(GetParameters, '/om6dof_controller/get_parameters', req, 30.).values
        if len(values) != 4:
            raise RuntimeError('Incomplete canonical READY configuration.')
        transition, ready = [list(map(math.radians, v.double_array_value)) for v in values[:2]]
        if any(len(p) != 6 or not all(math.isfinite(v) for v in p) for p in (transition, ready)):
            raise RuntimeError('Invalid canonical READY configuration.')
        topic, duration = values[2].string_value, values[3].double_value
        if not topic or not math.isfinite(duration) or not 0 < duration <= 30:
            raise RuntimeError('Invalid READY topic or duration.')
        states = self.call(ListControllers, '/controller_manager/list_controllers', ListControllers.Request())
        if not any(c.name == 'arm_controller' and c.state == 'active' for c in states.controller):
            # Startup spawners may still be in progress. Do not send a READY
            # request until the trajectory controller is actually active.
            # On the AGX the controller-manager can still be loading the arm
            # controller while joint-state/offset spawners are starting. This
            # is only a readiness wait for force-start pose capture; it never
            # sends READY.
            self.wait(lambda: self.arm_active(), 60., 'Position controller not active.')
        if self.auto_force:
            # Force mode starts from the measured pose already held by the
            # mechanism.  It must never publish READY or TRANSITION.
            self.wait(lambda: self.pose.q is not None and time.monotonic() - self.pose.received < .2,
                      10., 'Current force-start pose unavailable.')
            current_pose = list(self.pose.q)
            self._force_start_limits = load_force_limits()
            problem = force_pose_problem(current_pose, self.pose.v, self._force_start_limits)
            if problem:
                raise RuntimeError('Force start refused before Mode 0: ' + problem + ' Position stack retained.')
            self.phase('preparing', 'Force mode: holding the current pose; READY is skipped.')
            self.stable_ready(current_pose, 5.)
            return position, current_pose
        publisher = self.create_publisher(String, topic, 1)
        self.wait(lambda: publisher.get_subscription_count() > 0 and self.pose.q is not None
                  and time.monotonic() - self.pose.received < .2, 10., 'READY receiver/state unavailable.')
        requested = time.monotonic()
        publisher.publish(String(data='READY'))
        self.phase('moving_ready', 'Go to READY requested: waiting for measured TRANSITION, then READY.')
        self.wait(lambda: self.pose.received > requested and self.pose.at(transition, time.monotonic(), False),
                  2 * duration + 15, 'TRANSITION was not observed; impedance will not start.')
        self.phase('moving_ready', 'TRANSITION confirmed; waiting for settled READY.')
        self.stable_ready(ready, duration + 15)
        if not self.auto_force:
            self.await_support(position, ready)
        return position, ready

    def await_support(self, position, ready):
        # Recovery is read-only and operator-requested, only at this pre-switch
        # stage. Never recover a dead launch, a mode-switch fault, or an active
        # impedance fault. Each recheck requires a new support confirmation.
        while True:
            token = secrets.token_hex(16)
            epoch = self.pose.epoch
            self.phase('await_support', 'READY confirmed. Support the arm, then confirm within 60 seconds.', token)
            deadline = time.monotonic() + 60.
            while True:
                line = self.pump()
                if any(p.poll() is not None for p in position):
                    raise RuntimeError('Owned position launch exited before confirmation; no mode switch.')
                now = time.monotonic()
                reason = self.pose.problem(ready, now)
                if not reason and self.pose.epoch != epoch:
                    reason = self.pose.disruption
                if not reason and now >= deadline:
                    reason = 'Support confirmation expired.'
                if reason:
                    self.phase('ready_blocked', reason + ' No mode switch. Position stack retained. '
                               'Use Check READY again (read-only); support must be confirmed again.')
                    break
                if line == 'supported ' + token:
                    # The live controller must still own position; service reads
                    # may take time, so settle fresh feedback again afterward.
                    if not self.arm_active():
                        raise RuntimeError('Position controller no longer active; no mode switch.')
                    self.stable_ready(ready)
                    return
            while True:
                line = self.pump()
                if any(p.poll() is not None for p in position):
                    raise RuntimeError('Owned position launch exited; cannot recheck READY.')
                if line == 'recheck_ready':
                    self.phase('checking_ready', 'Read-only READY check; no motion or mode switch requested.')
                    if not self.arm_active():
                        raise RuntimeError('Position controller no longer active; cannot recheck READY.')
                    self.stable_ready(ready)
                    break

    def arm_active(self):
        result = self.call(ListControllers, '/controller_manager/list_controllers', ListControllers.Request(), 2.)
        return any(c.name == 'arm_controller' and c.state == 'active' for c in result.controller)

    def confirm_supported_pose(self):
        """Capture a frozen post-switch pose, then require NEW operator consent.

        No position goal is sent. A disturbed pose cannot silently become a new
        reference: it invalidates the token and needs an explicit read-only
        recapture. Initial TRANSITION -> READY was already completed by prepare.
        """
        if self.auto_force:
            self.phase('capturing_pose', 'Force start: waiting for a stationary Mode-0 pose.')
            anchor, since, epoch = [None], [None], [self.pose.epoch]
            def settled():
                now = time.monotonic()
                if self.pose.epoch != epoch[0] or self.pose.q is None:
                    anchor[0], since[0], epoch[0] = None, None, self.pose.epoch
                    return False
                if anchor[0] is None or not self.pose.at(anchor[0], now):
                    anchor[0], since[0] = list(self.pose.q), now
                return now - since[0] >= 1.
            self.wait(settled, 60., 'Mode-0 pose did not settle for force start.')
            problem = force_pose_problem(self.pose.q, self.pose.v, self._force_start_limits)
            if problem:
                raise RuntimeError('Mode-0 activation refused: ' + problem + ' Keep supporting the arm and Stop.')
            return list(self.pose.q), self.pose.epoch, time.monotonic() + 5.
        while True:
            self.phase('capturing_pose', 'Mode 0: no gravity support yet. Keep supporting at a stable pose; '
                       'capturing measured joints for a NEW confirmation, not moving to READY.')
            anchor, since, epoch = [None], [None], [self.pose.epoch]
            def settled():
                now = time.monotonic()
                if self.pose.epoch != epoch[0]:
                    anchor[0], since[0], epoch[0] = None, None, self.pose.epoch
                if self.pose.q is None or not self.pose.at(self.pose.q, now):
                    anchor[0], since[0] = None, None
                    return False
                if anchor[0] is None or not self.pose.at(anchor[0], now):
                    anchor[0], since[0] = list(self.pose.q), now
                return now - since[0] >= 1.
            try:
                self.wait(settled, 60., 'Supported pose did not settle.')
            except RuntimeError as exc:
                if str(exc) == 'Supported pose did not settle.':
                    raise PositionRecoveryRequired('Mode 0, impedance NOT active. ' + str(exc)) from exc
                raise
            pose, captured_epoch = list(self.pose.q), self.pose.epoch
            token, deadline = secrets.token_hex(16), time.monotonic() + 60.
            angles = ', '.join(f'{j}={math.degrees(q):+.2f}°' for j, q in zip(JOINTS, pose))
            self.phase('await_pose_support', 'Captured supported pose: ' + angles + '. No gravity support yet. '
                       'Confirm THIS pose within 60 seconds to request the guarded gravity ramp; '
                       'joint limits, speed, bus health and gravity capacity are checked before torque.', token)
            while True:
                line = self.pump()
                if any(p.poll() is not None for p in self.children):
                    raise RuntimeError('Owned launch exited before supported-pose confirmation.')
                now = time.monotonic()
                problem = self.pose.problem(pose, now)
                if self.pose.epoch != captured_epoch:
                    problem = self.pose.disruption
                if now >= deadline:
                    problem = 'Supported-pose confirmation expired.'
                if problem:
                    self.phase('pose_blocked', problem + ' Impedance NOT active. Keep supporting and use '
                               'Capture supported pose again (no motion), then confirm the NEW pose.')
                    break
                if line == 'supported_pose ' + token:
                    return pose, captured_epoch, deadline
            while True:
                line = self.pump()
                if any(p.poll() is not None for p in self.children):
                    raise RuntimeError('Owned launch exited; cannot recapture supported pose.')
                if line == 'recapture_pose':
                    break

    def check_confirmed_pose(self, pose, epoch, deadline):
        if (time.monotonic() >= deadline or self.pose.epoch != epoch
                or not self.pose.at(pose, time.monotonic())):
            raise RuntimeError('Confirmed supported pose changed/expired; no activation. Support and Stop.')

    def arm_confirmed_pose(self, pose, epoch, deadline):
        states = self.call(ListControllers, '/controller_manager/list_controllers', ListControllers.Request())
        matches = [c for c in states.controller if c.name == CONTROLLER]
        commands = {f'{j}/{kind}' for j in JOINTS for kind in ('position', 'velocity', 'effort')}
        if (len(matches) != 1 or matches[0].state != 'inactive' or
                any(c.state == 'active' and commands.intersection(c.claimed_interfaces) for c in states.controller)):
            raise RuntimeError('Supported-pose activation refused: inactive exclusive impedance not confirmed.')
        def check():
            self.check_confirmed_pose(pose, epoch, deadline)
        check()
        request = SetParametersAtomically.Request(parameters=[
            Parameter(name='activation_pose', value=ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE_ARRAY, double_array_value=pose)),
            Parameter(name='enable_on_activation', value=ParameterValue(
                type=ParameterType.PARAMETER_BOOL, bool_value=True))])
        result = self.call(SetParametersAtomically, '/' + CONTROLLER + '/set_parameters_atomically',
                           request, before_send=check)
        if not result.result.successful:
            raise RuntimeError('Confirmed pose parameters rejected: ' + result.result.reason)
        check()  # A parameter-service delay cannot make stale consent actionable.

    def verify_supported_pose_gate(self):
        result = self.call(GetParameters, '/' + CONTROLLER + '/get_parameters',
                          GetParameters.Request(names=['supported_pose_protocol_version',
                                                       'require_activation_pose', 'enable_on_activation']))
        values = result.values
        if (len(values) != 3 or values[0].type != ParameterType.PARAMETER_INTEGER
                or values[0].integer_value != 1 or values[1].type != ParameterType.PARAMETER_BOOL
                or not values[1].bool_value or values[2].type != ParameterType.PARAMETER_BOOL
                or values[2].bool_value):
            raise RuntimeError('Supported-pose controller version/gate mismatch. Load the rebuilt plugin; no activation.')

    def activate(self, position, ready, params_path):
        self.phase('switching', 'Support confirmed. Stopping our position stack; keep supporting the arm.')
        for process in reversed(position):
            stop_owned(process)
            self.children.remove(process)
        preflight()  # mandatory, never overlap U2D2 owners
        self.pose = PoseWatch()  # require measurements from the new hardware instance
        self.phase('starting', 'Starting Mode 0 with zero initial current; support required.')
        self.spawn(['ros2', 'launch', 'om6dof_bringup', 'hardware.launch.py',
                    'current_control:=true', 'arm_operating_mode:=0', 'start_arm_controller:=false'])
        self.spawn(['ros2', 'launch', 'om6dof_controllers', 'position_impedance.launch.py',
                    'params_file:=' + str(params_path)])
        def configured():
            result = self.call(ListControllers, '/controller_manager/list_controllers', ListControllers.Request(), 5.)
            for c in result.controller:
                if c.name == CONTROLLER:
                    if c.state == 'active':
                        raise RuntimeError('Unexpected automatic activation; support and stop.')
                    return c.state == 'inactive'
            return False
        self.wait(configured, 40., 'Impedance did not configure inactive.')
        self.verify_supported_pose_gate()
        pose, epoch, deadline = self.confirm_supported_pose()
        self.arm_confirmed_pose(pose, epoch, deadline)
        # Exactly one activation request; the plugin checks bus clean window,
        # explicitly confirmed supported pose, speed, current caps and fresh state.
        from controller_manager_msgs.srv import SwitchController
        request = SwitchController.Request()
        request.activate_controllers = [CONTROLLER]
        request.strictness = SwitchController.Request.STRICT
        request.timeout.sec = 5
        self.phase('activating', 'Supported pose confirmed separately in Mode 0; requesting guarded gravity ramp. '
                   'Keep supporting; no position target is sent.')
        if not self.call(SwitchController, '/controller_manager/switch_controller', request,
                         before_send=lambda: self.check_confirmed_pose(pose, epoch, deadline)).ok:
            raise RuntimeError('Impedance activation rejected. Keep supporting; inspect log and Stop.')
        self.wait(self.activation_ready,
                  15., 'Impedance readiness not confirmed. Keep supporting; inspect log and Stop.')
        if self.auto_force:
            self.force_mode_pub.publish(String(data='FORCE'))
            self.force_mode_state_pub.publish(String(data='FORCE'))
            self.phase('active', 'Gravity compensation + damping ready. Joystick requests a 1 N resultant force; '
                       'joint torque budgets may reduce it. Check force telemetry.')
        else:
            self.phase('active', 'Position impedance ready. Use Target mode → Run impedance target; no target sent yet.')
        while True:
            self.pump()
            if any(p.poll() is not None for p in self.children):
                raise RuntimeError('Owned stack exited; no automatic restart.')
            self.check_limit_fault()
            if not self.bridge_ready or time.monotonic() - self.bridge_at > .5:
                raise RuntimeError('Impedance fault or stale readiness; no automatic reactivation.')

    def recover_position(self, reason, active_limit_fault=False):
        """Explicit single-use motion consent, not an in-place fault clear.

        Failure to capture a fresh stationary supported pose cannot be
        repaired by blind activation. Stop only our children and
        return through the normal position TRANSITION -> READY preparation.
        """
        if active_limit_fault:
            self.confirm_limit_recovery(reason)
            self.restart_owned_for_recovery()
            return
        token = secrets.token_hex(16)
        self.phase('await_recovery', reason + ' Keep supporting. Recovery requires explicit approval '
                   'to restart position control and MOVE through TRANSITION → READY within 60 seconds.', token)
        deadline = time.monotonic() + 60.
        while True:
            line = self.pump()
            if any(p.poll() is not None for p in self.children):
                raise RuntimeError('Owned launch exited; supported Stop required, recovery refused.')
            if time.monotonic() >= deadline:
                raise RuntimeError('Recovery consent expired; no motion requested. Support and Stop.')
            if line != 'recover_position ' + token:
                continue
            result = self.call(ListControllers, '/controller_manager/list_controllers',
                               ListControllers.Request(), 3.)
            matches = [c for c in result.controller if c.name == CONTROLLER]
            arm_commands = {f'{joint}/{kind}' for joint in JOINTS
                            for kind in ('position', 'velocity', 'effort')}
            owner_ok = (len(matches) == 1 and matches[0].state == 'inactive'
                        and not any(c.state == 'active' and arm_commands.intersection(c.claimed_interfaces)
                                    for c in result.controller))
            if not owner_ok:
                raise RuntimeError('Recovery refused: inactive impedance/exclusive arm ownership not confirmed.')
            # A service read can take time. Require current stationary measured
            # feedback before handing position control back to the servos.
            now = time.monotonic()
            if (now >= deadline or self.pose.q is None
                    or not self.pose.at(self.pose.q, now)):
                raise RuntimeError('Recovery refused: consent expired or arm feedback stale/moving. Support and Stop.')
            self.restart_owned_for_recovery()
            return

    def check_recovery_feedback(self):
        if any(p.poll() is not None for p in self.children):
            raise RuntimeError('Owned launch exited; support and Stop. Recovery refused.')
        fresh = 0 <= time.monotonic() - self.controller_at < .2
        if fresh and (self.controller_status != 3 or self.controller_fault != 32):
            raise RuntimeError('Recovery refused: fault is no longer limit-only (32); support and Stop.')
        if not self.limit_fault_fresh() or not self.recovery_limits_valid():
            raise RecoveryPoseInvalid('Recovery feedback stale or current position/speed outside limits.')

    def check_recovery_pose(self, pose, epoch, deadline):
        self.check_recovery_feedback()
        now = time.monotonic()
        if now >= deadline:
            raise RecoveryPoseInvalid('Recovery confirmation expired.')
        problem = self.pose.problem(pose, now)
        if self.pose.epoch != epoch:
            problem = self.pose.disruption
        if problem:
            raise RecoveryPoseInvalid('Supported recovery pose invalid: ' + problem)

    def confirm_limit_recovery(self, reason):
        """Capture BEFORE consent; movement invalidates consent, never changes its pose."""
        while True:
            self.phase('capturing_recovery_pose', reason + ' Hold the arm still. Read-only capture of '
                       'the supported recovery pose, NOT READY; no restart or motion yet.')
            anchor, since, epoch = None, None, self.pose.epoch
            deadline = time.monotonic() + 12.
            try:
                while True:
                    self.pump()  # All old consent lines are ignored during capture.
                    self.check_recovery_feedback()
                    now = time.monotonic()
                    if now >= deadline:
                        raise RecoveryPoseInvalid('Supported recovery pose did not settle for one second.')
                    if epoch != self.pose.epoch:
                        anchor, since, epoch = None, None, self.pose.epoch
                    if self.pose.q is None or not self.pose.at(self.pose.q, now):
                        anchor, since = None, None
                    elif anchor is None or not self.pose.at(anchor, now):
                        anchor, since = list(self.pose.q), now
                    if since is not None and now - since >= 1.:
                        break
                pose, epoch = list(self.pose.q), self.pose.epoch
                token, deadline = secrets.token_hex(16), time.monotonic() + 60.
                angles = ', '.join(f'{j}={math.degrees(q):+.2f}°' for j, q in zip(JOINTS, pose))
                self.phase('await_recovery', 'Supported recovery pose captured (not READY): ' + angles +
                           '. Confirm support AND permission to MOVE through TRANSITION → READY within 60 seconds. '
                           'Restart removes gravity support. No old target resumes.', token)
                while True:
                    line = self.pump()
                    self.check_recovery_pose(pose, epoch, deadline)
                    if line != 'recover_position ' + token:
                        continue
                    result = self.call(ListControllers, '/controller_manager/list_controllers',
                                       ListControllers.Request(), 3.)
                    if not exclusive_controller(result.controller):
                        raise RuntimeError('Recovery refused: exclusive active impedance ownership lost; support and Stop.')
                    # A slow service cannot make moved/stale/expired consent usable.
                    self.check_recovery_pose(pose, epoch, deadline)
                    return
            except RecoveryPoseInvalid as exc:
                self.phase('recovery_pose_blocked', reason + ' Recovery check: ' + str(exc) + ' No restart or motion requested. '
                           'Keep supporting; Capture recovery pose again, then give NEW support and motion consent.')
                while True:
                    line = self.pump()
                    if any(p.poll() is not None for p in self.children):
                        raise RuntimeError('Owned launch exited; support and Stop.')
                    if (0 <= time.monotonic() - self.controller_at < .2 and
                            (self.controller_status != 3 or self.controller_fault != 32)):
                        raise RuntimeError('Additional/changed controller fault; support and Stop.')
                    if line == 'recapture_recovery':
                        break

    def restart_owned_for_recovery(self):
        self.phase('recovering', 'Recovery approved. Keep supporting while owned Mode-0 stack stops; '
                   'position control will then MOVE through TRANSITION → READY. Impedance remains unconfirmed.')
        for child in reversed(list(self.children)):
            stop_owned(child)
            self.children.remove(child)
        preflight()  # Never overlap serial owners.
        self.pose = PoseWatch()
        self.bridge_ready, self.bridge_at = False, 0.
        self.controller_status = self.controller_fault = -1
        self.controller_at = self.limit_at = 0.
        self.limit_detail = ''
        self.current_limits_ok = False


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--auto-force', action='store_true')
    options, ros_args = parser.parse_known_args(args)
    rclpy.init(args=ros_args)
    node = ImpedanceStack(auto_force=options.auto_force)
    try:
        while True:
            position, ready = node.prepare()
            source = Path(get_package_share_directory('om6dof_controllers')) / 'config/position_impedance.yaml'
            config = trial_config(yaml.safe_load(source.read_text()), ready, node.auto_force)
            try:
                # Ephemeral profile: never rewrite the user's YAML or identified values.
                with tempfile.TemporaryDirectory(prefix='om6dof-impedance-') as directory:
                    path = Path(directory) / 'trial.yaml'
                    path.write_text(yaml.safe_dump(config))
                    node.activate(position, ready, path)
                break
            except PositionRecoveryRequired as exc:
                node.recover_position(str(exc))
                # Only operator-authorized recovery reaches another preparation;
                # prepare still observes TRANSITION and requires NEW support consent.
            except ActiveLimitRecoveryRequired as exc:
                node.recover_position(str(exc), active_limit_fault=True)
                # Full owned-stack reload, canonical TRANSITION -> READY, and
                # BOTH new support confirmations; never clear/replay a target.
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        node.phase('failed', str(exc) + ' Existing owned processes retained; support and press Stop.')
        try:
            while True:
                node.pump()
        except KeyboardInterrupt:
            pass
    finally:
        failed_cleanup = False
        for child in reversed(node.children):
            try:
                stop_owned(child)
            except Exception as exc:
                failed_cleanup = True
                node.phase('failed', f'Child shutdown failed: {exc}. Verify port ownership before any restart.')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if failed_cleanup:
            raise SystemExit(1)


if __name__ == '__main__':
    main()
