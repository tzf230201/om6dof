"""Commissioning-only Mode-5 admittance with explicit position ownership.

STRICT controller switches transfer position between JTC and a direct forward
controller. Direct commands require its ACK. No position offset or cubic
trajectory tail is used. Mode 5, PID and current-budget control stay intact.
"""

import copy
from dataclasses import fields
import math
import time

import numpy as np
from builtin_interfaces.msg import Duration
from control_msgs.msg import JointTrajectoryControllerState
from controller_manager_msgs.srv import ListControllers, SwitchController
from std_msgs.msg import Float64MultiArray

from .admittance import AdmittanceConfig, JointAdmittance


class AdmittanceTargetMixin:
    _admittance_enabled = False
    _admittance_target_active = False
    _admittance_result = None
    _arm_action_active = False
    _arm_state_updated = 0.0
    _arm_state_stamp_seconds = 0.0
    _admittance_stopping = False
    _admittance_switch_pending = False
    _admittance_owner = 'position'
    _admittance_phase = 'idle'

    def _initialize_admittance(self):
        self.declare_parameter('admittance.enabled', False)
        self.declare_parameter('admittance.calibration_verified', False)
        self.declare_parameter('admittance.observer_timeout_seconds', 0.12)
        self._admittance_enabled = bool(self.get_parameter('admittance.enabled').value)
        self._admittance_calibrated = bool(self.get_parameter('admittance.calibration_verified').value)
        self._admittance_timeout = float(self.get_parameter('admittance.observer_timeout_seconds').value)
        if not math.isfinite(self._admittance_timeout) or not 0.02 <= self._admittance_timeout <= 0.25:
            raise RuntimeError('invalid admittance observer timeout')
        defaults = AdmittanceConfig(self.joint_lower, self.joint_upper,
            [208.541119, 208.541119, 208.541119, 285.006196, 208.541119, 285.006196])
        parameters = {}
        for field in fields(AdmittanceConfig):
            if field.name in ('lower', 'upper'):
                continue
            default = getattr(defaults, field.name)
            self.declare_parameter('admittance.' + field.name,
                list(default) if isinstance(default, tuple) else default)
            parameters[field.name] = self.get_parameter('admittance.' + field.name).value
        self._admittance_config = AdmittanceConfig(self.joint_lower, self.joint_upper, **parameters)
        self._admittance_core = JointAdmittance(self._admittance_config)
        self._admittance_sample = None
        self._admittance_sample_time = 0.0
        self._admittance_last_command = None
        self._admittance_last_velocity = [0.0] * 6
        self._admittance_direct_initialized = False
        self._admittance_switch_generation = 0
        self._admittance_switch_pending = False
        self._admittance_stop_requested = False
        self._admittance_recovery_requested = False
        self._admittance_stop_pose = None
        self._admittance_stop_settled_since = None
        self._admittance_reason = ''
        self._admittance_owner = 'position'
        self._admittance_phase = 'idle'
        self._admittance_arm_controller = 'arm_controller'
        self._admittance_direct_controller = 'forward_position_controller'
        self._admittance_direct_topic = '/forward_position_controller/commands'
        self._admittance_controllers = None
        self._admittance_controllers_updated = 0.0
        self._admittance_list_pending = False
        self._admittance_list_requested = 0.0
        self._admittance_list_generation = 0
        self._admittance_direct_pub = self.create_publisher(
            Float64MultiArray, self._admittance_direct_topic, 1)
        self._admittance_reference_pub = self.create_publisher(
            JointTrajectoryControllerState, '/om6dof/admittance/reference', 1)
        self._admittance_switch_client = self.create_client(
            SwitchController, '/controller_manager/switch_controller')
        self._admittance_list_client = self.create_client(
            ListControllers, '/controller_manager/list_controllers')
        self.create_subscription(Float64MultiArray,
            '/gravity_current_budget_controller/observer_sample', self._on_admittance_sample, 10)

    def _admittance_ros_seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_admittance_arm_state(self, message):
        if not self._admittance_enabled:
            return
        stamp = message.header.stamp
        if stamp.sec < 0 or stamp.nanosec >= 1000000000:
            return
        with self.lock:
            self._arm_state_stamp_seconds = stamp.sec + stamp.nanosec * 1e-9
            self._arm_state_updated = time.monotonic()

    def _on_admittance_sample(self, message):
        values = list(message.data)
        with self.lock:
            self._admittance_sample = None
            if len(values) != 32 or not all(math.isfinite(v) for v in values):
                return
            age = self._admittance_ros_seconds() - values[0]
            if age < -0.05 or age > self._admittance_timeout or values[1] not in range(6):
                return
            self._admittance_sample = values
            self._admittance_sample_time = time.monotonic()

    def _admittance_sample_ready_locked(self, allow_saturation=False):
        now = time.monotonic()
        sample = getattr(self, '_admittance_sample', None)
        if sample is None:
            return False, 'Admittance observer sample is unavailable.'
        source_age = self._admittance_ros_seconds() - sample[0]
        receipt_age = now - self._admittance_sample_time
        if (not -0.05 <= source_age <= self._admittance_timeout
                or not 0.0 <= receipt_age <= self._admittance_timeout):
            return False, 'Admittance observer sample is stale.'
        reasons = {1: 'observer startup ramp', 2: 'bus/current-budget fault',
                   3: 'invalid motor telemetry', 4: 'dynamics solver failure',
                   5: 'current saturation; support/contact cannot be distinguished'}
        quality = int(sample[1])
        if quality != 0 and not (allow_saturation and quality == 5):
            return False, 'Admittance paused: ' + reasons.get(quality, 'invalid quality') + '.'
        if any(not lo <= q <= hi for q, lo, hi in zip(
                sample[2:8], self._admittance_config.lower, self._admittance_config.upper)):
            return False, 'Admittance measurement exceeds joint limits.'
        return True, ''

    def _poll_admittance_controllers_locked(self, now):
        client = self._admittance_list_client
        if self._admittance_list_pending:
            if now - self._admittance_list_requested <= 0.6:
                return
            self._admittance_list_pending = False
            self._admittance_list_generation += 1
        if now - self._admittance_list_requested < 0.2 or not client.service_is_ready():
            return
        self._admittance_list_requested = now
        self._admittance_list_pending = True
        self._admittance_list_generation += 1
        generation = self._admittance_list_generation
        try:
            future = client.call_async(ListControllers.Request())
            future.add_done_callback(lambda result: self._on_admittance_controllers(generation, result))
        except Exception:
            self._admittance_list_pending = False

    def _on_admittance_controllers(self, generation, future):
        with self.lock:
            if generation != self._admittance_list_generation:
                return
            self._admittance_list_pending = False
            try:
                controllers = {entry.name: entry.state for entry in future.result().controller}
            except Exception:
                return
            self._admittance_controllers = controllers
            self._admittance_controllers_updated = time.monotonic()
            if (self._admittance_owner == 'uncertain' and self._admittance_recovery_requested
                    and not self._admittance_switch_pending and not self._arm_action_active):
                arm = controllers.get(self._admittance_arm_controller)
                direct = controllers.get(self._admittance_direct_controller)
                if arm == 'inactive' and direct == 'active':
                    # Explicit Stop may recover a lost switch response, but a
                    # new ACK is required before emitting any direct command.
                    self._admittance_recovery_requested = False
                    self._request_admittance_switch_locked('direct')
                elif arm == 'active' and direct == 'inactive':
                    self._admittance_recovery_requested = False
                    self._request_admittance_switch_locked('position')

    def _admittance_resources_ready_locked(self):
        now = time.monotonic()
        self._poll_admittance_controllers_locked(now)
        controllers = self._admittance_controllers
        if controllers is None or not 0.0 <= now - self._admittance_controllers_updated <= 0.6:
            return False, 'Checking fresh controller ownership.'
        arm = controllers.get(self._admittance_arm_controller)
        direct = controllers.get(self._admittance_direct_controller)
        expected = ((arm == 'inactive' and direct == 'active') if self._admittance_owner == 'direct'
                    else (arm == 'active' and direct == 'inactive'))
        if not expected:
            return False, 'Position controller ownership is not confirmed.'
        for name in ('gravity_current_budget_controller', 'forward_offset_controller'):
            if controllers.get(name) != 'active':
                return False, f'Admittance requires active {name}.'
        return True, ''

    def _admittance_exclusive_locked(self):
        if self.count_publishers(self._admittance_direct_topic) != 1:
            return False, 'Admittance requires one exclusive direct-position publisher.'
        if self.count_publishers(self.offset_pub.topic_name) != 1:
            return False, 'Admittance requires one exclusive zero-offset publisher.'
        if self._admittance_direct_pub.get_subscription_count() < 1:
            return False, 'Direct position controller subscription is unavailable.'
        if np.any(np.abs(self.offset) > 1e-6):
            return False, 'Clear the operator offset before a compliant target.'
        return True, ''

    def _admittance_ready_locked(self):
        if not self._admittance_enabled:
            return True, ''
        if not self._admittance_calibrated:
            return False, 'Admittance needs verified current signs, model bias and contact thresholds.'
        if self._admittance_switch_pending:
            return False, 'Waiting for explicit position-controller switch confirmation.'
        if self._admittance_owner == 'uncertain':
            return False, 'Position ownership is uncertain; use Stop after checking the controller state.'
        ready, reason = self._admittance_sample_ready_locked()
        if ready:
            ready, reason = self._admittance_resources_ready_locked()
        if ready:
            ready, reason = self._admittance_exclusive_locked()
        if ready and self._admittance_owner == 'position':
            now = time.monotonic()
            source_age = self._admittance_ros_seconds() - self._arm_state_stamp_seconds
            if (not 0.0 <= now - self._arm_state_updated <= self._admittance_timeout
                    or not -0.05 <= source_age <= self._admittance_timeout):
                ready, reason = False, 'Admittance needs fresh trajectory-controller feedback.'
        return ready, reason

    def _request_admittance_switch_locked(self, destination):
        if self._admittance_switch_pending:
            return
        if not self._admittance_switch_client.service_is_ready():
            self._admittance_recovery_requested = False
            self._admittance_owner = 'uncertain'
            self._admittance_phase = 'blocked'
            self._admittance_reason = 'Controller-switch service is unavailable; ownership is uncertain.'
            self._publish_target_status_locked('blocked', self._admittance_reason)
            return
        request = SwitchController.Request()
        request.strictness = SwitchController.Request.STRICT
        request.activate_asap = False
        request.timeout = Duration(sec=2)
        if destination == 'direct':
            request.activate_controllers = [self._admittance_direct_controller]
            request.deactivate_controllers = [self._admittance_arm_controller]
        else:
            request.activate_controllers = [self._admittance_arm_controller]
            request.deactivate_controllers = [self._admittance_direct_controller]
        # Recovery may observe the requested resource state already in place
        # after an ACK was lost. A STRICT no-op confirms the service path while
        # the fresh ListControllers response supplies the ownership evidence.
        snapshot = self._admittance_controllers or {}
        request.activate_controllers = [name for name in request.activate_controllers
                                        if snapshot.get(name) != 'active']
        request.deactivate_controllers = [name for name in request.deactivate_controllers
                                          if snapshot.get(name) != 'inactive']
        self._admittance_switch_pending = True
        self._admittance_switch_requested = time.monotonic()
        self._admittance_switch_generation += 1
        generation = self._admittance_switch_generation
        self._admittance_phase = 'switching_to_direct' if destination == 'direct' else 'restoring'
        try:
            future = self._admittance_switch_client.call_async(request)
            future.add_done_callback(
                lambda result: self._on_admittance_switch(generation, destination, result))
        except Exception as exc:
            self._admittance_recovery_requested = False
            self._admittance_switch_pending = False
            self._admittance_owner = 'uncertain'
            self._admittance_phase = 'blocked'
            self._admittance_reason = f'Controller-switch result is unknown ({exc}); no automatic retry.'
            self._publish_target_status_locked('blocked', self._admittance_reason)

    def _on_admittance_switch(self, generation, destination, future):
        with self.lock:
            if (generation != self._admittance_switch_generation
                    or not self._admittance_switch_pending):
                return
            self._admittance_switch_pending = False
            try:
                if future.result().ok is not True:
                    raise RuntimeError('STRICT switch was not confirmed')
            except Exception as exc:
                self._admittance_recovery_requested = False
                self._admittance_owner = 'uncertain'
                self._admittance_phase = 'blocked'
                self._admittance_reason = f'Controller ownership is uncertain ({exc}); Stop remains available.'
                self._publish_target_status_locked('blocked', self._admittance_reason)
                return
            self._admittance_owner = 'direct' if destination == 'direct' else 'position'
            self._admittance_controllers = None
            self._admittance_list_generation += 1
            self._admittance_list_pending = False
            self._admittance_list_requested = 0.0
            if destination != 'direct':
                self._admittance_target_active = False
                self._admittance_stopping = False
                self.target_active = False
                self._admittance_phase = 'idle'
                self._publish_target_status_locked('stopped',
                    'Measured hold settled; position-controller restoration confirmed.')
                return
            if self._arm_action_active:
                self._admittance_owner = 'uncertain'
                self._admittance_phase = 'blocked'
                self._admittance_reason = 'An arm action raced the ownership switch; inspect ownership and use Stop.'
                self._publish_target_status_locked('blocked', self._admittance_reason)
                return
            ready, reason = self._admittance_sample_ready_locked(allow_saturation=True)
            exclusive, exclusive_reason = self._admittance_exclusive_locked()
            if not ready or not exclusive:
                self._admittance_phase = 'blocked'
                self._admittance_reason = reason or exclusive_reason
                self._publish_target_status_locked('blocked', self._admittance_reason)
                return
            # First direct command only after ACK: a fresh measured pose,
            # never the possibly distant final target.
            pose = tuple(self._admittance_sample[2:8])
            self._admittance_core.reset(pose)
            self._admittance_last_command = pose
            self._admittance_last_velocity = [0.0] * 6
            self._admittance_direct_initialized = True
            self._admittance_phase = 'stopping' if self._admittance_stop_requested else 'running'
            if self._admittance_stop_requested:
                self._admittance_stop_pose = pose
            self._publish_admittance_direct_locked(pose, self.nominal_dt)
            self._publish_target_status_locked(
                'stopping' if self._admittance_stop_requested else 'running',
                'Direct position ownership confirmed; measured-pose reference initialized.')

    def _start_admittance_target_locked(self, feedback):
        ready, reason = self._admittance_ready_locked()
        if ready and self._arm_action_active:
            ready, reason = False, 'Wait for the existing arm trajectory to finish.'
        if not ready:
            self._publish_target_status_locked('blocked', reason)
            return False
        self._admittance_core.reset(feedback)
        self._admittance_last_command = tuple(feedback)
        self._admittance_last_velocity = [0.0] * 6
        self._admittance_direct_initialized = False
        self._admittance_result = None
        self._admittance_stopping = False
        self._admittance_stop_requested = False
        self._admittance_recovery_requested = False
        self._admittance_stop_pose = None
        self._admittance_stop_settled_since = None
        self._admittance_target_active = True
        self.target_active = True
        self._target_action_succeeded = False
        self.control_velocity = np.zeros(6)
        self.last_control_cmd = 0.0
        self._publish_target_status_locked('requesting', 'Requesting exclusive direct-position ownership.')
        self._request_admittance_switch_locked('direct')
        return True

    def _publish_admittance_direct_locked(self, command, dt):
        if self._admittance_owner != 'direct' or self._admittance_switch_pending:
            return
        previous = self._admittance_last_command
        velocity = [(a - b) / dt for a, b in zip(command, previous)]
        acceleration = [(a - b) / dt for a, b in zip(velocity, self._admittance_last_velocity)]
        self._admittance_direct_pub.publish(Float64MultiArray(data=list(command)))
        reference = JointTrajectoryControllerState()
        reference.header.stamp = self.get_clock().now().to_msg()
        reference.joint_names = list(self.joint_names)
        reference.reference.positions = list(command)
        reference.reference.velocities = velocity
        reference.reference.accelerations = acceleration
        self._admittance_reference_pub.publish(reference)
        self._admittance_last_command = tuple(command)
        self._admittance_last_velocity = velocity

    def _admittance_collision_free_locked(self, command):
        if self.ik is None or not self.ik_self_collision:
            return True
        from .target_planner import path_is_clear
        try:
            return path_is_clear(self.ik, np.asarray(self._admittance_last_command),
                                 np.asarray(command), self.ik_collision_radius)
        except Exception:
            return False

    def _tick_admittance_locked(self, now, dt):
        if not self._admittance_target_active:
            return
        if self._admittance_switch_pending:
            if now - self._admittance_switch_requested > 3.0:
                # A timeout is not proof that resources stayed unchanged. Keep
                # the pending callback alive and the mode guard reserved.
                self._admittance_owner = 'uncertain'
                self._admittance_phase = 'blocked'
                self._publish_target_status_locked('blocked',
                    'Controller-switch ACK is overdue; ownership is uncertain. '
                    'Support the arm and stop the stack if the response does not return.')
            return
        if self._admittance_owner != 'direct':
            if self._admittance_stop_requested:
                self._poll_admittance_controllers_locked(now)
            self._publish_target_status_locked('blocked', self._admittance_reason or 'Position ownership is uncertain.')
            return
        if self._admittance_stopping:
            self._tick_admittance_stop_locked(now, dt, True)
            return
        if not self._admittance_direct_initialized:
            self._publish_target_status_locked('blocked',
                'Direct ownership was acknowledged without a usable initial sample; use Stop to initialize a fresh hold.')
            return
        ready, reason = self._admittance_ready_locked()
        sample = self._admittance_sample
        old_core = copy.deepcopy(self._admittance_core)
        result = self._admittance_core.step(self.target_command_joints,
            sample[2:8] if sample else [0.0] * 6,
            sample[8:14] if sample else [0.0] * 6,
            sample[14:20] if sample else [0.0] * 6,
            sample[20:26] if sample else [0.0] * 6, dt, valid=ready)
        if result.valid and not self._admittance_collision_free_locked(result.command):
            self._admittance_core = old_core
            result = old_core.step(self.target_command_joints, sample[2:8], sample[8:14],
                                   sample[14:20], sample[20:26], dt, valid=False)
            reason = 'Admittance command blocked by the self-collision boundary.'
        self._admittance_result = result
        if result.valid:
            self._publish_admittance_direct_locked(result.command, dt)
            if any(result.contact):
                state, message = 'yielding', 'External torque estimated; original target retained.'
            elif max(abs(v) for v in result.offset) > 0.002:
                state, message = 'returning', 'Contact released; returning gradually to the retained target.'
            elif max(abs(a - b) for a, b in zip(self.target_command_joints, sample[2:8])) < 0.03:
                state, message = 'reached', 'Measured target reached; compliant holding remains active.'
            else:
                state, message = 'running', 'Tracking the retained target with bounded motion.'
        else:
            state, message = 'blocked', reason or result.reason
            # Forward position control holds its previous command on missing
            # data. No extrapolated trajectory remains in flight.
            self._admittance_last_velocity = [0.0] * 6
        self._admittance_reason = '' if result.valid else message
        if now - self.last_target_status_publish >= 0.2:
            self._publish_target_status_locked(state, message)

    def _request_admittance_stop_locked(self, request_id):
        self.target_request_id = request_id
        self._admittance_stopping = True
        self._admittance_stop_requested = True
        self._admittance_recovery_requested = True
        if not self._admittance_switch_pending:
            self._admittance_phase = 'stopping'
        self._admittance_stop_started = time.monotonic()
        self._admittance_stop_settled_since = None
        self.control_velocity = np.zeros(6)
        self.last_control_cmd = 0.0
        ready, _ = self._admittance_sample_ready_locked(allow_saturation=True)
        self._admittance_stop_pose = tuple(self._admittance_sample[2:8]) if ready else None
        self._publish_target_status_locked('stopping',
            'Original-target return disabled; preparing a bounded measured-position hold.')
        # Do not discard a pending switch callback: it may already have changed
        # hardware resource ownership before the response reaches this process.
        if self._admittance_switch_pending:
            if time.monotonic() - self._admittance_switch_requested > 3.0:
                self._publish_target_status_locked('blocked',
                    'Stop cannot resolve an unknown pending controller switch. '
                    'Support the arm and stop the stack if the response does not return.')
            return
        if self._admittance_owner == 'uncertain':
            self._poll_admittance_controllers_locked(time.monotonic())
            self._publish_target_status_locked('blocked', 'Stop is pending; controller ownership remains uncertain.')

    def _tick_admittance_stop_locked(self, now, dt, _valid):
        ready, reason = self._admittance_sample_ready_locked(allow_saturation=True)
        if ready:
            ready, reason = self._admittance_resources_ready_locked()
        if ready:
            ready, reason = self._admittance_exclusive_locked()
        if (not ready or not math.isfinite(dt) or not 0.0 < dt <= self._admittance_config.max_dt):
            self._admittance_last_velocity = [0.0] * 6
            self._admittance_stop_settled_since = None
            self._publish_target_status_locked('blocked', reason or 'Invalid Stop control interval.')
            return
        sample = self._admittance_sample
        if self._admittance_stop_pose is None:
            self._admittance_stop_pose = tuple(sample[2:8])
        if not self._admittance_direct_initialized:
            # Ownership was acknowledged earlier, but stale data prevented its
            # first position packet. Only explicit Stop may initialize it now.
            self._admittance_last_command = tuple(sample[2:8])
            self._admittance_last_velocity = [0.0] * 6
            self._admittance_direct_initialized = True
        command = []
        cfg = self._admittance_config
        for i in range(6):
            value, _ = self._admittance_core._govern(
                self._admittance_last_command[i], self._admittance_last_velocity[i],
                self._admittance_stop_pose[i], cfg.max_velocity[i], cfg.max_acceleration[i],
                dt, cfg.lower[i], cfg.upper[i])
            command.append(value)
        if not self._admittance_collision_free_locked(command):
            self._publish_target_status_locked('blocked', 'Measured Stop hold path is not collision-clear.')
            self._admittance_stop_settled_since = None
            return
        self._publish_admittance_direct_locked(command, dt)
        settled = (max(abs(v) for v in sample[8:14]) < 0.02
                   and max(abs(v) for v in self._admittance_last_velocity) < 0.005
                   and max(abs(a - b) for a, b in zip(command, self._admittance_stop_pose)) < 0.002
                   and max(abs(a - b) for a, b in zip(sample[2:8], self._admittance_stop_pose)) < 0.015)
        if settled:
            if self._admittance_stop_settled_since is None:
                self._admittance_stop_settled_since = now
            elif now - self._admittance_stop_settled_since >= 0.3:
                self._publish_target_status_locked('stopping', 'Measured hold settled; restoring the position controller.')
                self._request_admittance_switch_locked('position')
        else:
            self._admittance_stop_settled_since = None
        if now - self._admittance_stop_started > 3.0 and not self._admittance_switch_pending:
            self._publish_target_status_locked('blocked', 'Stop hold has not settled; original-target return stays disabled.')

    def _admittance_preempted_locked(self):
        if not self._admittance_target_active:
            return
        self._admittance_stop_requested = True
        self._admittance_recovery_requested = False
        self._admittance_stopping = True
        self._admittance_stop_started = time.monotonic()
        self._admittance_stop_settled_since = None
        self._admittance_owner = 'uncertain'
        self._admittance_phase = 'blocked'
        self._admittance_reason = 'Another arm action was observed; position ownership must be checked before Stop.'
        self._publish_target_status_locked('blocked', self._admittance_reason)

    def _admittance_status_locked(self):
        ready, reason = self._admittance_ready_locked()
        result = self._admittance_result
        valid_force = (result is not None and result.valid and ready
                       and not self._admittance_stopping and self._admittance_phase == 'running')
        return {'target_available': ready, 'target_block_reason': reason,
                'control_strategy': 'admittance' if self._admittance_enabled else 'position',
                'external_torque_nm': list(result.external_torque) if valid_force else None,
                'admittance_owner': self._admittance_owner,
                'admittance_phase': self._admittance_phase}
