"""Leased web targets for the effort-only impedance plugin; no mode switching.

The bridge never sends trajectories, offsets, lifecycle or torque-enable requests.
Only the browser which started a target may renew its short lease. A lost lease,
controller fault/restart or stale feedback retracts the command stream permanently
for that request. Recovery always needs another explicit Run target.
"""
import json
import math
import time
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from controller_manager_msgs.srv import ListControllers
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String

from .control_math import (cartesian_rpy_to_tip_rotation,
                           tip_rotation_to_cartesian_rpy, rotation_from_zyx,
                           rotation_to_zyx)
from .ik_solver import IKSolver
from .impedance_faults import fault_message, limit_description
from .target_planner import plan_pose, path_is_clear, pose_error

CONTROLLER = 'om6dof_position_impedance_controller'
COMMAND_TOPIC = '/om6dof/impedance_target/command'
STATUS_TOPIC = '/om6dof/impedance_target/status'
JOINTS = [f'joint{i}' for i in range(1, 7)]
LEASE_SECONDS = 1.0
FEEDBACK_SECONDS = 0.2
MANAGER_SECONDS = 0.6


def exclusive_controller(controllers):
    """Require the expected plugin, all six effort claims, no other arm owner."""
    expected = {f'{j}/effort' for j in JOINTS}
    found = False
    for item in controllers:
        if item.state != 'active':
            continue
        claims = set(item.claimed_interfaces)
        if item.name == CONTROLLER:
            found = (item.type == 'om6dof_controllers/PositionImpedanceController'
                     and claims == expected)
        elif any(f'{j}/{kind}' in claims for j in JOINTS
                 for kind in ('effort', 'position', 'velocity')):
            return False
    return found


class TargetSession:
    """ROS-independent lease state, with monotonic and source-time checks."""

    def __init__(self):
        self.request_id = ''
        self.goal = None
        self.deadline = 0.0
        self.last_stamp = 0
        self.state = 'idle'
        self.message = 'No impedance target yet.'

    @property
    def active(self):
        return self.goal is not None

    def check_stamp(self, request, ros_ns):
        stamp = request.get('stamp_ns')
        if (not isinstance(stamp, int) or isinstance(stamp, bool)
                or stamp <= self.last_stamp or not 0 <= ros_ns - stamp <= 500_000_000):
            raise ValueError('stale, future or replayed target request')
        self.last_stamp = stamp
        return stamp

    def begin(self, request_id, goal, now, source_age=0.0):
        if self.active:
            raise ValueError('another target is active; Stop target first')
        if not isinstance(request_id, str) or not 1 <= len(request_id) <= 80:
            raise ValueError('missing request id')
        values = np.asarray(goal, dtype=float)
        if values.shape != (6,) or not np.isfinite(values).all():
            raise ValueError('target needs six finite joints')
        self.request_id = request_id
        self.goal = values.copy()
        self.deadline = now + LEASE_SECONDS - source_age
        self.state, self.message = 'running', 'Position impedance target running.'

    def renew(self, request_id, now, source_age=0.0):
        if not self.active or request_id != self.request_id:
            raise ValueError('no matching active target')
        if now >= self.deadline:
            self.stop('timeout', 'Browser lease expired; Run target again.')
            raise ValueError(self.message)
        self.deadline = now + LEASE_SECONDS - source_age

    def stop(self, state='stopped', message='Target stopped; healthy gravity + bounded damping after watchdog.'):
        self.goal = None
        self.state, self.message = state, message

    def tick(self, now, ready):
        if self.active and (not ready or now >= self.deadline):
            self.stop('timeout' if ready else 'blocked',
                      'Browser lease expired.' if ready else 'Impedance feedback/ownership unavailable.')
        return self.goal


class ImpedanceTargetNode(Node):
    def __init__(self):
        super().__init__('om6dof_impedance_target')
        self.declare_parameter('controller_manager', '/controller_manager')
        self.session = TargetSession()
        self.ik = None
        self.urdf_xml = ''
        self.model_changed = False
        self.lower = self.upper = None
        self.q = None
        self.state_at = self.manager_at = 0.0
        self.controller_ok = False
        self.controller_status = -1
        self.fault = -1
        self.limit_detail, self.limit_at = '', 0.
        self.query = None
        self.query_at = 0.0
        self.command_started = 0.0
        self.last_command_stamp = 0
        self.mode = 'JOINT'
        self.values = None
        self.target_pose = None
        self.origin = (0.012, 0.0)
        self.radius = 0.025
        self.pub = self.create_publisher(JointState, f'/{CONTROLLER}/target', 1)
        status_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, STATUS_TOPIC, status_qos)
        self.create_subscription(String, '/robot_description', self.on_model, status_qos)
        self.create_subscription(Float64MultiArray, f'/{CONTROLLER}/state', self.on_state, 1)
        self.create_subscription(Float64MultiArray, f'/{CONTROLLER}/limit_event', self.on_limit, 1)
        self.create_subscription(String, COMMAND_TOPIC, self.on_command, 1)
        self.manager = self.create_client(ListControllers,
            str(self.get_parameter('controller_manager').value).rstrip('/') + '/list_controllers')
        self.create_timer(0.2, self.query_manager)
        self.create_timer(0.05, self.tick)  # fresh targets at 20 Hz; effort loop stays 100 Hz
        self.create_timer(0.1, self.publish_status)

    def on_model(self, msg):
        if self.model_changed:
            return
        if msg.data == self.urdf_xml:
            return
        if self.urdf_xml:
            self.ik = None
            self.model_changed = True
            self.session.stop('blocked', 'Live URDF changed; supported reload of controller and bridge required.')
            return
        self.session.stop('blocked', 'Robot model changed; waiting for a new target.')
        self.ik = None
        try:
            root = ET.fromstring(msg.data)
            joints = {j.attrib['name']: j for j in root.findall('joint')}
            for name in JOINTS:
                if joints[name].attrib.get('type') != 'revolute' or joints[name].find('mimic') is not None:
                    raise ValueError('expected six bounded independent revolute joints')
            ik = IKSolver(base_link='world', tip_link='end_effector_link', urdf_xml=msg.data)
            chain_names = [ik.chain.getSegment(i).getJoint().getName()
                           for i in range(ik.chain.getNrOfSegments())
                           if ik.chain.getSegment(i).getJoint().getName() in JOINTS]
            if chain_names != JOINTS:
                raise ValueError('unexpected joint order')
            lower, upper = ik.q_min + 0.05, ik.q_max - 0.05
            if not np.isfinite([lower, upper]).all() or np.any(lower >= upper):
                raise ValueError('invalid URDF limits')
            self.ik, self.lower, self.upper = ik, lower, upper
            self.urdf_xml = msg.data
        except Exception as exc:
            self.session.stop('blocked', f'Live URDF unavailable: {exc}')

    def on_limit(self, msg):
        self.limit_detail = limit_description(msg.data, JOINTS)
        self.limit_at = time.monotonic()

    def on_state(self, msg):
        now = time.monotonic()
        # No timestamp is present in this plugin's telemetry. A depth-one queue,
        # local receipt deadline and independent manager query fail closed.
        if len(msg.data) != 44 or not all(math.isfinite(v) for v in msg.data):
            self.state_at = 0.0
            self.session.stop('blocked', 'Malformed impedance telemetry.')
            return
        if self.session.active and now - self.state_at > FEEDBACK_SECONDS:
            self.session.stop('blocked', 'Impedance telemetry gap; Run target again.')
        self.controller_status, self.fault = msg.data[:2]
        self.q = np.asarray(msg.data[2:8])
        self.state_at = now
        if self.fault or self.controller_status not in (0, 1, 2):
            detail = (getattr(self, 'limit_detail', '')
                      if 0 <= now - getattr(self, 'limit_at', 0.) < FEEDBACK_SECONDS else '')
            self.session.stop('blocked', fault_message(self.controller_status, self.fault, detail))
        elif (self.session.active and now - self.command_started > 0.3
              and self.controller_status != 1):
            self.session.stop('blocked', 'Controller stopped tracking; Run target again.')

    def query_manager(self):
        now = time.monotonic()
        if self.query is not None:
            if now - self.query_at <= MANAGER_SECONDS:
                return
            self.query.cancel()
            self.query = None
            self.controller_ok = False
        if self.manager.service_is_ready():
            self.query_at = now
            self.query = self.manager.call_async(ListControllers.Request())
            self.query.add_done_callback(self.on_manager)

    def on_manager(self, future):
        if future is not self.query:
            return
        self.query = None
        try:
            now = time.monotonic()
            if self.session.active and now - self.manager_at > MANAGER_SECONDS:
                self.session.stop('blocked', 'Controller manager gap; Run target again.')
            self.controller_ok = exclusive_controller(future.result().controller)
            self.manager_at = now
        except Exception:
            self.controller_ok = False
        if not self.controller_ok:
            self.session.stop('blocked', 'Impedance must be active and the only arm command owner.')

    def controller_ready(self):
        now = time.monotonic()
        return bool(self.ik is not None and self.q is not None
                    and self.controller_ok and now - self.manager_at < MANAGER_SECONDS
                    and now - self.state_at < FEEDBACK_SECONDS and self.fault == 0
                    and self.controller_status in (0, 1, 2))

    def ready(self):
        return bool(self.controller_ready()
                    and np.all(self.q >= self.lower) and np.all(self.q <= self.upper))

    def force_ready(self):
        # FORCE uses URDF bounds directly. The position target planner keeps
        # its separate 0.05 rad inward margin and must not gate wrench input.
        return bool(self.controller_ready()
                    and np.all(self.q >= self.ik.q_min) and np.all(self.q <= self.ik.q_max))

    def plan(self, mode, values):
        values = np.asarray(values, dtype=float)
        if values.shape != (6,) or not np.isfinite(values).all():
            raise ValueError('six finite SI values required')
        pose = None
        if mode == 'JOINT':
            goal = values
        else:
            if mode == 'CARTESIAN':
                xyz, rotation = values[:3], cartesian_rpy_to_tip_rotation(*values[3:])
            elif mode == 'CYLINDRICAL':
                r, theta, z, roll, pitch, yaw = values
                if r < 0.03:
                    raise ValueError('cylindrical radius is too small')
                xyz = np.array([self.origin[0] + r * math.cos(theta),
                                self.origin[1] + r * math.sin(theta), z])
                rotation = rotation_from_zyx(roll, pitch, yaw)
            else:
                raise ValueError('unsupported target coordinate mode')
            # Use the current pose as the secondary seed: this plans only, it
            # does not initiate a READY sequence or command any waypoint.
            plan = plan_pose(self.ik, self.q, xyz, rotation,
                             self.lower, self.upper, self.q, self.radius)
            if plan.approximate:
                raise ValueError('exact IK unavailable; approximate targets are not sent')
            goal, pose = plan.joints, (xyz, rotation)
        if np.any(goal < self.lower) or np.any(goal > self.upper):
            raise ValueError('target outside URDF soft limits')
        if not path_is_clear(self.ik, self.q, goal, self.radius):
            raise ValueError('sampled joint path fails self-collision check')
        return goal, pose

    def on_command(self, msg):
        try:
            request = json.loads(msg.data)
            if not isinstance(request, dict):
                raise ValueError('request must be an object')
            action = request.get('action')
            # Stop never needs a valid lease and can only remove authority.
            if action == 'stop':
                self.session.stop()
                return
            if action == 'move' and not self.session.active:
                # Correlate a rejection as well as an accepted request, so the
                # web process can revoke this request's lease promptly.
                self.session.request_id = str(request.get('request_id', ''))[:80]
            now, ros_ns = time.monotonic(), self.get_clock().now().nanoseconds
            stamp = self.session.check_stamp(request, ros_ns)
            source_age = (ros_ns - stamp) / 1e9
            if action == 'renew':
                self.session.renew(request.get('request_id'), now, source_age)
            elif action == 'move':
                if not self.ready():
                    raise ValueError('activate and verify Mode-0 impedance after TRANSITION → READY first')
                if self.session.active:
                    raise ValueError('Stop target before sending another target')
                mode = str(request.get('mode', '')).upper()
                values = request.get('values', [])
                goal, pose = self.plan(mode, values)
                # IK may take time: never send a solution based on expired state.
                if not self.ready() or time.monotonic() - now + source_age > 0.5:
                    raise ValueError('planning exceeded freshness deadline; retry a closer target')
                self.session.begin(request.get('request_id'), goal, now, source_age)
                self.mode, self.values, self.target_pose = mode, list(values), pose
                self.command_started = time.monotonic()
            else:
                raise ValueError('unknown action')
        except Exception as exc:
            # Rejection also revokes the previous command stream, never resumes.
            self.session.stop('rejected', str(exc))
        finally:
            self.publish_status()

    def tick(self):
        goal = self.session.tick(time.monotonic(), self.ready())
        if goal is None:
            return
        # An approximate geometric check, not a swept-volume/environment guarantee.
        if self.ik.self_collides(self.q, self.radius):
            self.session.stop('blocked', 'Measured pose fails self-collision check.')
            return
        stamp = self.get_clock().now()
        if stamp.nanoseconds <= self.last_command_stamp:
            self.session.stop('blocked', 'ROS clock did not advance.')
            return
        self.last_command_stamp = stamp.nanoseconds
        command = JointState()
        command.header.stamp = stamp.to_msg()
        command.name = JOINTS
        command.position = goal.tolist()
        self.pub.publish(command)

    def publish_status(self):
        current, errors = {}, {}
        ready = self.ready()
        if self.ik is not None and self.q is not None and time.monotonic() - self.state_at < FEEDBACK_SECONDS:
            try:
                xyz, rotation = self.ik.fk_pose(self.q)
                rpy = rotation_to_zyx(rotation)
                dx, dy = xyz[0] - self.origin[0], xyz[1] - self.origin[1]
                current = {'joint': self.q.tolist(),
                           'cartesian': xyz.tolist() + list(tip_rotation_to_cartesian_rpy(rotation)),
                           'cylindrical': [math.hypot(dx, dy), math.atan2(dy, dx), float(xyz[2])] + list(rpy)}
                if self.session.active:
                    errors['joint_max_deg'] = math.degrees(float(np.max(abs(self.session.goal - self.q))))
                    pose_reached = True
                    if self.target_pose is not None:
                        ep, er = pose_error(self.ik, self.q, *self.target_pose)
                        errors.update(position_mm=1000 * ep, orientation_deg=math.degrees(er))
                        pose_reached = ep <= 0.001 and er <= math.radians(0.5)
                    reached = errors['joint_max_deg'] <= 0.5 and pose_reached
                    self.session.state = 'holding' if reached else 'running'
                    self.session.message = ('Target reached; compliant hold while browser lease remains.' if reached
                                            else 'Moving toward final target within the gravity-relative torque budget.')
            except Exception:
                self.session.stop('blocked', 'FK feedback unavailable.')
                ready = False
        payload = dict(backend='position_impedance', ready=ready,
                       force_ready=self.force_ready() and bool(current),
                       stamp_ns=self.get_clock().now().nanoseconds,
                       state=self.session.state, active=self.session.active,
                       request_id=self.session.request_id, mode=self.mode,
                       goal=self.values, current=current, errors=errors,
                       message=self.session.message if ready else
                       'Impedance unavailable: needs active exclusive controller, live URDF and fresh fault-free telemetry. ' + self.session.message)
        self.status_pub.publish(String(data=json.dumps(payload, allow_nan=False)))


def main(args=None):
    rclpy.init(args=args)
    node = ImpedanceTargetNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop publishing; the C++ watchdog retracts target torque, retaining gravity + damping
        # only while its state and bus checks pass. No torque-off request here.
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
