"""YOLO -> calibrated Boxer3D centre -> MoveIt pregrasp, insertion, close.

Independent of the DD-GNG graph. Reuses the existing coordinator's tested
controller transport, measured-action verification and additive-channel guards;
its graph constructor, subscriptions, planner and execution worker are not used.
"""
import copy
from collections import deque
from dataclasses import dataclass
import json
import math
import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from control_msgs.action import FollowJointTrajectory, GripperCommand
from controller_manager_msgs.srv import ListControllers
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Pose, Point
from moveit_msgs.msg import DisplayTrajectory
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from .graph_pick_node import GraphPickNode, ordered_joint_positions
from .boxer_pick_planning import BoxObstacle, PickupPlanner


def parse_boxes(payload, *, now_ns, calibration_sha256, camera_serial, max_age=30.0):
    """Use source acquisition age, never the completion/publication time."""
    if not isinstance(payload, dict) or payload.get('schema') != 'om6dof.boxer3d_detections.v1':
        raise ValueError('Format deteksi Boxer3D belum tersedia')
    if payload.get('frame_id') != 'world':
        raise ValueError('Deteksi Boxer3D belum dalam koordinat world robot')
    if (payload.get('calibration_verified') is not True or not calibration_sha256 or
            payload.get('calibration_sha256') != calibration_sha256 or
            str(payload.get('camera_serial')) != str(camera_serial)):
        raise ValueError('Kalibrasi kamera pada deteksi tidak cocok dengan launch pickup')
    age = (now_ns - int(payload.get('source_stamp_ns', 0))) * 1e-9
    if not math.isfinite(age) or age < -0.1 or age > max_age:
        raise ValueError('Gambar sumber Boxer3D kedaluwarsa; tunggu hasil baru')
    boxes = []
    seen = set()
    for item in payload.get('boxes', []):
        center = tuple(float(v) for v in item['center'])
        size = tuple(float(v) for v in item['size'])
        quat = tuple(float(v) for v in item['quaternion_xyzw'])
        identity = str(item['id'])
        if (len(center) != 3 or len(size) != 3 or len(quat) != 4 or identity in seen
                or not all(math.isfinite(v) for v in center + size + quat)
                or any(v <= 0.0 or v > 3.0 for v in size)
                or abs(sum(v*v for v in quat) - 1.0) > 0.002):
            raise ValueError('Geometri kotak Boxer3D tidak valid')
        seen.add(identity)
        score = float(item['score'])
        if not math.isfinite(score) or not 0.0 <= score <= 1.0:
            raise ValueError('Confidence Boxer3D tidak valid')
        boxes.append(dict(item, id=identity, center=center, size=size, quaternion_xyzw=quat))
    return boxes


def choose_box(boxes, label, center=None, max_shift=.02):
    candidates = [b for b in boxes if b['label'] == label and b['score'] >= .3
                  and int(b['inside_point_count']) >= 30]
    if center is None:
        if not candidates:
            raise ValueError(f'Belum ada kotak {label} dengan dukungan depth yang cukup')
        return max(candidates, key=lambda b: b['score'])
    matches = [b for b in candidates if math.dist(b['center'], center) <= max_shift]
    if len(matches) != 1:
        raise ValueError('Target bergeser, hilang, atau ambigu; buat Preview baru')
    return matches[0]


def collision_boxes(boxes):
    result = []
    for box in boxes:
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = box['center']
        (pose.orientation.x, pose.orientation.y,
         pose.orientation.z, pose.orientation.w) = box['quaternion_xyzw']
        result.append(BoxObstacle(box['id'], pose, box['size']))
    return result


@dataclass
class FrozenBoxPick:
    created_monotonic: float
    center: tuple
    target_class: str
    motion: object
    bus_read_failures: int
    bus_write_failures: int


class BoxerPickNode(GraphPickNode):
    def __init__(self):
        Node.__init__(self, 'boxer_center_pick')
        defaults = {
            'robot_description': '', 'target_class': 'bottle', 'execution_enabled': False,
            'camera_serial': '243222076197', 'camera_calibration_sha256': '',
            'detections_topic': '/boxer3d/detections', 'max_detection_age_sec': 30.0,
            'max_plan_hold_sec': 15.0, 'max_target_shift_m': .02,
            'pregrasp_distance': .075, 'gripper_open': .019, 'gripper_close': -.010,
            'arm_action_name': '/arm_controller/follow_joint_trajectory',
            'gripper_action_name': '/gripper_controller/gripper_cmd',
            'gripper_joint_name': 'gripper_left_joint', 'gripper_max_effort': 0.0,
            'gripper_position_tolerance': .0015, 'gripper_result_match_tolerance': .002,
            'gripper_contact_position_margin': .002, 'gripper_stationary_time_sec': .25,
            'gripper_stationary_tolerance': .0005, 'gripper_verification_timeout_sec': 3.0,
            'action_timeout_sec': 120.0, 'max_joint_state_age_sec': .5,
            'max_start_joint_error_rad': .02, 'arm_endpoint_verification_timeout_sec': 2.0,
            'arm_controller_name': 'arm_controller', 'offset_controller_name': 'forward_offset_controller',
            'controller_state_timeout_sec': 1.5,
            'dynamixel_health_topic': '/dynamixel_hardware_interface/health',
            'dynamixel_health_status_name': 'dynamixel_hardware_interface/BusHealth',
            'dynamixel_health_timeout_sec': .3,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        for name, default in defaults.items():
            if isinstance(default, float):
                value = float(self.get_parameter(name).value)
                if not math.isfinite(value) or (default > 0 and value <= 0):
                    raise ValueError(f'{name} harus positif dan finite')
        if self.param('gripper_open') <= self.param('gripper_close'):
            raise ValueError('gripper_open harus lebih besar dari gripper_close')
        self.execution_enabled = bool(self.param('execution_enabled'))
        self.joint_names = [f'joint{i}' for i in range(1, 7)]
        self.gripper_joint_name = self.param('gripper_joint_name')
        self.world_frame = 'world'
        self._lock = threading.RLock()
        self._cancel = threading.Event()
        self._joint_state = None
        self._joint_state_time = 0.0
        self._gripper_measurements = deque(maxlen=512)
        self._controller_snapshot = None
        self._controller_snapshot_time = 0.0
        self._controller_query_future = None
        self._controller_query_started = 0.0
        self._bus_health = None
        self._bus_health_time = 0.0
        self._busy = self._holding_object = self._faulted = False
        self._fault_reason = ''
        self._arm_goal_handle = self._gripper_goal_handle = None
        self._last_gripper_result_state = 'not_commanded'
        self._frozen = None
        self._detections = None
        self._phase = ''
        self._state = 'waiting'
        self._message = 'Menunggu deteksi YOLO dan Boxer3D'
        cb = ReentrantCallbackGroup()
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(String, self.param('detections_topic'), self._on_boxes, latched,
                                 callback_group=cb)
        self.create_subscription(JointState, '/joint_states', self._on_joint_state, 5, callback_group=cb)
        self.create_subscription(DiagnosticArray, self.param('dynamixel_health_topic'),
                                 self._on_bus_health, latched, callback_group=cb)
        self._controller_client = self.create_client(ListControllers, '/controller_manager/list_controllers',
                                                     callback_group=cb)
        self._arm_client = ActionClient(self, FollowJointTrajectory, self.param('arm_action_name'), callback_group=cb)
        self._gripper_client = ActionClient(self, GripperCommand, self.param('gripper_action_name'), callback_group=cb)
        self._status_pub = self.create_publisher(String, '/boxer_pick/status', latched)
        self._preview_pub = self.create_publisher(DisplayTrajectory, '/boxer_pick/display_planned_path', latched)
        self._marker_pub = self.create_publisher(MarkerArray, '/boxer_pick/markers', latched)
        self.planner = PickupPlanner(self, namespace='/boxer_pick', robot_description=self.param('robot_description'))
        for endpoint, callback in [('preview', self._preview), ('execute', self._execute), ('cancel', self._on_cancel)]:
            self.create_service(Trigger, '/boxer_pick/' + endpoint, callback, callback_group=cb)
        self.create_timer(.5, self._poll_controllers, callback_group=cb)
        self.create_timer(.2, self._publish_status, callback_group=cb)
        self.create_timer(.1, self._monitor_execution_interlocks, callback_group=cb)

    def param(self, name):
        return self.get_parameter(name).value

    def _on_boxes(self, message):
        try:
            value = json.loads(message.data)
        except ValueError:
            value = None
        with self._lock:
            self._detections = value

    def _boxes(self):
        with self._lock:
            value = copy.deepcopy(self._detections)
        return parse_boxes(value, now_ns=self.get_clock().now().nanoseconds,
                           calibration_sha256=self.param('camera_calibration_sha256'),
                           camera_serial=self.param('camera_serial'), max_age=self.param('max_detection_age_sec'))

    def _joints(self):
        with self._lock:
            if self._joint_state is None or time.monotonic() - self._joint_state_time > self.param('max_joint_state_age_sec'):
                raise ValueError('Encoder robot belum tersedia atau kedaluwarsa')
            state = copy.deepcopy(self._joint_state)
        if ordered_joint_positions(state, self.joint_names + [self.gripper_joint_name]) is None:
            raise ValueError('Encoder arm/gripper tidak lengkap')
        return state

    def _segment_start_rejection(self, trajectory):
        # MoveIt may return another valid joint-name ordering. The shared
        # transport compares canonical arm order, so normalize its check only.
        if (len(trajectory.joint_names) != len(self.joint_names) or
                set(trajectory.joint_names) != set(self.joint_names) or
                not trajectory.points or
                len(trajectory.points[0].positions) != len(self.joint_names)):
            return 'arm_segment_start_invalid'
        first = dict(zip(trajectory.joint_names, trajectory.points[0].positions))
        normalized = copy.deepcopy(trajectory)
        normalized.joint_names = list(self.joint_names)
        normalized.points = normalized.points[:1]
        normalized.points[0].positions = [first[name] for name in self.joint_names]
        return GraphPickNode._segment_start_rejection(self, normalized)

    def _publish_status(self):
        with self._lock:
            frozen = self._frozen
            ready = frozen is not None and time.monotonic() - frozen.created_monotonic <= self.param('max_plan_hold_sec')
            payload = dict(state=self._state, message=self._message, busy=self._busy,
                           plan_ready=ready, target_class=self.param('target_class'),
                           execution_enabled=self.execution_enabled,
                           holding_object=self._holding_object, motion_faulted=self._faulted,
                           executable=ready and self.execution_enabled and not self._busy and not self._faulted)
            if frozen:
                payload['target_center'] = list(frozen.center)
        self._status_pub.publish(String(data=json.dumps(payload)))

    def _stage(self, state, message):
        self._state, self._message = state, message
        self._publish_status()

    def _preview(self, request, response):
        with self._lock:
            if self._busy:
                response.success, response.message = False, 'Proses masih berjalan'
                return response
            self._busy = True
            self._phase = 'planning'
            self._frozen = None
            self._cancel.clear()
        threading.Thread(target=self._preview_worker, daemon=True).start()
        response.success, response.message = True, 'Membuat jalur sampai pusat kotak Boxer3D'
        return response

    def _preview_worker(self):
        try:
            self._stage('planning', 'Merencanakan pra-jepit dan pendekatan lurus ke pusat kotak')
            boxes = self._boxes()
            target = choose_box(boxes, self.param('target_class'))
            motion = self.planner.plan(self._joints(), target['center'], collision_boxes(boxes),
                                       selected_id=target['id'], pregrasp_distance=self.param('pregrasp_distance'),
                                       gripper_open=self.param('gripper_open'), gripper_close=self.param('gripper_close'))
            if self._cancel.is_set():
                raise ValueError('Preview dibatalkan')
            choose_box(self._boxes(), target['label'], target['center'], self.param('max_target_shift_m'))
            counts = self._bus_failure_counts()
            self._frozen = FrozenBoxPick(time.monotonic(), target['center'], target['label'], motion, *counts)
            self._preview_pub.publish(motion.display)
            marker = Marker()
            marker.header.frame_id = 'world'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns, marker.id, marker.type, marker.action = 'boxer_pick_center', 0, Marker.SPHERE, Marker.ADD
            marker.pose.position = Point(x=target['center'][0], y=target['center'][1], z=target['center'][2])
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = marker.scale.z = .025
            marker.color.r, marker.color.g, marker.color.a = 1.0, .5, 1.0
            self._marker_pub.publish(MarkerArray(markers=[marker]))
            self._stage('preview_ready', 'Preview siap: buka → pra-jepit → pusat kotak → jepit')
        except Exception as error:
            self._stage('planning_failed', str(error))
            self.get_logger().error(f'Boxer pickup preview: {error}')
        finally:
            self._busy = False
            self._phase = ''
            self._publish_status()

    def _execute(self, request, response):
        with self._lock:
            if self._busy or self._frozen is None or self._faulted or not self.execution_enabled:
                response.success, response.message = False, 'Buat Preview baru dan aktifkan execution_enabled saat launch'
                return response
            if time.monotonic() - self._frozen.created_monotonic > self.param('max_plan_hold_sec'):
                response.success, response.message = False, 'Preview kedaluwarsa; buat Preview baru'
                return response
            plan = self._frozen
            self._busy, self._holding_object = True, False
            self._phase = 'executing'
            self._cancel.clear()
        threading.Thread(target=self._pickup_worker, args=(plan,), daemon=True).start()
        response.success, response.message = True, 'Pickup dimulai'
        return response

    def _refresh_scene(self, plan):
        if not self._execution_interlocks_ready(plan):
            raise ValueError(self._message)
        boxes = self._boxes()
        target = choose_box(boxes, plan.target_class, plan.center, self.param('max_target_shift_m'))
        self.planner.set_scene(collision_boxes(boxes), selected_id=target['id'])
        if self._cancel.is_set():
            raise ValueError('Pickup dibatalkan')

    def _pickup_worker(self, plan):
        try:
            self._stage('opening', 'Membuka gripper')
            self._refresh_scene(plan)
            rejection = self._segment_start_rejection(plan.motion.pregrasp)
            if rejection:
                raise ValueError(rejection)
            measured = self._joints()
            opening = self.param('gripper_open')
            self.planner.validate_gripper(measured, ordered_joint_positions(measured, [self.gripper_joint_name])[0], opening)
            if not self._send_gripper(opening, 'open gripper'):
                raise ValueError(self._message)
            for state, trajectory, label in [('pregrasp', plan.motion.pregrasp, 'menuju pra-jepit'),
                                             ('insertion', plan.motion.insertion, 'mendekati pusat kotak')]:
                self._stage(state, label)
                self._refresh_scene(plan)
                self.planner.validate_segment(trajectory, self._joints(), opening)
                if not self._execution_interlocks_ready(plan) or not self._send_arm(trajectory, label):
                    raise ValueError(self._message)
            self._stage('closing', 'Menjepit bagian tengah benda')
            self._refresh_scene(plan)
            terminal = copy.deepcopy(plan.motion.insertion)
            terminal.points = terminal.points[-1:]
            self.planner.validate_segment(terminal, self._joints(), opening, check_closure=True,
                                          gripper_close=self.param('gripper_close'))
            if not self._execution_interlocks_ready(plan) or not self._send_gripper(self.param('gripper_close'), 'close gripper'):
                raise ValueError(self._message)
            self._holding_object = self._last_gripper_result_state == 'contact_detected'
            self._stage('done', 'Kontak jepitan terdeteksi; benda belum diangkat' if self._holding_object
                        else 'Gripper tertutup; benda terpegang belum terkonfirmasi')
        except Exception as error:
            self._stage('stopped', str(error))
            self.get_logger().error(f'Boxer pickup stopped: {error}')
        finally:
            self._busy = False
            self._phase = ''
            self._frozen = None
            self._publish_status()

    def _monitor_execution_interlocks(self):
        if self._phase == 'executing':
            super()._monitor_execution_interlocks()


def main(args=None):
    rclpy.init(args=args)
    node = BoxerPickNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node._cancel.set()
        for handle in (node._arm_goal_handle, node._gripper_goal_handle):
            if handle is not None:
                handle.cancel_goal_async()
        executor.shutdown(timeout_sec=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
