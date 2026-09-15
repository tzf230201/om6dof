"""Planning-only MoveIt helpers for a horizontal grasp of a Boxer3D center.

There are deliberately no action clients in this module. Call blocking methods
from a worker while the node's executor services the reentrant service clients.
The physical V2 gripper approaches along local +Z and closes along local Y.
"""

from __future__ import annotations

import copy
import math
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from typing import Iterable, Sequence

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Pose, Quaternion
from moveit_msgs.msg import (
    CollisionObject, Constraints, DisplayTrajectory, OrientationConstraint,
    PositionConstraint, RobotState, RobotTrajectory,
)
from moveit_msgs.srv import (
    ApplyPlanningScene, GetCartesianPath, GetMotionPlan, GetPositionFK,
    GetStateValidity,
)
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


ARM_JOINTS = tuple(f'joint{i}' for i in range(1, 7))
GRIPPER_JOINTS = ('gripper_left_joint', 'gripper_right_joint')


class PlanningError(RuntimeError):
    """A reason the complete pickup cannot currently be planned/validated."""


@dataclass(frozen=True)
class BoxObstacle:
    id: str
    pose: Pose
    size: tuple[float, float, float]


@dataclass
class PickupPlan:
    pregrasp: JointTrajectory
    insertion: JointTrajectory
    start_state: RobotState
    pregrasp_pose: Pose
    grasp_pose: Pose
    center: tuple[float, float, float]
    display: DisplayTrajectory
    position_error: float


def _vector(values: Sequence[float], size: int, label: str) -> tuple:
    result = tuple(float(v) for v in values)
    if len(result) != size or not all(math.isfinite(v) for v in result):
        raise PlanningError(f'{label}_nonfinite_or_wrong_size')
    return result


def normalized_quaternion(q: Quaternion) -> Quaternion:
    values = _vector((q.x, q.y, q.z, q.w), 4, 'quaternion')
    norm = math.sqrt(sum(v * v for v in values))
    if norm < 1e-8:
        raise PlanningError('quaternion_zero')
    return Quaternion(**dict(zip(('x', 'y', 'z', 'w'), (v / norm for v in values))))


def _quaternion_from_axes(x_axis, y_axis, z_axis) -> Quaternion:
    """Convert columns of a proper orthogonal rotation to xyzw quaternion."""
    m = [[x_axis[i], y_axis[i], z_axis[i]] for i in range(3)]
    trace = sum(m[i][i] for i in range(3))
    if trace > 0.0:
        s = 2.0 * math.sqrt(trace + 1.0)
        q = Quaternion(x=(m[2][1] - m[1][2]) / s,
                       y=(m[0][2] - m[2][0]) / s,
                       z=(m[1][0] - m[0][1]) / s, w=0.25 * s)
    else:
        i = max(range(3), key=lambda j: m[j][j])
        j, k = (i + 1) % 3, (i + 2) % 3
        s = 2.0 * math.sqrt(max(0.0, 1.0 + m[i][i] - m[j][j] - m[k][k]))
        v = [0.0] * 4
        v[i] = 0.25 * s
        v[j] = (m[i][j] + m[j][i]) / s
        v[k] = (m[i][k] + m[k][i]) / s
        v[3] = (m[k][j] - m[j][k]) / s
        q = Quaternion(x=v[0], y=v[1], z=v[2], w=v[3])
    return normalized_quaternion(q)


def tool_forward(q: Quaternion) -> tuple[float, float, float]:
    q = normalized_quaternion(q)
    return (2.0 * (q.x * q.z + q.w * q.y),
            2.0 * (q.y * q.z - q.w * q.x),
            1.0 - 2.0 * (q.x * q.x + q.y * q.y))


def side_grasp_poses(center: Sequence[float], distance: float = 0.075,
                     yaw_offset: float = 0.0, roll_sign: int = 1,
                     base_xy: Sequence[float] = (0.0, 0.0)) -> tuple[Pose, Pose]:
    """Return pregrasp and pinch poses at equal height, aimed at box center.

    Local X is vertical; local Y (jaw closure) is horizontal. Roll sign -1
    rotates the symmetric gripper by pi around its forward axis. Both choices
    remain side grasps; neither silently falls back to a top-down grasp.
    """
    center = _vector(center, 3, 'center')
    base_xy = _vector(base_xy, 2, 'base_xy')
    distance, yaw_offset = _vector((distance, yaw_offset), 2, 'approach')
    if not 0.005 <= distance <= 0.20 or roll_sign not in (-1, 1):
        raise PlanningError('approach_parameters_invalid')
    dx, dy = center[0] - base_xy[0], center[1] - base_xy[1]
    if math.hypot(dx, dy) < 0.02:
        raise PlanningError('target_horizontal_direction_undefined')
    yaw = math.atan2(dy, dx) + yaw_offset
    z_axis = (math.cos(yaw), math.sin(yaw), 0.0)
    x_axis = (0.0, 0.0, float(roll_sign))
    y_axis = (z_axis[1] * roll_sign, -z_axis[0] * roll_sign, 0.0)
    grasp = Pose()
    grasp.position.x, grasp.position.y, grasp.position.z = center
    grasp.orientation = _quaternion_from_axes(x_axis, y_axis, z_axis)
    pregrasp = copy.deepcopy(grasp)
    pregrasp.position.x -= distance * z_axis[0]
    pregrasp.position.y -= distance * z_axis[1]
    return pregrasp, grasp


def joint_limits_from_urdf(description: str) -> dict:
    try:
        root = ET.fromstring(description)
        result = {}
        for joint in root.findall('joint'):
            limit = joint.find('limit')
            if limit is not None and 'lower' in limit.attrib and 'upper' in limit.attrib:
                result[joint.attrib['name']] = (
                    float(limit.attrib['lower']), float(limit.attrib['upper']))
        return result
    except (ET.ParseError, ValueError, KeyError) as exc:
        raise PlanningError('robot_description_invalid') from exc


def state_with_joints(start: JointState | RobotState, names: Iterable[str],
                      positions: Iterable[float]) -> RobotState:
    """Create an explicit frozen state without reusing stale velocity/stamp."""
    js = start.joint_state if isinstance(start, RobotState) else start
    if (len(js.name) != len(js.position) or len(set(js.name)) != len(js.name)
            or not all(math.isfinite(v) for v in js.position)):
        raise PlanningError('joint_state_invalid')
    values = dict(zip(js.name, js.position))
    names, positions = list(names), list(positions)
    if len(names) != len(positions) or not all(math.isfinite(v) for v in positions):
        raise PlanningError('joint_update_invalid')
    values.update(zip(names, positions))
    state = RobotState()
    state.is_diff = False
    state.joint_state.name = list(values)
    state.joint_state.position = [float(v) for v in values.values()]
    return state


def trajectory_endpoint(start: JointState | RobotState,
                        trajectory: JointTrajectory) -> RobotState:
    if not trajectory.points:
        raise PlanningError('trajectory_empty')
    return state_with_joints(start, trajectory.joint_names,
                             trajectory.points[-1].positions)


def _trajectory_positions(trajectory: JointTrajectory) -> list[tuple]:
    if (len(trajectory.joint_names) != len(ARM_JOINTS)
            or set(trajectory.joint_names) != set(ARM_JOINTS)):
        raise PlanningError('trajectory_arm_joints_invalid')
    if not trajectory.points:
        raise PlanningError('trajectory_empty')
    return [_vector(p.positions, len(ARM_JOINTS), 'trajectory_positions')
            for p in trajectory.points]


def retime_stopped_waypoints(trajectory: JointTrajectory, velocity: float = 0.20,
                             acceleration: float = 0.40) -> JointTrajectory:
    """Bound JTC quintic interpolation without changing collision-checked edges.

    Zero velocity/acceleration at each waypoint produces a scalar quintic along
    each original joint-space segment. Its peak speed is 1.875*dq/T and its
    peak acceleration is 10/sqrt(3)*dq/T^2. This intentionally modest baseline
    may pause at waypoints, but cannot curve outside the checked joint edges.
    """
    positions = _trajectory_positions(trajectory)
    if not (math.isfinite(velocity) and math.isfinite(acceleration)
            and 0.0 < velocity <= 0.5 and 0.0 < acceleration <= 1.0):
        raise PlanningError('trajectory_speed_invalid')
    result = JointTrajectory()
    result.joint_names = list(trajectory.joint_names)
    elapsed = 0.0
    for index, q in enumerate(positions):
        if index:
            delta = max(abs(a - b) for a, b in zip(q, positions[index - 1]))
            elapsed += max(0.10, 1.875 * delta / velocity,
                           math.sqrt((10.0 / math.sqrt(3.0)) * delta / acceleration))
        ns = int(round(elapsed * 1e9))
        point = JointTrajectoryPoint()
        point.positions = list(q)
        point.velocities = [0.0] * len(q)
        point.accelerations = [0.0] * len(q)
        point.time_from_start = Duration(sec=ns // 1000000000, nanosec=ns % 1000000000)
        result.points.append(point)
    return result


class PickupPlanner:
    """Namespaced MoveIt services, explicit start state, complete-path contract."""

    def __init__(self, node, namespace='/boxer_pick', reference_frame='world',
                 group_name='arm', ee_link='end_effector_link',
                 service_timeout=8.0, robot_description='', joint_limits=None,
                 planning_time=3.0, validation_timeout=40.0):
        self.node = node
        self.reference_frame = str(reference_frame)
        self.group_name = str(group_name)
        self.ee_link = str(ee_link)
        self.service_timeout = float(service_timeout)
        self.validation_timeout = float(validation_timeout)
        self.planning_time = float(planning_time)
        self.joint_limits = dict(joint_limits or joint_limits_from_urdf(robot_description))
        for name in (*ARM_JOINTS, *GRIPPER_JOINTS):
            if name not in self.joint_limits:
                raise PlanningError(f'joint_limit_missing:{name}')
            lo, hi = _vector(self.joint_limits[name], 2, 'joint_limit')
            if lo >= hi:
                raise PlanningError(f'joint_limit_invalid:{name}')
        self._scene_ids = set()
        group = ReentrantCallbackGroup()
        prefix = '/' + str(namespace).strip('/')
        self._scene = node.create_client(ApplyPlanningScene, prefix + '/apply_planning_scene',
                                         callback_group=group)
        self._plan = node.create_client(GetMotionPlan, prefix + '/plan_kinematic_path',
                                        callback_group=group)
        self._cartesian = node.create_client(GetCartesianPath, prefix + '/compute_cartesian_path',
                                             callback_group=group)
        self._valid = node.create_client(GetStateValidity, prefix + '/check_state_validity',
                                          callback_group=group)
        self._fk = node.create_client(GetPositionFK, prefix + '/compute_fk', callback_group=group)

    def _call(self, client, request, deadline=None):
        deadline = min(deadline or math.inf, time.monotonic() + self.service_timeout)
        remaining = deadline - time.monotonic()
        if remaining <= 0 or not client.wait_for_service(timeout_sec=min(remaining, 2.0)):
            raise PlanningError(f'planning_service_unavailable:{client.srv_name}')
        future = client.call_async(request)
        while not future.done():
            if time.monotonic() >= deadline:
                future.cancel()
                raise PlanningError(f'planning_service_timeout:{client.srv_name}')
            time.sleep(0.002)
        try:
            result = future.result()
        except Exception as exc:
            raise PlanningError(f'planning_service_failed:{client.srv_name}:{exc}') from exc
        if result is None:
            raise PlanningError(f'planning_service_empty:{client.srv_name}')
        return result

    def set_scene(self, obstacles: Sequence[BoxObstacle], selected_id: str):
        """Replace only this helper's boxes; omit the explicitly selected target."""
        request = ApplyPlanningScene.Request()
        request.scene.is_diff = True
        request.scene.robot_state.is_diff = True
        new_ids, objects = set(), []
        seen = set()
        for box in obstacles:
            if str(box.id) in seen:
                raise PlanningError('duplicate_box_id')
            seen.add(str(box.id))
            if str(box.id) == str(selected_id):
                continue
            size = _vector(box.size, 3, 'box_size')
            if any(v <= 0.0 or v > 5.0 for v in size):
                raise PlanningError('box_size_invalid')
            _vector((box.pose.position.x, box.pose.position.y, box.pose.position.z), 3, 'box_position')
            obj = CollisionObject()
            obj.id = 'boxer_pick_box_' + str(box.id)
            obj.header.frame_id = self.reference_frame
            obj.operation = CollisionObject.ADD
            primitive = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=list(size))
            pose = copy.deepcopy(box.pose)
            pose.orientation = normalized_quaternion(pose.orientation)
            obj.primitives = [primitive]
            obj.primitive_poses = [pose]
            objects.append(obj)
            new_ids.add(obj.id)
        for old_id in sorted(self._scene_ids - new_ids):
            objects.append(CollisionObject(id=old_id, operation=CollisionObject.REMOVE))
        request.scene.world.collision_objects = objects
        if not self._call(self._scene, request).success:
            raise PlanningError('planning_scene_update_failed')
        self._scene_ids = new_ids
        return True

    def _check_bounds(self, state: RobotState):
        positions = dict(zip(state.joint_state.name, state.joint_state.position))
        for name in (*ARM_JOINTS, *GRIPPER_JOINTS):
            if name not in positions or not math.isfinite(positions[name]):
                raise PlanningError(f'joint_state_missing_or_nonfinite:{name}')
            lo, hi = self.joint_limits[name]
            if not lo - 1e-7 <= positions[name] <= hi + 1e-7:
                raise PlanningError(f'joint_out_of_bounds:{name}:{positions[name]:.6f}')

    def _valid_state(self, state: RobotState, deadline=None):
        self._check_bounds(state)
        request = GetStateValidity.Request()
        request.robot_state = state
        # Empty group checks the complete robot, including gripper and mounts.
        request.group_name = ''
        response = self._call(self._valid, request, deadline)
        if not response.valid:
            pairs = sorted({f'{c.contact_body_1}__{c.contact_body_2}' for c in response.contacts})
            raise PlanningError('state_collision_or_invalid:' + ','.join(pairs[:4]))

    def validate_gripper(self, start_joint_state: JointState | RobotState,
                         from_position: float, to_position: float):
        from_position, to_position = _vector((from_position, to_position), 2, 'gripper')
        count = max(1, math.ceil(abs(to_position - from_position) / 0.001))
        if count > 100:
            raise PlanningError('gripper_sweep_invalid')
        deadline = time.monotonic() + self.validation_timeout
        for i in range(count + 1):
            position = from_position + (to_position - from_position) * i / count
            state = state_with_joints(start_joint_state, GRIPPER_JOINTS, [position, position])
            self._valid_state(state, deadline)
        return True

    def validate_segment(self, trajectory: JointTrajectory,
                         start_joint_state: JointState | RobotState,
                         gripper_position: float, check_closure=False,
                         gripper_close=-0.010):
        positions = _trajectory_positions(trajectory)
        state = state_with_joints(start_joint_state, GRIPPER_JOINTS,
                                  [gripper_position, gripper_position])
        self._check_bounds(state)
        q_map = dict(zip(state.joint_state.name, state.joint_state.position))
        previous = tuple(q_map[name] for name in trajectory.joint_names)
        # Returning to a frozen start through an unplanned connector is not
        # allowed. Small measurement drift is checked in the same collision sweep.
        if max(abs(a - b) for a, b in zip(previous, positions[0])) > 0.08:
            raise PlanningError('trajectory_start_changed')
        deadline = time.monotonic() + self.validation_timeout
        samples = 0
        self._valid_state(state, deadline)
        for q in positions:
            count = max(1, math.ceil(max(abs(a - b) for a, b in zip(q, previous)) / 0.025))
            samples += count
            if samples > 3000:
                raise PlanningError('trajectory_validation_budget_exceeded')
            for i in range(1, count + 1):
                interpolated = [a + (b - a) * i / count for a, b in zip(previous, q)]
                sample = state_with_joints(state, trajectory.joint_names, interpolated)
                self._valid_state(sample, deadline)
            previous = q
        if check_closure:
            self.validate_gripper(trajectory_endpoint(state, trajectory),
                                  gripper_position, gripper_close)
        return True

    def fk(self, state: RobotState) -> Pose:
        request = GetPositionFK.Request()
        request.header.frame_id = self.reference_frame
        request.fk_link_names = [self.ee_link]
        request.robot_state = state
        response = self._call(self._fk, request)
        if (response.error_code.val != 1 or not response.pose_stamped
                or self.ee_link not in response.fk_link_names):
            raise PlanningError('forward_kinematics_failed')
        return response.pose_stamped[response.fk_link_names.index(self.ee_link)].pose

    def motion_request(self, state: RobotState, pose: Pose) -> GetMotionPlan.Request:
        request = GetMotionPlan.Request()
        motion = request.motion_plan_request
        motion.group_name = self.group_name
        motion.pipeline_id = 'ompl'
        motion.planner_id = 'RRTConnect'
        motion.start_state = copy.deepcopy(state)
        motion.num_planning_attempts = 2
        motion.allowed_planning_time = self.planning_time
        motion.max_velocity_scaling_factor = 0.15
        motion.max_acceleration_scaling_factor = 0.15
        motion.workspace_parameters.header.frame_id = self.reference_frame
        motion.workspace_parameters.min_corner.x = -1.0
        motion.workspace_parameters.min_corner.y = -1.0
        motion.workspace_parameters.min_corner.z = -0.2
        motion.workspace_parameters.max_corner.x = 1.0
        motion.workspace_parameters.max_corner.y = 1.0
        motion.workspace_parameters.max_corner.z = 1.5
        constraints = Constraints(name='boxer_center_side_grasp')
        position = PositionConstraint()
        position.header.frame_id = self.reference_frame
        position.link_name = self.ee_link
        position.weight = 1.0
        position.constraint_region.primitives = [SolidPrimitive(
            type=SolidPrimitive.SPHERE, dimensions=[0.0015])]
        sphere_pose = Pose()
        sphere_pose.position = copy.deepcopy(pose.position)
        sphere_pose.orientation.w = 1.0
        position.constraint_region.primitive_poses = [sphere_pose]
        orientation = OrientationConstraint()
        orientation.header.frame_id = self.reference_frame
        orientation.link_name = self.ee_link
        orientation.orientation = copy.deepcopy(pose.orientation)
        orientation.absolute_x_axis_tolerance = 0.025
        orientation.absolute_y_axis_tolerance = 0.025
        orientation.absolute_z_axis_tolerance = 0.025
        orientation.parameterization = OrientationConstraint.ROTATION_VECTOR
        orientation.weight = 1.0
        constraints.position_constraints = [position]
        constraints.orientation_constraints = [orientation]
        motion.goal_constraints = [constraints]
        return request

    def cartesian_request(self, state: RobotState, pose: Pose) -> GetCartesianPath.Request:
        request = GetCartesianPath.Request()
        request.header.frame_id = self.reference_frame
        request.group_name = self.group_name
        request.link_name = self.ee_link
        request.start_state = copy.deepcopy(state)
        request.waypoints = [copy.deepcopy(pose)]
        request.max_step = 0.003
        request.jump_threshold = 0.0  # Use the absolute radian threshold below.
        request.revolute_jump_threshold = 0.15
        request.prismatic_jump_threshold = 0.002
        request.avoid_collisions = True
        return request

    def _complete_insertion(self, response, start_state, grasp_pose):
        if response.error_code.val != 1:
            raise PlanningError(f'cartesian_planning_failed:{response.error_code.val}')
        if not math.isfinite(response.fraction) or response.fraction < 1.0 - 1e-9:
            raise PlanningError(f'cartesian_path_incomplete:{response.fraction:.3f}')
        trajectory = response.solution.joint_trajectory
        positions = _trajectory_positions(trajectory)
        if len(positions) < 2:
            raise PlanningError('cartesian_path_empty')
        for a, b in zip(positions, positions[1:]):
            if max(abs(x - y) for x, y in zip(a, b)) > 0.15 + 1e-6:
                raise PlanningError('cartesian_joint_jump')
        endpoint = self.fk(trajectory_endpoint(start_state, trajectory))
        error = math.dist((endpoint.position.x, endpoint.position.y, endpoint.position.z),
                          (grasp_pose.position.x, grasp_pose.position.y, grasp_pose.position.z))
        if not math.isfinite(error) or error > 0.003:
            raise PlanningError(f'grasp_center_not_reached:{error:.6f}')
        a, b = tool_forward(endpoint.orientation), tool_forward(grasp_pose.orientation)
        qa, qb = normalized_quaternion(endpoint.orientation), normalized_quaternion(grasp_pose.orientation)
        dot = abs(qa.x * qb.x + qa.y * qb.y + qa.z * qb.z + qa.w * qb.w)
        if (abs(a[2]) > math.sin(math.radians(10))
                or sum(x * y for x, y in zip(a, b)) < 0.995
                or 2.0 * math.acos(min(1.0, dot)) > 0.06):
            raise PlanningError('grasp_orientation_misaligned')
        return retime_stopped_waypoints(trajectory), error

    def _shortcut_pregrasp(self, trajectory: JointTrajectory,
                           start_state: RobotState) -> JointTrajectory:
        """Remove OMPL sampling waypoints only across a mesh-checked joint edge.

        The Cartesian insertion is never shortened by this function. Prefix
        motion can use any collision-free arm configuration on its way to the
        required side-grasp pose, and retaining 100+ interpolated OMPL points
        would otherwise cause unnecessary stops in the conservative retiming.
        """
        positions = _trajectory_positions(trajectory)
        result = copy.deepcopy(trajectory)
        result.points = [copy.deepcopy(trajectory.points[0])]
        index, attempts = 0, 0
        deadline = time.monotonic() + self.validation_timeout
        while index < len(positions) - 1:
            candidate = len(positions) - 1
            while candidate > index + 1 and attempts < 32:
                attempts += 1
                before, after = positions[index], positions[candidate]
                count = max(1, math.ceil(max(abs(a - b) for a, b in zip(before, after)) / 0.025))
                try:
                    for i in range(1, count + 1):
                        q = [a + (b - a) * i / count for a, b in zip(before, after)]
                        self._valid_state(state_with_joints(start_state, trajectory.joint_names, q), deadline)
                    break
                except PlanningError as exc:
                    if not str(exc).startswith('state_collision_or_invalid:'):
                        raise
                    candidate = index + max(1, (candidate - index) // 2)
            if attempts >= 32:
                # The original remaining path is still densely validated by
                # the caller; exhausting optimization never bypasses checks.
                result.points.extend(copy.deepcopy(trajectory.points[index + 1:]))
                break
            result.points.append(copy.deepcopy(trajectory.points[candidate]))
            index = candidate
        return result

    def plan(self, start: JointState, center: Sequence[float],
             obstacles: Sequence[BoxObstacle], *, selected_id: str,
             pregrasp_distance=0.075, gripper_open=0.019, gripper_close=-0.010,
             yaw_offsets=(0.0, 0.25, -0.25)) -> PickupPlan:
        center = _vector(center, 3, 'center')
        self.set_scene(obstacles, selected_id)
        frozen = state_with_joints(start, GRIPPER_JOINTS, [gripper_open, gripper_open])
        self._valid_state(frozen)
        failures = []
        for yaw in yaw_offsets:
            for roll_sign in (1, -1):
                try:
                    requested_pre, grasp = side_grasp_poses(
                        center, pregrasp_distance, yaw, roll_sign)
                    response = self._call(self._plan, self.motion_request(frozen, requested_pre))
                    motion = response.motion_plan_response
                    if motion.error_code.val != 1:
                        raise PlanningError(f'pregrasp_planning_failed:{motion.error_code.val}')
                    pregrasp = retime_stopped_waypoints(self._shortcut_pregrasp(
                        motion.trajectory.joint_trajectory, frozen))
                    pre_state = trajectory_endpoint(frozen, pregrasp)
                    actual_pre = self.fk(pre_state)
                    pre_error = math.dist(
                        (actual_pre.position.x, actual_pre.position.y, actual_pre.position.z),
                        (requested_pre.position.x, requested_pre.position.y, requested_pre.position.z))
                    if pre_error > 0.003 or not math.isfinite(pre_error):
                        raise PlanningError('pregrasp_pose_not_reached')
                    forward = tool_forward(actual_pre.orientation)
                    ray = (center[0] - actual_pre.position.x,
                           center[1] - actual_pre.position.y,
                           center[2] - actual_pre.position.z)
                    ray_length = math.sqrt(sum(v * v for v in ray))
                    if (ray_length < 0.005 or abs(forward[2]) > math.sin(math.radians(10))
                            or sum(a * b for a, b in zip(forward, ray)) / ray_length < 0.995):
                        raise PlanningError('pregrasp_orientation_misaligned')
                    # Hold the reached orientation throughout the insertion.
                    grasp.orientation = copy.deepcopy(actual_pre.orientation)
                    cartesian = self._call(self._cartesian, self.cartesian_request(pre_state, grasp))
                    insertion, error = self._complete_insertion(cartesian, pre_state, grasp)
                    self.validate_segment(pregrasp, frozen, gripper_open)
                    self.validate_segment(insertion, pre_state, gripper_open,
                                          check_closure=True, gripper_close=gripper_close)
                    display = DisplayTrajectory()
                    display.model_id = 'om6dof'
                    display.trajectory_start = copy.deepcopy(frozen)
                    display.trajectory = [RobotTrajectory(joint_trajectory=copy.deepcopy(pregrasp)),
                                          RobotTrajectory(joint_trajectory=copy.deepcopy(insertion))]
                    return PickupPlan(pregrasp, insertion, frozen, actual_pre, grasp,
                                      center, display, error)
                except PlanningError as exc:
                    failures.append(f'yaw={float(yaw):+.2f},roll={roll_sign}:{exc}')
        raise PlanningError('pickup_plan_unavailable:' + '; '.join(failures))
