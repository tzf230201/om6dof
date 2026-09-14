"""Read-only validation of ros2_control's independent command channels.

Topology validation does not certify the collision safety of a modified path.
"""


def additive_controller_blockers(controllers, joints, arm_name, offset_name):
    blockers = []
    names = [controller.name for controller in controllers]
    if len(names) != len(set(names)) or arm_name == offset_name:
        return ['controller_snapshot_ambiguous']
    by_name = {controller.name: controller for controller in controllers}
    expected = (
        (arm_name, 'joint_trajectory_controller/JointTrajectoryController', 'position'),
        (offset_name, 'forward_command_controller/ForwardCommandController', 'position_offset'),
    )
    for name, controller_type, interface in expected:
        controller = by_name.get(name)
        if controller is None or controller.state != 'active':
            blockers.append(f'required_controller_not_active:{name}')
            continue
        if controller.type != controller_type:
            blockers.append(f'controller_type_mismatch:{name}')
        required = {f'{joint}/{interface}' for joint in joints}
        claims = list(controller.claimed_interfaces)
        if set(claims) != required or len(claims) != len(required):
            blockers.append(f'controller_command_interfaces_mismatch:{name}')
        if any(other.name != name and other.state == 'active'
               and required.intersection(other.claimed_interfaces)
               for other in controllers):
            blockers.append(f'controller_command_interfaces_conflict:{name}')
    return blockers
