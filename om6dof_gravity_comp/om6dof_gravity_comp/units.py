"""What the numbers coming out of this arm's driver actually mean.

Every value here was read out of the repository rather than assumed, because
the units are not uniform and one of them is a trap.

Arm joints (1-6)
    ``/joint_states.effort`` carries the **raw** Dynamixel current register
    value. Both ``xm430_w350.model`` and ``xm430_w210.model`` declare

        Present Current   1.0   raw   signed   0.0

    so no scaling is applied on the way out. It is not milliamps and it is
    not newton-metres.

Gripper (dxl7)
    A per-device override in ``om6dof.ros2_control.xacro`` changes that:

        Present Current,2.69,mA,signed,0.0

    so the gripper's effort *is* in milliamps. Nothing here logs the gripper,
    but the difference is why the raw/mA distinction is kept explicit
    throughout rather than folded into one number.

That override is also where the tick size is stated outright -- the comment
reads "45 raw ticks * 2.69 mA/tick = 121.05 mA" -- which is where
``CURRENT_TICK_MA`` comes from.

Velocity
    ``Present Velocity`` is declared with scale 0.0239691227 and unit rad/s,
    so ``/joint_states.velocity`` is already in rad/s and needs no
    conversion.
"""

from __future__ import annotations

from typing import Sequence

# Milliamps per raw current tick, from the dxl7 unit override in
# om6dof_bringup/urdf/om6dof.ros2_control.xacro.
CURRENT_TICK_MA = 2.69

JOINT_NAMES = tuple(f"joint{index}" for index in range(1, 7))

# From the ID/model table at the top of om6dof.ros2_control.xacro.
JOINT_SERVO_MODELS = {
    "joint1": "XM430-W350",
    "joint2": "XM430-W350",
    "joint3": "XM430-W350",
    "joint4": "XM430-W210",
    "joint5": "XM430-W350",
    "joint6": "XM430-W210",
}

# Axes as declared in the URDF, kept here so a reader can sanity-check which
# joints gravity can load at all: only the Y axes can.
JOINT_AXES = {
    "joint1": "Z", "joint2": "Y", "joint3": "Y",
    "joint4": "Z", "joint5": "Y", "joint6": "Z",
}

CURRENT_UNIT_RAW = "raw_dynamixel_ticks"


def raw_to_ma(raw: float) -> float:
    """Convert a raw current tick count to milliamps."""
    return float(raw) * CURRENT_TICK_MA


def ma_to_raw(milliamps: float) -> float:
    return float(milliamps) / CURRENT_TICK_MA


def order_by_joint(names: Sequence[str], values: Sequence[float]) -> list:
    """Reorder a JointState field into JOINT_NAMES order.

    ``/joint_states`` does not arrive in chain order and the order is not
    stable across runs, so anything that reads it positionally is reading a
    different joint than it thinks.
    """
    lookup = {name: index for index, name in enumerate(names)}
    out = []
    for joint in JOINT_NAMES:
        index = lookup.get(joint)
        out.append(
            float(values[index])
            if index is not None and index < len(values)
            else None
        )
    return out


# The arm stack runs whatever RMW is default for the distro: its systemd unit
# sets neither RMW_IMPLEMENTATION nor a Cyclone URI, so on Humble that is
# rmw_fastrtps_cpp.
STACK_RMW = "rmw_fastrtps_cpp"


def match_stack_rmw() -> None:
    """Speak the same DDS implementation as the arm stack, for this process.

    rmw_cyclonedds and rmw_fastrtps do not interoperate for services. Nodes
    still appear across the two -- RTPS discovery is standard -- so a shell
    set to Cyclone can list every node on a FastDDS stack and then have every
    service call time out, which looks like the service being broken rather
    than the client being on the wrong stack. Measured here: the same call
    answers in 0.5 s under FastDDS and never under Cyclone.

    A Cyclone URI pinned to a physical interface causes a similar-looking
    stall, so it is dropped too; om6dof-hardware.service already unsets it
    for the same reason. Only this process is affected.
    """
    import os
    import sys

    current = os.environ.get("RMW_IMPLEMENTATION", "")
    if current and current != STACK_RMW:
        print(f"note: switching this process from {current} to {STACK_RMW} to "
              "match the arm stack; they cannot exchange services",
              file=sys.stderr)
    os.environ["RMW_IMPLEMENTATION"] = STACK_RMW
    if os.environ.pop("CYCLONEDDS_URI", None) is not None:
        print("note: dropped CYCLONEDDS_URI for this process", file=sys.stderr)
