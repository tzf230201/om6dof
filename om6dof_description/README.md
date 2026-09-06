# om6dof_description

URDF/Xacro, meshes, kinematic frames, joint limits, and inertial parameters for
the OM6DOF manipulator. This package is the common robot model for MoveIt,
`robot_state_publisher`, collision checking, and the KDL gravity model used by
the Mode 0 leader controller.

For the complete gravity-compensation architecture and the current validation
status, see [the leader-arm research record](../docs/leader_arm_gravity_compensation.md).

## Kinematic chain

The arm chain used by the leader controller is:

```text
world -> base_link (20 mm pedestal) -> link1 -> joint1 -> link2 -> joint2 -> link3
      -> joint3 -> link4 -> joint4 -> link5 -> joint5
      -> link6 -> joint6 -> link7 -> end_effector_link
```

The gripper fingers and D405 payload are branches from the wrist structure.
They are visible in the full URDF tree but are not automatically included in a
single serial `KDL::Tree::getChain()` result.

### Arm joint contract

| Joint | Axis | Lower limit | Upper limit | Velocity limit |
|---|---|---:|---:|---:|
| `joint1` | Z | -2.82743 rad | +2.82743 rad | 4.8 rad/s |
| `joint2` | Y | -2.04204 rad | +2.10487 rad | 4.8 rad/s |
| `joint3` | Y | -1.88496 rad | +2.13628 rad | 4.8 rad/s |
| `joint4` | Z | -2.82743 rad | +2.82743 rad | 4.8 rad/s |
| `joint5` | Y | -1.97920 rad | +2.10487 rad | 4.8 rad/s |
| `joint6` | Z | -2.82743 rad | +2.82743 rad | 4.8 rad/s |

The companion leader controller validates this exact joint order and applies a
separate soft-limit margin before the hard URDF limits.

## Dynamics and gravity-model notes

Each serial-chain link must have a positive mass and physically valid inertia.
The leader controller refuses to configure if required dynamics are missing or
the KDL joint order differs from `joint1..joint6`.

Existence and positivity do not prove that a model is physically accurate.
Before publication or calibrated torque control, record for every link:

- mass measurement method and uncertainty;
- centre-of-mass source and coordinate frame;
- inertia source (CAD, identification, or approximation);
- mesh scale and origin;
- payload contents and mounting pose;
- author, date, and Git commit.

Earlier gravity work found two model issues:

1. approximately 0.114 kg of gripper and D405 payload mass lived on branches
   outside the extracted serial KDL chain;
2. several link centres of mass were corrected after the first current-data fit.

Consequently, coefficients fitted against the pre-correction model are stale.
The historical plots are retained as diagnostics in
[`docs/assets/identification/legacy-20260820`](../docs/assets/identification/legacy-20260820/MANIFEST.md),
not as final validation.

## Meshes and images

The `meshes/` directory contains the main chain, gripper fingers, D405, and
wrist-mount STL geometry. These tracked CAD assets can be used to create a
neutral RViz/URDF render for documentation.

No verified physical OM6DOF photograph was found during the 2026-08-21 audit.
Do not use photographs of a Piper or phosphobot as a substitute. A future
physical photo should record author, date, and reuse permission; a future RViz
render should record the URDF commit and fixed camera pose.

## Expand and inspect

Normal profile:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
xacro \
  ~/ros2_ws/src/om6dof/om6dof_bringup/urdf/om6dof.urdf.xacro \
  > /tmp/om6dof-normal.urdf
check_urdf /tmp/om6dof-normal.urdf
```

Leader profile:

```bash
xacro \
  ~/ros2_ws/src/om6dof_leader_controller/urdf/om6dof.leader.urdf.xacro \
  > /tmp/om6dof-leader.urdf
check_urdf /tmp/om6dof-leader.urdf
```

When reviewing an expanded file, verify exactly one intended `<ros2_control>`
block. A stale `robot_state_publisher` can publish a description without that
block and race the hardware owner at startup; use one publisher and one
controller manager per profile.

## Change-control checklist

Any change to mass, CoM, inertia, joint origin, axis, payload, or frame can alter
gravity compensation. After such a change:

1. expand and validate the URDF;
2. compare the KDL joint order;
3. regenerate gravity predictions at recorded poses;
4. invalidate old fitted current coefficients explicitly;
5. rerun supported low-gain gravity trials;
6. update the research document, dataset manifest, and model commit.
