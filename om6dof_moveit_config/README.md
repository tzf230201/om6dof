# om6dof_moveit_config

> **Control-profile compatibility:** MoveIt requires the normal OM6DOF position
> profile and `arm_controller`. It is mutually exclusive with the isolated
> Mode 0 leader hardware stack. Support the arm, disarm, verify torque OFF, and
> restart the hardware owner before switching profiles. See the
> [leader-arm gravity-compensation record](../docs/leader_arm_gravity_compensation.md).

Start MoveIt and RViz after the OM6DOF hardware/controller stack is running:

```bash
ros2 launch om6dof_moveit_config om6dof_moveit.launch.py
```

Disable RViz for headless operation:

```bash
ros2 launch om6dof_moveit_config om6dof_moveit.launch.py start_rviz:=false
```
