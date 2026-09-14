# Component-center target selection

Separate experiment launch selects target_node_selection=component_center.
Default core selection remains all; strict atomic queries retain exact requested targets.
Selection uses each same-class connected component bounding-box center and the nearest existing surface node, with stable ID tie-breaking.
No environment geometry, collision tolerance, green reachability dataset or execution policy changed.
No robot, camera, controller or action execution started. Active user launch not restarted.

Validation:
- Build om6dof_dd_gng succeeded.
- 21 Python tests passed (experiment/V2 launch).
- 3 new selection C++ tests passed (dense-top bias, separate objects/classes, stable ties/empty).
- 13 existing reachability and 4 workspace importer tests passed.
- Saved bottle replay: 170 component nodes; Z bounds 0.133037 to 0.211438 m; selected existing node 50943 Z=0.176913 m.
- Replay-only budget200, isolated ROS domain71: target_intersection_exact_blocked_or_disconnected, 159 rejected candidates, 9676 exact checks, 776.003 ms.
- No validated trajectory to the selected center on this saved scene. No fallback to a top target.

The selected point is the middle of the observed component, not a full unseen-object shape or a certified grasp pose. Gripper orientation, clearance and closure are not supplied by this change.
