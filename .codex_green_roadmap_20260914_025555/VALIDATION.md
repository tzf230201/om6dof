# Green-only experiment roadmap validation

Default separate launch now selects status=pose_found, position_found=1, pose_found=1.
Blue pose_near_singular and yellow orientation_unresolved are excluded.
Original experiment CSV SHA256 verified unchanged against before.json.
Environment parameters unchanged; execution still defaults false.

- 21 pytest cases passed (experiment launch + V2 launch).
- colcon build --packages-select om6dof_dd_gng --symlink-install succeeded.
- Isolated ROS domain 71 replay: 3,198 input/accepted green nodes, 18,554 candidate edges, one connected component.
- Every runtime graph joint configuration verified to originate from a selected green CSV row.
- Max FK discrepancy 0.001 m; zero current self-collision/bounds/duplicate rejections.
- Saved bottle scene did NOT produce a validated path: exact_collision_replan_exhausted, 21 replans, 1,256 exact checks, final planning cycle 223.614 ms.
  This does not prove no collision-free path exists. The present search budget was exhausted for the sparser graph.
- No robot, controller, torque, camera, or action execution was started. Replay child stopped normally.

The previous all-category replay succeeded; that result must not be attributed to green-only mode.
