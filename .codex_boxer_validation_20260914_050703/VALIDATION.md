# BOXER isolated validation

New package om6dof_boxer; source/model checkout under src/.boxer_runtime/COLCON_IGNORE.
Existing robot/perception/controller packages and frozen datasets not edited for this task.

- Geometry pytest: 3 passed.
- Python compile passed; colcon build succeeded; launch --show-args passed.
- Real official BoxerNet checkpoint loaded 100% (320 tensors, 85.81M parameters).
- Synthetic inference: 12.677 s CPU. Predicted OBB score 0.914, zero interior synthetic depth points.
  This is an API/model smoke test only; no claim of real object accuracy or valid cutting.
- ROS output publisher exercised on isolated domain71 for both empty and nonempty colored cuts.
- No new camera owner, live capture, robot state, controller restart, torque or action command during verification.
- Live camera/IMU extrinsics and gravity sign still require inspection in the operator's standalone capture.
- BOXER outputs oriented boxes, not segmentation masks; manual labels are not model classifications.
- Global broken torch left unchanged; isolated torch2.6 CPU used. No CUDA performance claim.
