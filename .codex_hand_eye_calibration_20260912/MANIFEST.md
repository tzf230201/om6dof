# DD-GNG D435i hand-eye calibration repair — 2026-09-12

- Backups with `.before` suffix preserve every pre-edit file.
- `SHA256_BEFORE.txt` records the pre-edit hashes.
- `SHA256_AFTER.txt` records the final hashes.
- The V2 launch accepts a measured `om6dof.hand_eye.v1` artifact.
- Launch and C++ independently reject missing, malformed, weak, or wrong-serial calibration.
- C++ uses the measured EoE-to-colour transform and live RealSense factory depth-to-colour extrinsic.
- The new collector is read-only and never commands the robot.
- Build completed for `om6dof_dd_gng` and `om6dof_pick_and_place`.
- Package test result: 939 tests, 0 errors, 0 failures, 15 skipped.
- Protected ordinary reachability GNG hashes remained unchanged:
  - `reachability_graph_node.cpp`: `a3605e37110eac1d2e00e7a6ea9271edbc6721d295173a8ee689f378d3e98431`
  - `reachability_graph.hpp`: `d3bdc9fab36695217dcd4dde7fa2b67b181c180f3346eab5d0c159b3d4bcbce8`
- No controller restart, torque command, operation-mode change, or robot motion was performed.
