# DD-GNG provisional world-Z correction — 2026-09-12

Purpose: apply the ruler-verified `-0.0185 m` world-Z correction coherently to
the V2 native-depth and raw-RGB camera poses used by environment DD-GNG and
semantic reprojection.

Evidence:

- camera range about 0.224 m: raw world Z 0.1483 m, corrected Z 0.1298 m,
  ruler reference about 0.1300 m;
- camera range about 0.410 m: raw world Z 0.1327 m, corrected Z 0.1142 m,
  ruler crosshair about 0.114 m.

Safety boundary: this correction is provisional. It does not modify the URDF,
does not set `calibration_verified`, and does not unlock physical execution.
A valid serial-bound hand-eye artifact supersedes it.

Backups captured before editing:

- `topo_gng_node.cpp.before`
- `topo_gng_v2.yaml.before`
- `test_v2_launch.py.before`

Validation:

- `colcon build --packages-select om6dof_dd_gng --symlink-install`: passed;
- package tests: 144 passed, 0 errors, 0 failures, 0 skipped;
- live D435i perception-only check: mapping accepted, exact timestamp TF,
  498–500 environment nodes, TensorRT YOLO 18–21 ms, geometry usable, and no
  inference, TF, or processing errors in the sampled interval.

Reachability implementation was not edited; its protected hashes are recorded
in `SHA256_AFTER.txt`.
