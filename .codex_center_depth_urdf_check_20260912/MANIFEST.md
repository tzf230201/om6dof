# Center-depth URDF verification tool

Read-only addition for checking the D435i RGB-centre depth in the `world`
frame through the nominal V2 URDF TF chain. No controller, torque, robot
motion, DD-GNG algorithm, frozen experiment, or reachability implementation
was changed.

Added:

- `om6dof_pick_and_place/om6dof_pick_and_place/center_depth_urdf_check.py`
- `om6dof_pick_and_place/launch/center_depth_urdf_check.launch.py`
- `om6dof_pick_and_place/test/test_center_depth_urdf_check.py`

Existing files received additive edits only:

- `om6dof_pick_and_place/setup.py`: one console entry point
- `om6dof_pick_and_place/README.md`: operator instructions

Validation:

- Python compile passed.
- Launch argument discovery passed.
- `om6dof_pick_and_place`: 73 tests, 0 errors, 0 failures.
- Installed executable discovered as `center_depth_urdf_check`.
- Live start correctly rejected a concurrently owned camera. `fuser` showed
  PID 297301 (`apriltag_detector`) owning `/dev/video4` and `/dev/video5`;
  it was not stopped or modified.
- Fixed the first-frame OpenCV visibility race that stopped publishing before
  a newly created window had become visible. A subsequent headless live check
  delivered `/om6dof/center_depth/status` to a separate subscriber and kept
  publishing valid depth/world coordinates for the full test window.
- Added an explicit verifier-only `world_z_offset` launch default of `-0.0185`
  m after the operator measured a one-point residual of +18.5 mm. The raw URDF
  point remains visible and is published separately. Live evidence at the same
  target was `raw Z=0.14847 m`, `corrected Z=0.12997 m`. No URDF/planning or
  execution transform was changed.

Protected reachability hashes remained:

- `reachability_graph_node.cpp`: `a3605e37110eac1d2e00e7a6ea9271edbc6721d295173a8ee689f378d3e98431`
- `reachability_graph.hpp`: `d3bdc9fab36695217dcd4dde7fa2b67b181c180f3346eab5d0c159b3d4bcbce8`
