# Experiment roadmap validation — 2026-09-14

New separate launch: ddgng_experiment_reachability.launch.py.
Environment configuration is copied unchanged; only the reachability block is overridden.
Dataset SHA256 before/after verified unchanged; see dataset.sha256.
Every one of the 15,064 replay graph nodes was checked against the original recorded
joint-witness set: no synthesized nodes or GNG prototypes in this backend.

Checks: 52 Python tests passed (experiment launch, V2 launch, graph-pick coordinator);
4 new importer/spatial-neighbor C++ tests passed; 13 existing reachability C++ tests passed.
Package build passed. Installed launch --show-args passed. Initial launch test failed
because ROS attempted a sandbox read-only log path; test now supplies a temporary log dir.

Isolated domain71 snapshot replay: 15,175 input witnesses, 15,064 accepted,
111 rejected by current bounds/self-collision/duplicate filtering. Max FK discrepancy
1 mm. Graph has 93,355 candidate edges, 13 components. Valid path has 8 graph edges,
35.919624 mm goal distance, 124 exact state checks, 82.982304 ms planning cycle.
Offline coordinator fixture with synthetic track metadata accepted 10 trajectory points;
its sole blocker was deliberately execution_disabled_at_launch. Negative replay with
an obstacle at initial EoE returned current_full_body_intersects_environment and no path.

All replay traffic was isolated on domain71, with explicit topic parameters and no
camera/controller/action clients. No hardware restart, torque command or robot motion.
The running original planning launch was left unchanged. Stop it before using the new
launch, so only one process owns RealSense. Real joint states are still required.

Initial build log called candidate edges validated; final build corrected this label
and RViz namespace to workspace_candidate_edges. Only shortlisted, fully checked
trajectory edges are certified; all background candidate edges are not collision proofs.

Full behavior, launch command and limitations are documented in
om6dof_dd_gng/docs/experiment_workspace_roadmap.md. No experiment or paper results changed.
