# Implementation audit for the new ICRA 2027 manuscript

Audit date: 2026-09-14. This is a read-only implementation audit of the working source tree; it is not evidence that the active ROS processes loaded that source, that a deployment succeeded, or that a physical trial passed. No robot, camera, ROS node, or controller was started. Paths below are relative to the `om6dof` repository root.

## Defensible scope and distinction from the earlier paper

The existing `om6dof_dd_gng/paper/icra2027_reachability/icra2027_latex/root.tex` is titled **Configuration-Retaining Reachability Roadmaps for Dual-Topology Manipulation under Environment Updates**. It already presents full-configuration roadmaps, dual topology, capsule filtering, lazy interpolated MoveIt/FCL checking, GNG/guarded-GNG/Halton comparisons, and the frozen 21,600-query benchmark. These are inherited components, not new contributions of a second paper. Do not transfer its numerical outcomes to the new online pipeline.

The new, defensible subject is **perception-conditioned, object-centered pregrasp planning with separate semantic environment and recorded-configuration graphs**. Its potentially publishable contribution is the integration and evaluation of:

1. Depth-supported YOLOX semantic evidence with source-age and camera-motion guards feeding bounded semantic-attention DD-GNG;
2. Explicit separation between collision geometry, a component-center target reference, changing graph-node identity, and persistent object-track identity;
3. Reuse of recorded green workspace joint witnesses with stand-off/tool-axis feasibility and target-preserving collision checks;
4. A reviewable preview and pre-execution validation interface, with limitations below.

None of these integration choices alone establishes a new planning algorithm, state-of-the-art performance, collision guarantees, or successful grasping. A new benchmark must connect perception effects to planning outcomes and compare meaningful ablations.

## 1. Environment construction and semantics

Source: `om6dof_dd_gng/src/topo_gng_node.cpp`, especially `processFrame`, `labelGraph`, `publishObjectClusters`, and `publishEnvironmentGraphData`; active learner: `include/om6dof_dd_gng/dynamic_density_gng.hpp`; attention lifecycle: `semantic_density_attention.hpp`. The legacy `ddgng.hpp` is **not** the active environment learner. `docs/semantic_ddgng_v1.md` documents the adaptation and evidence limits.

Each accepted depth pixel becomes a metric world-frame point:

\[
 p_W = T_{WC}(t)\,\pi^{-1}(u,v,d;K).
\]

Use the actual RealSense deprojection model in implementation; a pinhole equation in the manuscript is a shorthand, not a claim that lens distortion is ignored. Robot-body masking removes observed points inside configured robot volumes. TF and frame-freshness acceptance affect whether a mapping update is usable. The nominal world-Z correction is `-0.0185 m` in the V2 configuration; its implementation explicitly does not certify hand-eye calibration. The screenshot measuring one height is not a multi-view calibration evaluation.

YOLOX produces 2-D detections. Current graph nodes are reprojected into the retained detector source frame and receive class evidence only after image-box, depth, and visibility tests. A representative implemented scoring expression is

\[
 s_{ik}=c_k\exp[-\tfrac12(\Delta z_{ik}/g_k)^2]
                \exp[-\tfrac12(\Delta z_i^{\rm vis}/g_i^{\rm vis})^2]
                (0.7+0.3(1-r_{ik})).
\]

Here `c_k` is detector confidence, `r` is normalized box-center distance clipped at one, the box gate is `clip(max(0.12 m, 2.5*1.4826*MAD, 0.08*median_depth), 0.12 m, 0.60 m)`, and the visibility gate is `max(0.10 m, 0.05*observed_depth)`. Close cross-class scores (`best < 1.20*competitor`) produce UNKNOWN. A box also needs sufficient supporting nodes and summed evidence. These are heuristic scores, not calibrated probabilities. They cannot resolve every box overlap or reconstruct unseen object surfaces.

Source-age guards reject detector results older than 1.0 s, camera translation beyond 0.02 m, or camera rotation beyond 0.10 rad since the retained source. Time uses source host receipt, not calibrated exposure/encoder synchronization. Cached results do not renew density-attention TTL. Temporal display-label evidence is a separate mechanism and may still reuse cached results; it must not be described as independent confidence accumulation.

### Semantic density attention

For directly supported semantic points, the node proposes a spherical attention region (default radius 0.05 m) with `S=1+(Smax-1)*s`, `Smax=3`. Overlap uses maximum strength. Deterministic greedy selection limits the set to 64 separated centers. The active learner has a hard node budget, stable non-reused IDs inside an epoch, and competitive-Hebbian aged edges.

For a sample `p`, the nearest node and its neighbors move toward it with rates 0.08 and 0.0008. Winner squared-error and utility accumulators receive the nearest distance and nearest/second-nearest difference. With geometric strength `S_i`, forgetting and insertion use

\[
 E_i\leftarrow E_i(1-0.001/S_i^4),\qquad
 U_i\leftarrow 0.999 U_i,\qquad
 P_i=E_iS_i^4.
\]

Splitting occurs every 100 updates, selecting an adjacent pair by insertion priority. At capacity, eligible low-`U_i S_i` nodes can be recycled. Each update batch contains at most `floor(0.5*updates)` focused draws; the rest draw from the whole accepted cloud. This quota does not guarantee background coverage: globally sampled points may also lie in attended regions, and `S^4` can strongly concentrate the finite node budget.

### Unknown-node completion

`semantic_cluster_propagation.hpp` assigns an UNKNOWN node a class only when at least two direct graph neighbors within 0.055 m support exactly one class, with confidence at least 0.15. Updates are simultaneous, so inherited labels do not cascade within that call. Confidence becomes the mean support confidence. Existing labels and direct detector evidence are not overwritten. Although its header says “visual-only,” the propagated class vector is also passed into `publishEnvironmentGraphData`, so it can alter planning target-component membership. It does not remove any raw geometry from the complete collision scene or add direct attention evidence.

## 2. Object reference, identity, and collision geometry

Sources: `semantic_target_selection.hpp`; `reachability_graph_node.cpp::applyEnvironment`; `graph_pick_node.py::semantic_component` and `component_bounding_center`.

The typed planning view retains graph positions and edges but makes only the selected target class nonnegative. Other classes become `-1` in that planning view; the full semantic display is retained separately. Same-class connected components define object candidates. Their physical target reference is the **axis-aligned bounding-box midpoint of the observed component**:

\[
 c_C = \tfrac12\left(\min_{i\in C}p_i+\max_{i\in C}p_i\right),
\]

where minima and maxima are coordinatewise. This is not the arithmetic centroid, volumetric center, center of mass, or an inferred grasp contact. Occlusion and sparse/outlier nodes can bias it. A nearest observed graph-node ID is kept as a representative identity; its surface position does not replace `c_C`. C++ planner and Python preview currently use the same midpoint definition in live component-center mode.

Separately, tracked object clusters are grouped by current accepted detector instance (`node_detection_index`), and their arithmetic mean is smoothed for track association. Temporal/inherited labels do not become current direct detector-instance memberships. Same-class graph components and detector-instance tracks therefore need not be identical. Nearby same-class objects can merge in the graph or associate ambiguously.

With `include_target_in_collision=true`, collision geometry contains **every** input graph node and edge, including target and unknown nodes. Nodes become 12-mm-radius spheres and edges 6-mm-radius cylinders in a MoveIt scene. These are observed topology primitives, not watertight object meshes, occupancy volumes, or shape completion. Gaps and spurious bridging edges respectively permit false negatives and false positives. The cheaper capsule broad phase has a separate target-neighborhood exclusion; final geometry checking still uses the complete scene in the experiment launch.

## 3. Recorded configuration graph

Sources: experiment launch; `workspace_samples.hpp`; `reachability_graph_node.cpp::sampleWorkspaceNodes`, `connectRoadmapEdges`, `validatedStartNode`.

`ddgng_experiment_reachability.launch.py` selects CSV rows with `status==pose_found`, `pose_found==1`, and `position_found==1` for green mode. It snapshots the selected CSV and records source/snapshot SHA-256 digests. Blue near-singular rows are excluded. Read the scanner protocol before calling these physical demonstrations: the CSV contains IK/workspace witnesses, not necessarily measured robot trajectories.

Each node stores six-joint vector `q_i` and recomputed FK pose `(p_i,R_i)`. Model mismatch beyond 5 mm relative to CSV XYZ aborts import; bounds, self-collision, and duplicate configurations are checked. Identical XYZ can retain different joint configurations; exactly duplicated joint vectors are removed.

Candidate edges satisfy Cartesian and normalized-joint thresholds and the configured nearest-neighbor rule. The distances are

\[
 d_Q(q_i,q_j)=\sqrt{\sum_k((q_{ik}-q_{jk})/(q_k^{max}-q_k^{min}))^2},
 \qquad w_{ij}=\|q_i-q_j\|_2.
\]

Spatial buckets accelerate candidate discovery. Neighbors are ranked by normalized joint distance; edge cost uses the unnormalized Euclidean joint norm. Workspace edges are candidates, not measured motions and not learned DD-GNG adjacency. A measured start connects through a bounded ranked search with bounds/self-collision, capsule sweep, and interpolated mesh collision checks.

## 4. Pregrasp relation, planning objective, and an important bug

Source: `reachability_graph.hpp::pregraspIntersectionMask` and `planToNearestTarget`.

For **one selected component**, the implemented feasible goal condition is

\[
 0.07\le\|c_C-p_i\|\le0.13\;\mathrm m,
 \qquad
 (R_i e_x)^\top {c_C-p_i\over\|c_C-p_i\|}\ge0.70.
\]

`e_x=(1,0,0)` is the tool-local approach axis, transformed through FK. This permits up to approximately 45.6 degrees of alignment error. It does **not** constrain world-horizontal approach, a particular object side, roll/jaw closing direction, or target height. A downward-facing tool above the object can meet the condition. Changing arrow endpoints or replacing a surface point by component center does not make the robot approach horizontally. The manuscript cannot claim side-grasp enforcement without an additional task-frame direction/elevation constraint and tests.

### Confirmed scanner/planner axis convention discrepancy

Independent source inspection of `experiments/cartesian_workspace/cpp/scan.cpp:115` confirms that the workspace scanner specifies desired tip orientation through

\[
 A=\begin{bmatrix}0&0&-1\\0&1&0\\1&0&0\end{bmatrix},\qquad
 R_{tip}=R_z(\psi)R_y(\theta)R_x(\phi)A^\top.
\]

For the default V2 green bank, whose requested roll/pitch are zero and yaw is radial, this implies

\[
 R_{tip}e_x=(0,0,-1)^\top,\qquad
 R_{tip}e_z=(\cos\psi,\sin\psi,0)^\top.
\]

Thus **the bank requests downward local tip-X and horizontal radial tip-Z**. The later planner's choice `tool_approach_axis=+X` is not equivalent to the scanner's horizontal gripper convention. The screenshot's repeated overhead approach has a source-level explanation beyond a stale process. This derivation concerns desired orientations; validating each stored joint vector against the current deployed URDF requires a separate FK comparison. It does not establish which physical axis should be chosen without resolving the actual gripper/tool frame. Correct frame conventions and regenerate/revalidate the witness bank before claiming horizontal side-approach results. Merely drawing a horizontal arrow would hide this disagreement.

Dijkstra computes shortest graph costs from the connected start. Goal selection minimizes **target distance first**, then graph cost only for tied target distances. It is not a weighted sum of path length, grasp quality, and clearance. This nearest-first criterion can choose a long route toward a slightly closer goal.

**Current multi-target bug:** `pregraspIntersectionMask` produces a Boolean union over all component centers. `planToNearestTarget` later loops over target-node pairs but checks only the union mask and maximum radius. A node eligible for object A may then be selected for object B without satisfying B's orientation or minimum stand-off. A pairwise relation `M[i,C]` is the correct formulation, but is not currently implemented. Restrict claims and initial evaluation to one selected component, or repair and separately validate this before multi-object planning experiments.

Strict atomic query mode keeps the explicitly requested node and skips live component-center reduction. Benchmarks using that interface must state which target definition they evaluated.

## 5. Collision filtering and shortcut behavior

Source: `reachability_graph_node.cpp::bodySweepForTransition`, `exactTransitionIsValid`, `planWithExactValidation`, `shortcutValidatedPath`.

For each candidate edge, joint interpolation is sampled with

\[
 N=\max(1,\lceil\|q_j-q_i\|_\infty/\delta_q\rceil),\qquad
 q^{(s)}=(1-s/N)q_i+(s/N)q_j,
\]

including both endpoints. Capsule sampling uses `delta_q=0.08 rad`; FCL sampling uses `0.05 rad`. Robot collision geometry is the URDF/SRDF mesh model and environment geometry is the graph primitive scene. “Exact” in source names means detailed FCL geometry checks at sampled configurations; it is not continuous collision detection, a guarantee about between-sample motion, or a claim of perfect environment reconstruction.

The first rejected route edge is disabled and Dijkstra is repeated, up to an experiment rejection budget of 80. Failed-edge state is cleared when the environment scene is replaced. Budget exhaustion does not prove that no feasible path exists. It can also indicate sparse configuration witnesses, target relation gaps, or conservative/spurious graph geometry.

After a route passes, a greedy farthest-waypoint shortcut tests direct joint transitions with capsule and FCL checking. Accepted shortcuts can connect nodes with **no original roadmap edge**. Describe this as validated augmentation/shortcutting, not strict traversal of original learned edges. Cartesian waypoint lines in RViz are only a visual interpolation between FK samples; joint interpolation generally traces a curved EoE path. Neither smooth Cartesian motion nor acceleration/jerk optimality follows from shortening the waypoint list. Published `graph_cost` is not recomputed after shortcutting and should not be used as shortened-trajectory length without recalculation.

Collision exceptions matter: the configured fixed `link1` is ignored against the environment. The experiment launch additionally allows the `link2:link6` pair globally, based on a local mechanical-contact rationale; this is not a contact-depth-limited tolerance. Planner states set the arm group and default values for remaining joints; actual gripper aperture and payload collision geometry are not fully modeled. Claims of complete whole-robot collision avoidance must state these exclusions and modeling limitations.

## 6. Frozen preview, execution, and controller context

Source: `graph_pick_node.py`; defaults `config/graph_pick.yaml`.

The current experiment task is `move_to_target`: it sends a joint trajectory toward a pregrasp witness and leaves the gripper unchanged. It is not an end-to-end pickup. Optional `pickup` opens the gripper, traverses the trajectory, closes at that endpoint, and optionally reverses; no separate final Cartesian insertion/contact-grasp solver or grasp-success sensor is implemented. Retreat is disabled by default because payload-aware checking is absent. `_holding_object=true` after successful gripper action is software state, not measured object acquisition.

Preview construction checks accepted/fresh perception, matching environment and object-track history, named joints and trajectory shape, detailed-collision validation status, distance limits, measured start mismatch, calibration status, and execution-enabled state. The retained plan carries trajectory, graph identifiers, track/class, target geometry, fingerprints, and hardware health counters. Default preview lifetime is 5 s. Before explicit execution, the coordinator checks freshness/interlocks, rebuilds the live preview, rejects changed track/class, displacement beyond 2 cm, and changed motion fingerprint. Nominal trajectory timing uses `max(min_segment_time, max_joint_delta/0.20 rad/s)`.

These are useful rejection checks, but **not** a full time-indexed execution safety proof. Live environment matching is based on timestamp proximity and node identity, not a cryptographically identical scene. The fingerprints deliberately omit the first measured waypoint; `_execution_rejection` compares a new live plan's fingerprint but sends the frozen trajectory, without an explicit comparison of current joints to the original frozen first point. Thus unchanged roadmap waypoints can coexist with a changed initial segment. Evaluation should include this condition before claiming exact preview/execution equivalence. The moving robot also can diverge from a checked trajectory during execution.

The hardware combines autonomous position and teleoperation offset:

\[
 q_{cmd}=\operatorname{limit}(q_{auto}+\Delta q_{teleop}).
\]

Offset is sticky and rebased when a new autonomous goal is observed. Distinct command interfaces permit both controllers to remain active. This is implementation context, not a collision-preserving property: collision validation of `q_auto(t)` does not establish validity of `q_auto(t)+Delta q(t)`. A new paper must evaluate blended commands separately or scope its planning claims to the nominal trajectory. No physical safety guarantee is established by the additive design.

## Recommended manuscript statements

Use: “We construct a semantic environment graph and query a recorded configuration roadmap for collision-checked pregrasp motion toward the observed component center.”

Use: “Detailed collision checking is applied at discretized joint configurations against a sphere-and-cylinder approximation of the observed environment topology.”

Use: “The object remains collision geometry while its observed component midpoint supplies a target reference.”

Avoid: “The system grasps arbitrary objects,” “all motion follows learned graph edges,” “continuous collision-free trajectories,” “centroid equals true object center,” “world-X side approach is guaranteed,” “proven robot safety,” and “new dual-topology planner outperforms baselines” without new supporting implementation and trials.

## Minimal new evaluation needed

1. Semantic accuracy and completeness on bottle/keyboard box overlap, adjacent same-class objects, occlusion, and moving-camera sequences; separate direct, held, and inherited labels.
2. Fixed-budget DD-GNG attention ablation: target representation gains versus background-obstacle coverage loss, with per-source-frame provenance and stale-result tests.
3. Center target versus nearest surface node; report center stability/error, candidate feasibility, alignment, distance, path length, and planning latency.
4. Recorded green witnesses versus same-budget GNG/Halton alternatives under the **new** semantic scenes, with collision exceptions and sampling resolution held fixed.
5. Shortcuts on/off: recomputed joint and Cartesian path length, exact-check count, rejected edges, and time; no screenshot-derived path statistics.
6. Explicit side-approach and multiple-instance tests after pairwise goal eligibility is corrected; do not silently fold these fixes into a description of the current algorithm.
7. Preview revalidation and interruption tests using changed starts, moved objects, stale scenes, changed controller state, and additive offset perturbations. Hardware trials, if performed, need separate synchronized logs and measured outcome definitions.

The existing unit tests validate individual behaviors; accumulated test-count summaries and user screenshots are not manipulation success-rate data.
