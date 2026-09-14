# Green roadmap search budget diagnosis

Saved bottle scene, recorded joints, same green-only 3198-node dataset and collision settings.
Isolated ROS domain 71; reachability node only. No robot/camera/controller/action started.
No production source or parameters changed.

Baseline from prior replay: exact_max_replans=20, fails after 21 rejected candidates.
Diagnostic-only override exact_max_replans=200: succeeds after 147 rejected candidates.
Validated graph path: 8 recorded nodes, 7 existing edges; target distance 0.038859063 m.
9055 exact state checks; planning cycle 951.529 ms.
Reason: path_ready_exact_validated_target_protected_preview_only.
Offline coordinator fixture (synthetic track metadata, not live validation) accepts 9 points including measured start;
only execution_disabled_at_launch blocker.

Code findings: applyEnvironment resets exact_blocked_edges on every applied scene.
planWithExactValidation stops at first failed edge per candidate, blocks it and reroutes.
Thus the baseline failure on this fixed scene is search-budget exhaustion, not absence of a collision-validated path.
Live intermittent results may additionally depend on scene/joint changes; not proven from screenshots.

Next targeted improvement: reduce repeated checks within a frozen query through scene-scoped validity caching,
and eliminate all incident edges of a node only when that node itself is proven colliding.
Keep full robot collision, target protection and green-only sample selection intact.
