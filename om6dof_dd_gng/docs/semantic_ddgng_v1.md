# Semantic-attention DD-GNG environment: v1

## Two independent graphs

- Environment: aligned RGB-D -> metric camera points -> timestamped world TF ->
  robot-body masking -> online DD-GNG; YOLOX provides depth-supported node labels
  and world-coordinate attention regions for the next update.
- Robot action area: the existing ordinary-GNG reachability roadmap, unchanged.
  Environment graph nodes/edges keep the existing typed ROS interface and stable
  node IDs. Geometric overlap alone is not collision-free trajectory execution.

## What is implemented

This is a bounded semantic-attention adaptation of the strength mechanisms in
[Saputra, Botzheim and Kubota (2023)](https://www.mdpi.com/2075-1702/11/6/619),
not a bit-exact replication of their terrain-attention system. The old
`realsense_ddgng/core/gng.cpp` is an additional implementation reference, not
the active C++ world-mapping backend.

Each directly supported semantic node proposes a world-space sphere, radius
0.05 m, with strength S = 1 + (Smax - 1) * score (default Smax = 3).
The score is the existing detector/depth/visibility/box-centre fusion score,
not a calibrated probability. Greedy selection is deterministic by descending
score then node index, separating centres by more than half a radius, capped at
64 regions. Overlapping regions use maximum strength rather than addition.
An unlabeled point has S = 1. This focuses all accepted target classes; it does
not choose the next grasp or infer instance identity.

At each update:

1. Draw at most floor(updates * attention_sample_fraction) samples from points
   in active attention regions (default fraction 0.5). Randomly interleave
   these with whole-cloud uniform draws. With no focused points, use only
   whole-cloud draws. This is a sample-budget guarantee, not a coverage theorem.
2. Find two nearest nodes; update winner error/utility, adapt winner and
   neighbors (0.08 / 0.0008), connect winners, age edges (maximum age 50).
3. Recompute node strength geometrically after movement. Forget error using
   E <- E * (1 - 0.001 / S^4); forget utility using U <- 0.999 U.
4. Every 100 updates, select a connected split node and neighbor using E*S^4.
   At capacity, recycle an eligible low-U*S node, retaining the split pair.
   This soft utility protection, max-strength overlap, continuous semantic
   score, always-weighted splitting and bounded mixture are adaptation choices,
   not claims to reproduce the original algorithm exactly.
5. Maintain stable, never-reused IDs within an epoch, finite coordinates,
   undirected edges and a hard node limit (default 500; valid range 2..4096).

Reset clears graph, attention, counters and starts a new ID epoch. The core uses
seed 7; repeatability is within the same compiled implementation/standard-library
environment, not a promise of bit-identical cross-platform random distributions.
Strength=1 with attention fraction=0 is a same-core ablation, not an assertion
that every detail equals the previous vendored learner.

## Freshness and camera motion

YOLOX submission remains at most once per 0.5 s, with one inference worker.
Slow inference can reduce the realized rate below 2 Hz.
Each submission records its input sequence, monotonic host-receipt time and
camera-to-world transform. A result is usable only when its source is known,
source age <=1.0 s, translation since source <=0.02 m, and rotation <=0.10 rad.
Otherwise its boxes are withheld from semantic association.

Association retains the existing current-depth visibility and box-depth gates.
Only direct, accepted matches create attention, never temporally held labels.
A new empty result clears regions. A cached result cannot refresh their TTL,
follow moved nodes, or resurrect cleared regions. Attention expires from the
source image receipt time, NOT inference completion time. It affects the next
learning update; expiration is checked before learning.

These are bounded stale-result guards, not exact RGB/encoder synchronization,
motion-compensated detection, or dynamic-object tracking. Small motion inside
the thresholds and moving objects can still cause association error. Host
receipt is not the physical exposure timestamp. Hand-eye calibration and
live camera evaluation remain necessary.
The pre-existing semantic display evidence can still be reinforced when a
cached result is reused; it is not independent-inference confidence. Density
attention is separately deduplicated and cannot be prolonged by that evidence.

## Configuration and observability

Both standard C++ YAML profiles set environment_graph_method: dd_gng.
Density and semantic guard parameters are startup-only; restart the perception
node to change them. Reachability graph_method: gng is unchanged.

Status and per-frame perception metrics identify environment_algorithm /
density.algorithm as semantic_dd_gng_v1. Additive fields in the existing v1
metrics schema include yolo.source_age_ms, yolo.geometry_usable and:

- density.active_attention_regions / focused_nodes: current learning snapshot;
- density.focused_samples_cumulative / uniform_samples_cumulative: draw counters;
- density.attention_regions_next_frame: attention after semantic fusion.

Whole-cloud draws may land inside attention regions. Raw detections counts
remain detector output counts; geometry_usable indicates whether boxes may be
used. Existing gng_update and semantic_fusion timing fields include density
selection/expiry and region construction respectively. These are wall durations,
not isolated CPU or heap-allocation measurements.

## Reproducibility and evidence boundary

New C++ regression tests cover graph invariants, deterministic/reset behavior,
nonfinite inputs, region validation, sampling budget, spatial strength, semantic
TTL/sequence handling, empty results, and synthetic attention switching at
capacity. Synthetic density tests compare identical seeded point clouds and
update budgets against neutral strength in this same new core.
The S^4 priority can concentrate most nodes in a small attended area even
with half the samples drawn globally. Evaluate background/obstacle coverage
and false-positive sensitivity before any physical-robot performance trial;
neither sampling quota nor graph edges certify collision safety.

Existing frozen catalogs, recorded configs, benchmark results, reachability
source and vendored core are not changed. Historical metrics must retain their
original algorithm provenance: the former C++ environment learner had no
active semantic density feedback. New camera runs and new end-to-end benchmarks
are required before the paper can claim semantic_dd_gng_v1 performance.

Build/test without starting perception, controllers or robot motion:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select om6dof_dd_gng
colcon test --packages-select om6dof_dd_gng --event-handlers console_direct+
colcon test-result --verbose
```
