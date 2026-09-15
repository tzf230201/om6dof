# Implementation audit and claim boundaries

Date: 2026-09-10. Source inspected: `native_fgng/include/fgng_core.hpp` and
`native_fgng/src/main.cpp`. This document records observations, not benchmark results.

## What the original implementation already does

- `buildPoints` backprojects depth pixels to metric camera-frame XYZ using
  `(u-cx)z/fx, (v-cy)z/fy, z`. The input to GNG is already three-dimensional.
- Sampling stride increases with retinal eccentricity. Sample weights `step^2`
  compensate unequal image-space sampling rates, not equal surface-area sampling.
- Each batch assigns first and second winners, moves prototypes, rebuilds edges,
  computes density demand, and applies hysteretic merging and splitting.
- Attention is a hyperbolic field in image coordinates, optionally depth weighted.
- The core has persistent-within-reset integer IDs, alias resolution, and optional
  semantic labels. The camera application does not provide semantic labels.

## Why a moving camera requires more than a coordinate transform

1. Frame registration: before aggregation, all samples need
   `p_world = R_world_camera * p_camera + t_world_camera` at the correct timestamp.
2. Historical support: the original `accumulateBatch` resets winner mass and edge
   strength every batch. Edges are rebuilt only from the current batch and isolated
   nodes can be removed. A static surface outside the new field of view can therefore
   lose support even after perfect frame registration.
3. Observation versus disappearance: being outside a frustum, occluded, or missing
   valid depth is not evidence that a surface has disappeared. The implemented
   static-scene replay extension preserves samples without claiming a dynamic
   object-deletion model.
4. Attention coordinates: a persistent world-space target avoids accidental target
   motion caused solely by camera motion. It is a distinct choice from retinal
   attention in the original live viewer and must be documented as such.

## Proposed experiment boundaries

- Use externally supplied ground-truth camera poses. This is a conditional
  reconstruction/representation benchmark, not a SLAM evaluation.
- Hold out disjoint depth pixels from graph learning. Accumulate reference points
  causally. Such points come from the same sensor and poses; they are not an
  independent surveyed mesh or exact world ground truth.
- Report node budget, replay budget, retained support count, actual node count,
  and measured update time separately. Equal node limits do not mean equal total
  memory or equal processing time.
- Do not infer robot task success, collision safety, topological guarantees, or
  material-point tracking from nearest-neighbor reconstruction metrics.
- World-memory versus world-only isolates history support. Attention versus
  uniform attention with the same memory isolates attention. DBL-GNG with the
  same memory is an algorithm comparison. A camera-frame original is a diagnostic
  baseline for the original implementation, not a competitive registered mapper.

## Known inherited limitations

- Dense graph arrays scale quadratically with maximum node count.
- The original retired-ID set can grow over time. Its comment claiming no
  post-construction allocation is not accurate for all dynamic containers.
- `reset` restarts ID numbering; IDs are not globally unique across reset sessions.
- Persistent IDs identify evolving prototypes, not tracked physical surface points.
- Hysteresis pressure decays toward zero while in balance; it does not require
  strictly consecutive threshold violations in all cases.
- The local complexity term is dispersion of edge directions,
  `1 - lambda_max / trace`; it is not an estimate of surface curvature.
- A positive attention floor is not a minimum geometric-coverage guarantee.
- Learned adjacency may bridge disconnected surfaces. Edge support is evaluated
  separately; nearest-node error alone does not establish correct topology.

## Scientific reporting

Use empirical results exactly as observed, including negative comparisons. Keep
profiling outside disk I/O/evaluation clearly labeled; do not call it end-to-end
latency. All figures must derive from recorded arrays or be explicitly labeled
schematics. Submission readiness additionally requires human scientific review,
author approval, verified references, and final format checks.
