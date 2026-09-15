# New-system evaluation plan — proposed, not run

## Research questions

1. Does targeting the observed cluster center reduce cap/edge target selection
   and target-reference jitter relative to an individual labeled node?
2. How much does correct pose/frame eligibility reduce invalid approach choices
   at a matched witness and search budget?
3. Does semantic DD-GNG attention improve object representation while preserving
   enough background geometry for collision-sensitive planning?

## Prerequisites for interpretable results

- Measure the physical approach and jaw-closing axes relative to the URDF TCP.
  The archived scan uses gripper +X = TCP +Z. State the chosen convention and
  regenerate/revalidate witnesses if a different approach policy is intended.
- Preserve an eligibility relation per `(robot node, object component)`. The
  current union mask must not allow an A-eligible node to become a B-goal without
  checking B's orientation and minimum distance.
- Put the authoritative center, target identity/membership, scene/model revision,
  tool-axis convention, and exact frozen joint trajectory in the query/response.
- Check current joints against the frozen trajectory's first point immediately
  before transmission; a rebuilt live plan is not identical by definition.
- Model gripper aperture and any payload consistently. Document the configured
  `link2:link6` and base/environment collision exceptions.
- Distinguish a stand-off endpoint from contact insertion, closure and lifting.

These changes are requirements to evaluate; creating this paper did not implement
or execute them.

## Matched offline study

Freeze RGB-D/world-pose observations, object labels used as annotation, start
states, gripper state, URDF, method revision and source hashes. Replay identical
inputs for every policy. Reset adaptive learner memory identically per scene.
Use scene resets as the independent replication unit, not adjacent frames.

| Policy | Target reference | Pose eligibility |
|---|---|---|
| A | Single labeled surface node | Distance only |
| B | Observed component AABB center | Distance only |
| C | Component center | Correctly expressed physical tool axis + stand-off |
| D | Component center | C + world-frame side-approach cone |

Hold candidate configurations and collision rules fixed for A–D. Separately
compare the orientation-specific bank with a bank containing multiple poses per
position; otherwise a bank change would be confounded with the target policy.
For attention, compare the same DD-GNG core and node/update budget with neutral
strength versus semantic attention. Include a conventional MoveIt/OMPL planner
with identical start, terminal region/orientation and collision scene.

Scenes should include bottle-only, bottle/keyboard overlap, occlusion, two bottles,
and genuine geometric obstacles, across different starts and target heights.
Choose trial count after a development pilot/power analysis. Freeze the held-out
scene list and tolerances before comparing outcomes; no arbitrary “N trials”
is presented as an already collected dataset.

## Measurements and failure attribution

- Segmentation/association: labeled-node precision/recall against independent
  annotations; class/component merges; direct/held/inherited label contribution.
- Reference geometry: center displacement and repeatability against measured or
  annotated reference; uncertainty due to partial views reported explicitly.
- Reachability: eligible target-node pairs and connection from measured start.
- Planning: complete query success and reasons for rejection, sampled checks,
  elapsed time, recomputed joint length, densely sampled FK path length.
- Approach: local/world axis directions, alignment, height error, stand-off and
  actual finger clearance. Colors or arrow direction alone are not a metric.

Do not infer shortest-path improvement from stale `graph_cost` after shortcutting;
recompute cost on the shortened trajectory. Report all failures in the total
denominator and both unconditional and stage-conditional rates. Use paired
scene-reset bootstrap intervals (resample full resets, not frames). Timing claims
require controlled load and separate perception/planning execution-time measures.

## Physical study, after the offline contract is checked

Record synchronized joint states, commanded reference and offset, TF, RGB-D,
scene/plan IDs, gripper position/effort and external video. Evaluate reaching,
contact-aware final approach, closure, lift and placement separately. Define
lift success before collection (e.g. a measured 50 mm lift and 3 s hold).
Treat teleoperation as a recorded intervention with a separate condition, because
it changes the checked nominal trajectory. Hardware actions require their own
explicit task authorization; this paper build performs none.

## Tables to populate from future collected trials

| Condition | Independent scene resets | Valid plan / attempts | Approach reached | Lift successes |
|---|---:|---:|---:|---:|
| New system, controlled scenes | Not collected | Not measured | Not measured | Not measured |

This table is a collection template and is intentionally absent from the paper's
reported numerical results. Legacy frozen-query results, instrumentation smoke
recordings, screenshots, and code-test counts cannot populate it.
