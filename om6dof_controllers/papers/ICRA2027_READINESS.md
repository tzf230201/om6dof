# ICRA 2027 readiness plan

The current manuscript is a double-anonymous technical draft, not yet a
submission. ICRA 2027 requires a double-column PDF of at most eight pages,
including references, and uses double-anonymous review. The initial technical
paper submission window is 16 July--15 September 2026. A supporting video must
be submitted in the initial submission window; a video added later is not
accepted.

## What is already supported by the repository

- URDF/KDL gravity computation with distal branch-mass handling.
- Manually bounded multisine collection and current-domain least-squares
  fitting. This is not yet autonomous safe experiment synthesis.
- One chronological held-out analysis of `multisine_teaching_all.csv`:
  18,156 raw samples over 181.05 s; J2/J3/J5 test (R^2) of 0.869/0.952/0.956.
- A qualitative physical observation that Mode 5 is stiff and signed Mode 0
  current is more backdrivable. It is not yet a reproducible comparison.
- A preliminary Mode 0 joint-2 snapshot: $g_2=0.1090$ Nm, command 19.38 raw
  ticks, and present current 18 ticks. This verifies current-command tracking,
  not physical gravity torque or current-to-torque scale.

The repository does **not** yet implement autonomous collision-safe pose
generation, paired polarity verification, SVD observability analysis,
information-optimal trajectory generation, known-payload torque-scale
calibration, validated friction sweeps, automatic acceptance gates, or
cross-morphology recommissioning.

## Non-negotiable evidence before submission

1. Archive the exact expanded URDF, controller YAML, firmware/model/ID register
   dumps, supply voltage, payload configuration, code commit, and raw logs.
2. Numerically verify the offline expanded-tree and deployed KDL gravity
   implementations over the logged configuration set.
3. Implement and test URDF/world-derived collision-safe candidate generation,
   including joint, velocity, acceleration, clearance, current, temperature,
   tracking-error, and abort constraints.
4. Implement paired current-polarity verification under mechanical support;
   archive every accepted/rejected pulse and threshold decision.
5. Calibrate observable raw-current/torque scales using a measured payload with
   known attachment and CoM. Use an external torque reference for axes that the
   gravity payload cannot excite.
6. Generate a normalized/whitened gravity regressor, report its SVD/rank, and
   compare manual multisine, safe-random, and information-optimized experiments.
7. Repeat the chronological train/test current-model analysis on independent
   trajectories, then report RMSE/MAE and confidence intervals at trial level.
8. Run matched bidirectional constant-speed sweeps, excluding acceleration and
   stiction regions, before enabling any friction term.
9. Compare Mode 5 and Mode 0 at identical poses and current safety limits.
   Quantify drift, activation displacement, current, and handle force. A load
   cell is required for a force claim.
10. Demonstrate the same procedure on modular 2-, 3-, 4-, and 6-DOF
    descriptions, and test payload-change detection and recommissioning.
11. Perform a pose grid and known-payload test; report all saturations, faults,
    temperature, minimum collision distance, abort latency, and rejected trials.
12. Record an anonymized, 180-second-or-shorter MP4 showing safe mode change,
   activation, smooth hand guiding, release behavior, and fault/deactivation.
13. Replace all placeholder author information only after the review version is
   accepted. Do not include laboratory, repository, video, or self-citing
   information that identifies the authors in the review PDF.

## Claim discipline

Use ``proposed framework'' for the full commissioning loop and ``preliminary
OM6DOF instantiation'' for current results. Do not claim that autonomous
commissioning, robot-agnostic scalability, accurate torque control, exact
mass/CoM recovery, validated friction, transparent interaction force, formal
safety, generalization, or superiority has already been demonstrated.

The paper must separate current-domain compensation from physical parameter
identification. The fitted `c_g` is raw current per nominal-model Nm and absorbs
both actuator scale and URDF inertial error; it is not a motor torque constant.
Exact link masses and CoMs are generally not individually observable. Physical
units require a known-load or external-torque calibration and an explicit
base-parameter observability analysis.

The existing multisine data were recorded with the normal trajectory/position
servo active, whereas deployment uses Mode 0 signed current. Treat the fit as
an effective current-response calibration until a Mode-0 validation confirms
the transfer. Also report the distinction between the offline whole-URDF
potential-energy gravity implementation and the deployment KDL chain with
folded branch masses; their numerical agreement has not yet been verified.

## Related-work positioning

The closest technical predecessors are Chen et al. (2018) and Chen et al.
(2023): both use current/torque-mode zero-moment teaching with gravity and
friction identification. The OM6DOF paper must not claim the first pure-current
teaching controller, first self-measured gravity/friction method, or a new
gravity--friction control law.

FIGAROH, AURT, classical base-parameter/optimal-excitation methods, backward
sequential identification, and safe online system identification already cover
many elements of URDF-driven calibration. The paper must not claim the first
URDF-to-identification workflow, automatic excitation, base-parameter method,
or distal-to-proximal identification.

The defensible research target is the hardware-facing transition from a robot
description to raw-current direct teaching: actuator capability and polarity
qualification, explicit current/physical-scale ambiguity handling,
observability-guided staged gravity/friction experiments, and a validation-
gated control-mode handoff. Whether that integration is sufficiently novel must
be supported by the completed implementation and comparisons, not asserted as
a first.

## Generative-AI disclosure

ICRA 2027 states that AI-generated article content must be disclosed in the
acknowledgments, identifying the system, affected sections, and its use. This
draft was generated with Codex and must be fact-checked, rewritten by the
authors as appropriate, and disclosed according to the final conference policy.
