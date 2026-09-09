# OM6DOF v1 / v2 matched workspace comparison

33401 identical XYZ samples, 25 mm grid, 500 mm sphere, radial orientation. Both models were rerun; this is not a comparison against the historical public snapshot.

| Result | v1 | v2 | Difference (percentage points) |
|---|---:|---:|---:|
| Position found | 15532 (46.50%) | 15175 (45.43%) | -1.07 |
| Pose found (including near-singular) | 2935 (8.79%) | 3532 (10.57%) | 1.79 |

| Metric | Both found | v1 only | v2 only | Neither found |
|---|---:|---:|---:|---:|
| Position | 14391 | 1141 | 784 | 17085 |
| Pose | 2829 | 106 | 703 | 29763 |

## Interpretation and limits

- Unresolved is not proven unreachable: the finite multi-start IK search can miss solutions.
- Only one target orientation per XYZ; not all rotations in SO(3).
- Collision checks use chain capsules, not the URDF triangle meshes. The new link3 mesh, D435 camera/bracket shape, mass and CoM are not evaluated by this experiment.
- This compares the URDF chain transforms and joint limits, not payload dynamics, load capacity, positioning accuracy or safe paths. No table/floor is modelled.
- Near-singularity belongs to the selected witness configuration, not every solution at XYZ.
- Same seed and search budget do not imply the same numerical seed bank across different models.
- Counts are discrete sample coverage, not exact continuous workspace volumes.

Open [comparison.html](comparison.html), [v1 report](v1/index.html), or [v2 report](v2/index.html). Settings and limits are preserved in [comparison.json](comparison.json).
