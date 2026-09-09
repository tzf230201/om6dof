# OM6DOF workspace demonstrations

- [V1 / V2 comparison](https://tzf230201.github.io/om6dof/comparison/comparison.html)
- [Original V1 3D viewer](https://tzf230201.github.io/om6dof/viewer3d.html)
- [Original V1 2D report](https://tzf230201.github.io/om6dof/)

The matched comparison uses 33,401 XYZ samples on a 25 mm grid inside a 500 mm
sphere, with the same radial orientation policy and IK search settings.
V2 finds 3,532 complete poses versus 2,935 for V1 (+20.34% relative), which is
why V2 is the selected direction for the next mechanical revision. Position
coverage decreases slightly, and near-singular pose witnesses increase.
These are sampled kinematic results, not verified physical performance or
full-mesh collision/path tests.

The original 31,799-point V1 demo remains at the root. Only navigation links
were added there; its data and analysis were not replaced. The comparison
snapshot lives under `comparison/` and does not need ROS or a robot connection.

Source: [OM6DOF main, publication commit 086c6fb](https://github.com/tzf230201/om6dof/commit/086c6fb).
Reproduction instructions and model hashes are in the source repository's
`experiments/cartesian_workspace/ANALYSIS_V1_V2_25MM_20260909.md`.
The static website excludes URDF/STL models and numerical seed banks.

Original viewer/report software is Apache-2.0; see LICENSE and NOTICE.
