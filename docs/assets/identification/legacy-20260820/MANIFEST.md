# Historical gravity/friction identification plots

These figures were copied from the Jetson AGX directory
`/home/kublab/om6dof_identification/plots/` on 2026-08-21. They are retained as
diagnostic evidence and as motivation for a new, traceable friction-identification
experiment.

## Important provenance limitation

The PNG metadata indicates that these plots were generated around 2026-08-20
02:07 JST with Matplotlib 3.5.1. The newest nearby fit file was produced later,
around 03:05, from `identification_20260820_030300.csv`. The plots therefore
must **not** be presented as figures from that newest fit unless they are
regenerated from a known dataset and script.

The historical identification used raw Dynamixel current ticks. One tick is
approximately 2.69 mA for the XM430 family. These units differ from the new
Mode 0 leader controller configuration, whose ROS `effort` values are expressed
in mA.

The model used for these figures also predates the latest URDF corrections to
off-chain payload mass and several centres of mass. Captions must say
"historical, pre-URDF-correction diagnostic". The figures are not final model
validation.

## Files

| File | Size (bytes) | SHA-256 |
|---|---:|---|
| `joint2_measured_vs_predicted.png` | 42,848 | `67ba8ceb72d62fba83afc041482f349bc2ca507173c9ea092ed023a5833976d2` |
| `joint2_residual_vs_velocity.png` | 48,964 | `bf03d7b0c6c21997be6cd379fab03ad3992b28614f3d8516de77038e775d8747` |
| `joint3_measured_vs_predicted.png` | 45,028 | `a3e059d55b3c4c800c1ad249d4f1710fb1f62d6f29739f74c90be992e91e751f` |
| `joint3_residual_vs_velocity.png` | 43,618 | `4824f56cf9ccce07723043eb79aba6b5a3a2af25828d298ab6d5b3e47f759d9c` |
| `joint5_measured_vs_predicted.png` | 40,908 | `b5ba243e12f8b6967c2c709f13d9e995c1d8d0e0eaddc9d53076e855c0bfc04c` |
| `joint5_residual_vs_velocity.png` | 44,029 | `5db3220efde8183ea3370ad6864e0591607717634d861a32ebe7979b0b91b4da` |

All six files are PNG, 880 × 440 pixels, RGBA.

## Safe interpretation

- Measured-versus-predicted plots show that the historical fit did not explain
  every baseline and pose-dependent effect, especially for J3 and J5.
- Residual-versus-velocity plots show branches, hysteresis-like structure, and
  broad residuals near zero speed. These observations motivate investigation
  of stiction, Coulomb/Stribeck friction, velocity dead zones, and data timing.
- These patterns are hypotheses, not proof of a particular friction law.

## Required before publication

Regenerate the figures from a versioned script and immutable dataset. Record:

1. dataset filename and SHA-256;
2. script path and Git commit;
3. robot configuration, payload, supply voltage, and temperature;
4. sample rate and filtering;
5. current units and joint sign convention;
6. train/validation split and fit metrics;
7. URDF commit and dynamics parameters;
8. figure author and publication license.
