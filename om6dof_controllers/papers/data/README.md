# Physical multisine data used in the manuscript

The figures and metrics in this directory are derived from the physical
`teaching_all` run, not from the legacy pre-URDF-correction identification
logs.

- Raw source: `../experiments/multisine_identification/data/multisine_teaching_all.csv`
- SHA-256: `253edb8b51532a5b9553a868cf5514fdaf4ae72f4818de83c0aab79011883161`
- Raw samples: 18,156 over 181.053 s (approximately 100.27 Hz)
- Recorded signals: position, reported velocity, and DYNAMIXEL Present Current
  in raw register units for all six joints.

The analysis uses every tenth row, discards the first/final 2 s, trains on
`[2, 120)` s, leaves `[120, 122)` s as a guard interval, and evaluates on
`[122, 179.33)` s. `Tg` is an offline whole-URDF potential-energy prediction;
it is not a torque-sensor measurement. Present Current is raw DYNAMIXEL data,
not direct joint torque.

Regenerate the figures and summaries with:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
python3 ~/ros2_ws/src/om6dof/om6dof_controllers/papers/scripts/generate_multisine_results.py
```

The generated PDF figures are in `../figures/`; the manuscript includes them
directly. Do not mix the results with `~/om6dof_identification` legacy logs,
which predate the current URDF/branch-mass correction.
