# Matched 50-seed reachability benchmark

## Protocol and validation

- 50 deterministic paired seeds per method (100 roadmap builds).
- Equal 800-node budget; seed-specific Halton offset shared by GNG and Halton/PRM.
- Method order counterbalanced within consecutive seed pairs.
- Identical start configuration, SRDF home target, and world-frame obstacle for both methods.
- Clear and dynamic queries are preview-only and exact-validated; no controller action is created.
- Raw rows: 100; process errors: 0.

## Primary results

- Connected components: GNG mean 1.70 versus Halton/PRM 11.82; mean paired reduction 10.12 (95% paired-bootstrap CI 9.22 to 11.02), rank-biserial r=1.000, Holm-adjusted p=3.59e-09.
- Build time: GNG mean 1723.78 ms versus Halton/PRM 1413.57 ms; paired difference (Halton-GNG) -310.20 ms (95% CI -321.85 to -297.56), r=-1.000, Holm-adjusted p=3.59e-09.
- Clear planning time: GNG mean 3.25 ms versus Halton/PRM 3.39 ms; paired difference 0.14 ms (95% CI -0.15 to 0.45), r=0.093, Holm-adjusted p=0.566.
- Dynamic planning time among seeds where both methods succeeded (n=42): GNG mean 22.19 ms versus Halton/PRM 25.09 ms; paired difference 2.89 ms (95% CI -0.44 to 6.73), r=0.231, Holm-adjusted p=0.383.
- Dynamic success: GNG 42/50 versus Halton/PRM 50/50; paired risk difference -16.0 percentage points (95% CI -26.0 to -6.0), exact McNemar Holm-adjusted p=0.0234.

## Interpretation boundary

The time comparison for the dynamic condition is conditional on both methods succeeding and must be reported together with the separate paired success analysis. Seed-level repetitions quantify roadmap-construction variability for this fixed robot, target, and obstacle; they do not establish generalization across object classes, workspace scenes, or physical executions. Background hardware/perception processes were active on the AGX during measurement, while benchmark ROS traffic remained isolated in domains 20-119. Method order was counterbalanced to reduce monotonic warm-up/thermal bias.
