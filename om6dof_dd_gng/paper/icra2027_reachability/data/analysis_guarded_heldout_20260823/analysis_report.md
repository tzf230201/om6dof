# Guarded-GNG held-out 50-offset analysis

## Validation

- 150 roadmap builds: 50 paired held-out sample-stream offsets (50--99) for each of pure GNG, guarded-GNG, and Halton/PRM.
- Equal 800-node output budget and identical anchored start, target, midpoint point obstacle, and offset-specific stream position. Candidate-information and compute budgets are not equal.
- Every permutation of the three method orders was cycled; position counts: {'gng': [18, 16, 16], 'guarded_gng': [16, 17, 17], 'halton_prm': [16, 17, 17]}.
- Process errors: 0; clear-scene successes: 150/150.

## Development-only selection

- On the eight original pure-GNG failure offsets, success increased from 0/8 at 0% guard to 1/8, 4/8, 7/8, and 8/8 at 10%, 25%, 50%, and 75% guard.
- The 75% value was fixed before the held-out run. These eight offsets are regression/development evidence only.

## Held-out results

- Dynamic exact-valid success: pure GNG 46/50 (92%), guarded-GNG 49/50 (98%), Halton/PRM 49/50 (98%). Cochran Q p=0.223.
- Guarded-GNG versus pure GNG: paired risk difference 6.0 pp (95% paired-bootstrap CI -2.0 to 14.0); exact McNemar Holm p=0.75.
- Guarded-GNG versus Halton/PRM: paired risk difference 0.0 pp (95% CI -6.0 to 6.0); exact McNemar Holm p=1.
- Mean connected components: pure GNG 1.48, guarded-GNG 12.08, Halton/PRM 12.14.
- Mean validated edges: pure GNG 4540.5, guarded-GNG 3618.2, Halton/PRM 3255.7.
- Mean build time: pure GNG 1739.9 ms, guarded-GNG 1631.7 ms, Halton/PRM 1407.3 ms.
- Dynamic timing is conditional on success and therefore secondary: means were 21.0, 23.5, and 27.6 ms, respectively.

## Interpretation

Guard sampling reduced the fixed-query held-out pure-GNG failure count from four to one, but the paired improvement was not statistically significant at n=50 and guarded-GNG also lost pure GNG's low-component-count characteristic. Guarded-GNG matched Halton/PRM's 49/50 point success rate, with different failed offsets, but did not establish superiority. The result motivates a target-robustness--connectivity hypothesis and query-time multi-anchor or topology-repair methods. It does not justify claiming general reachable-area coverage or that guard sampling solves obstacle-robust reachability.
