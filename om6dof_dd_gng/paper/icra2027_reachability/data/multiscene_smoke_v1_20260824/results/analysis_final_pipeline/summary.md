# Multi-scene reachability benchmark summary

**SMOKE / DESCRIPTIVE ONLY.** Confidence intervals and p-values are exploratory diagnostics, not confirmatory evidence.

Audit: **PASS** — 18 graph builds and 216 paired phase queries; no timeout or infrastructure-error rows.
Timing context: `loaded-system-integration-smoke`.

## Per-method descriptive results

| Method | Clear success | Dynamic success | Conditional retention | Path change on joint-success | Build ms (median [IQR]) |
|---|---:|---:|---:|---:|---:|
| gng | 50.0% [38.9, 61.1]% | 47.2% [36.1, 58.3]% | 94.4% | 64.7% | 1711.7 [1708.6, 1722.1] |
| guarded_gng | 72.2% [55.6, 88.9]% | 72.2% [55.6, 88.9]% | 100.0% | 34.6% | 1574.0 [1572.1, 1589.1] |
| halton_prm | 77.8% [66.7, 88.9]% | 77.8% [66.7, 88.9]% | 100.0% | 28.6% | 1382.8 [1368.0, 1393.1] |

| Method | Clear planning ms* | Dynamic planning ms* | Clear path cost* | Dynamic path cost* |
|---|---:|---:|---:|---:|
| gng | 5.669 [5.229, 5.834] | 11.349 [9.080, 21.808] | 10.974 [10.414, 12.368] | 12.368 [11.252, 14.744] |
| guarded_gng | 8.001 [6.050, 9.719] | 15.858 [10.340, 18.720] | 16.697 [14.079, 18.379] | 16.719 [14.079, 18.379] |
| halton_prm | 6.366 [6.089, 7.041] | 13.012 [9.557, 18.536] | 15.145 [14.172, 17.530] | 16.243 [14.283, 18.104] |

*Planning-time and path-cost summaries include exact-valid paths only. Values are median [IQR].*

| Method | Clear publish-to-plan ms (all outcomes) | Dynamic publish-to-plan ms (all outcomes) |
|---|---:|---:|
| gng | 6.023 [3.607, 8.422] | 15.262 [10.268, 19.230] |
| guarded_gng | 9.562 [6.049, 12.284] | 16.929 [11.338, 21.092] |
| halton_prm | 8.767 [7.289, 9.635] | 12.950 [10.293, 19.057] |

## Prespecified primary endpoint

The endpoint is dynamic exact-valid success risk difference over the frozen catalog. Effects are guarded GNG minus baseline. The primary CI resamples whole roadmap streams only; the p-value uses a studentized paired stream-level method-label permutation. Holm correction covers exactly the two prespecified contrasts.

| Comparison | Risk difference | Fixed-catalog CI | Permutation p / Holm p | Sign dominance* |
|---|---:|---:|---:|---:|
| guarded_gng − gng | 25.0% | [2.8, 47.2]% | 0.2500 / NA | 1.000 |
| guarded_gng − halton_prm | -5.6% | [-33.3, 22.2]% | 1.0000 / NA | 0.000 |

*Sign dominance is (positive streams − negative streams) / non-tied streams; it is not a rank-biserial effect.*

The two-way stream × base-trajectory bootstrap is reported only as a scene-generalization sensitivity in `pairwise.csv` and `analysis.json`; it is not the primary fixed-catalog CI.

## Descriptive catalog strata

These subgroup estimates are descriptive and are not multiplicity-tested.

| Dimension | Level | Scenes | GNG | Guarded GNG | Halton PRM | Guarded−GNG | Guarded−Halton |
|---|---|---:|---:|---:|---:|---:|---:|
| obstacle_kind | point | 3 | 50.0% | 72.2% | 77.8% | 22.2% | -5.6% |
| obstacle_kind | segment | 3 | 44.4% | 72.2% | 77.8% | 27.8% | -5.6% |
| difficulty | low | 2 | 100.0% | 100.0% | 100.0% | 0.0% | 0.0% |
| difficulty | medium | 2 | 25.0% | 66.7% | 33.3% | 41.7% | 33.3% |
| difficulty | high | 2 | 16.7% | 50.0% | 100.0% | 33.3% | -50.0% |
| difficulty_x_obstacle_kind | low:point | 1 | 100.0% | 100.0% | 100.0% | 0.0% | 0.0% |
| difficulty_x_obstacle_kind | low:segment | 1 | 100.0% | 100.0% | 100.0% | 0.0% | 0.0% |
| difficulty_x_obstacle_kind | medium:point | 1 | 33.3% | 66.7% | 33.3% | 33.3% | 33.3% |
| difficulty_x_obstacle_kind | medium:segment | 1 | 16.7% | 66.7% | 33.3% | 50.0% | 33.3% |
| difficulty_x_obstacle_kind | high:point | 1 | 16.7% | 50.0% | 100.0% | 33.3% | -50.0% |
| difficulty_x_obstacle_kind | high:segment | 1 | 16.7% | 50.0% | 100.0% | 33.3% | -50.0% |

All secondary effects, including build/planning time and path cost, are descriptive and retained in `pairwise.csv` and `analysis.json`. Planning and path-cost differences use only common exact-success cells; path-change uses common four-way clear/dynamic joint-success cells.

## Statistical scope

- The primary estimand averages within-stream dynamic exact-valid risk differences over the frozen scene catalog, then averages over roadmap streams.
- The primary bootstrap resamples complete roadmap streams and never resamples catalog scenes.
- Both primary contrasts use the same deterministic sequence of resampled roadmap-stream indices.
- Scene-level McNemar tests are intentionally not reported because repeated scenes within a stream are not independent pairs.
- Conditional retention is dynamic success divided by clear successes for the same method-stream-scene cells and remains method-specific descriptive.
- Path change compares clear and dynamic `path_ids` only when both plans are exact-valid and remains descriptive.
