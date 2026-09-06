# Multi-scene reachability benchmark summary

**EXPLICIT CONFIRMATORY ANALYSIS.** Inference is conditional on the fixed scene catalog and clusters deterministic roadmap streams.

Audit: **PASS** — 180 graph builds and 21600 paired phase queries; no timeout or infrastructure-error rows.
Timing context: `functional-run-not-controlled-timing`.
Catalog pairing: 30 base trajectories, each represented by exactly one point and one segment scene.

## Per-method descriptive results

| Method | Clear success | Dynamic success | Conditional retention | Path change on joint-success | Build ms (median [IQR]) |
|---|---:|---:|---:|---:|---:|
| gng | 74.9% [73.8, 76.1]% | 73.8% [72.6, 75.0]% | 98.4% | 37.4% | 1721.5 [1708.1, 1743.5] |
| guarded_gng | 78.1% [76.3, 79.9]% | 77.4% [75.7, 79.2]% | 99.1% | 32.7% | 1594.1 [1576.6, 1618.3] |
| halton_prm | 80.9% [79.3, 82.6]% | 80.5% [78.9, 82.1]% | 99.5% | 27.1% | 1393.8 [1377.7, 1411.2] |

| Method | Clear planning ms* | Dynamic planning ms* | Clear path cost* | Dynamic path cost* |
|---|---:|---:|---:|---:|
| gng | 5.074 [4.243, 5.962] | 11.739 [8.504, 18.820] | 11.558 [9.219, 13.951] | 12.227 [9.742, 14.568] |
| guarded_gng | 5.848 [4.610, 7.146] | 12.572 [9.220, 19.791] | 14.319 [10.973, 17.542] | 14.835 [11.692, 18.103] |
| halton_prm | 6.224 [4.773, 7.928] | 12.930 [9.318, 19.153] | 15.284 [11.386, 18.879] | 15.665 [12.042, 19.340] |

*Planning-time and path-cost summaries include exact-valid paths only. Values are median [IQR].*

| Method | Clear publish-to-plan ms (all outcomes) | Dynamic publish-to-plan ms (all outcomes) |
|---|---:|---:|
| gng | 6.763 [4.488, 8.242] | 14.165 [9.995, 21.206] |
| guarded_gng | 7.394 [4.890, 9.306] | 14.389 [10.575, 21.280] |
| halton_prm | 7.831 [5.451, 10.147] | 14.809 [10.746, 21.109] |

## Prespecified primary endpoint

The endpoint is dynamic exact-valid success risk difference over the frozen catalog. Effects are guarded GNG minus baseline. The primary CI resamples whole roadmap streams only; the p-value uses a studentized paired stream-level method-label permutation. Holm correction covers exactly the two prespecified contrasts.

| Comparison | Risk difference | Fixed-catalog CI | Permutation p / Holm p | Sign dominance* |
|---|---:|---:|---:|---:|
| guarded_gng − gng | 3.7% | [1.6, 5.8]% | 0.0016 / 0.0032 | 0.481 |
| guarded_gng − halton_prm | -3.1% | [-5.2, -0.9]% | 0.0088 / 0.0088 | -0.259 |

*Sign dominance is (positive streams − negative streams) / non-tied streams; it is not a rank-biserial effect.*

The two-way stream × base-trajectory bootstrap is reported only as a scene-generalization sensitivity in `pairwise.csv` and `analysis.json`; it is not the primary fixed-catalog CI.

## Descriptive catalog strata

These subgroup estimates are descriptive and are not multiplicity-tested.

| Dimension | Level | Scenes | GNG | Guarded GNG | Halton PRM | Guarded−GNG | Guarded−Halton |
|---|---|---:|---:|---:|---:|---:|---:|
| obstacle_kind | point | 30 | 74.3% | 77.8% | 80.7% | 3.4% | -2.9% |
| obstacle_kind | segment | 30 | 73.2% | 77.1% | 80.3% | 3.9% | -3.2% |
| difficulty | low | 20 | 74.2% | 74.9% | 78.1% | 0.7% | -3.2% |
| difficulty | medium | 20 | 73.4% | 74.0% | 77.2% | 0.6% | -3.2% |
| difficulty | high | 20 | 73.8% | 83.4% | 86.2% | 9.7% | -2.8% |
| difficulty_x_obstacle_kind | low:point | 10 | 75.8% | 75.7% | 78.5% | -0.2% | -2.8% |
| difficulty_x_obstacle_kind | low:segment | 10 | 72.5% | 74.2% | 77.7% | 1.7% | -3.5% |
| difficulty_x_obstacle_kind | medium:point | 10 | 73.3% | 74.2% | 77.3% | 0.8% | -3.2% |
| difficulty_x_obstacle_kind | medium:segment | 10 | 73.5% | 73.8% | 77.2% | 0.3% | -3.3% |
| difficulty_x_obstacle_kind | high:point | 10 | 73.8% | 83.5% | 86.2% | 9.7% | -2.7% |
| difficulty_x_obstacle_kind | high:segment | 10 | 73.7% | 83.3% | 86.2% | 9.7% | -2.8% |

All secondary effects, including build/planning time and path cost, are descriptive and retained in `pairwise.csv` and `analysis.json`. Planning and path-cost differences use only common exact-success cells; path-change uses common four-way clear/dynamic joint-success cells.

## Statistical scope

- The primary estimand averages within-stream dynamic exact-valid risk differences over the frozen scene catalog, then averages over roadmap streams.
- The primary bootstrap resamples complete roadmap streams and never resamples catalog scenes.
- Both primary contrasts use the same deterministic sequence of resampled roadmap-stream indices.
- Scene-level McNemar tests are intentionally not reported because repeated scenes within a stream are not independent pairs.
- Conditional retention is dynamic success divided by clear successes for the same method-stream-scene cells and remains method-specific descriptive.
- Path change compares clear and dynamic `path_ids` only when both plans are exact-valid and remains descriptive.
