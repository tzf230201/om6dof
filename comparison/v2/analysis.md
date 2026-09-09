# OM6DOF Cartesian workspace analysis

This offline experiment contains **33401 discrete points**. The report shows every constant-X, constant-Y, and constant-Z plane at multiples of **25 mm**, from negative to positive coordinates within the sample range. On each plane, the other two coordinates vary; the report is not limited to the X=Y=Z=0 slices.

Solver parameters, target orientation, tolerances, model radius, seeds, and computation time are recorded in [summary.json](summary.json). Joint values and the orientation actually tested at each point are in [points.csv](points.csv). Open the [interactive 3D viewer](viewer3d.html), [all constant-coordinate planes](index.html), or [per-plane statistics](slices.csv).

## Main results

- Position found without requiring a specific orientation: **15175/33401 (45.4%)**.
- Both position and the tested orientation found: **3532/33401 (10.6%)**.
- Among the poses found, 334 points are classified as near singularity. This describes the best candidate found; it does not prove that every configuration at that point is singular.
- 1848 selected position candidates have less than 1 degree of margin to an effective joint limit. This indicates sensitivity of those candidates, not that every solution lies near a limit.

| Category | Points | Percentage of all points |
|---|---:|---:|
| Pose found | 3198 | 9.57% |
| Pose near singularity | 334 | 1.00% |
| Position found; orientation unresolved | 11643 | 34.86% |
| Colliding candidates only | 358 | 1.07% |
| Position unresolved | 17868 | 53.50% |

These percentages count only tested grid points; they do not measure continuous workspace volume or the physical robot's probability of success.

## Interpretation for mechanical design

1. **Green** provides a witness joint configuration satisfying the position, tested orientation, joint limits, and model self-collision check.
2. **Blue triangles** mean a pose was found, but the best candidate's Jacobian is poorly conditioned. Small Cartesian motions may require large joint changes. The rcond metric depends on length normalization; compare only experiments with identical settings.
3. **Yellow squares** separate orientation constraints from position reachability. They help evaluate gripper direction, wrist range, and tasks allowing a free orientation. They do not prove the requested orientation is impossible.
4. **Purple** means the search found only colliding position candidates under the approximate collision model; it does not prove that no collision-free configuration exists.
5. **Red** means the position search did not succeed. Possible causes include geometry, joint limits, too few seeds, or convergence to a local minimum. Do not treat it as proof of a mechanical dead zone.

Investigate suspected problem regions with a finer local grid, more seeds/iterations, several orientations, and mesh collision checks. Comparing zero joint margin with the operational margin can separate software constraints from the URDF range, but does not change or establish the physical hardware limits.

## Selected-candidate statistics

Margins are measured against the effective joint limits used by the scanner. Position-only results use the candidate with the smallest position error; full-pose results use the candidate with the best rcond.

| Metric | Samples | Minimum | Median | Maximum |
|---|---:|---:|---:|---:|
| Pose rcond (dimensionless) | 3532 | 0.00000 | 0.04179 | 0.12024 |
| Position-candidate joint margin (deg) | 15175 | 0.00000 | 10.98942 | 70.85408 |
| Position error of found poses (mm) | 3532 | 0.00005 | 0.00035 | 0.99967 |
| Orientation error of found poses (deg) | 3532 | 0.00000 | 0.00007 | 0.46902 |

Example candidates with the smallest joint margins (all joint values are available in points.csv):

| X (mm) | Y (mm) | Z (mm) | Margin (deg) | Status |
|---:|---:|---:|---:|---|
| -350 | -125 | 0 | 0.0000 | Position found; orientation unresolved |
| -350 | -125 | 150 | 0.0000 | Position found; orientation unresolved |
| -350 | -100 | -25 | 0.0000 | Position found; orientation unresolved |
| -350 | -50 | -25 | 0.0000 | Position found; orientation unresolved |
| -350 | -25 | -25 | 0.0000 | Position found; orientation unresolved |
| -350 | -25 | 0 | 0.0000 | Position found; orientation unresolved |
| -350 | 0 | -25 | 0.0000 | Position found; orientation unresolved |
| -350 | 0 | 25 | 0.0000 | Position found; orientation unresolved |

## All constant-coordinate planes

Each point appears once in each axis group when the grid aligns with the slice increment. Do not add the X+Y+Z group counts and interpret them as unique points. Planes with no samples are marked n/a, not counted as failures.

### Constant X

![All constant-X planes](slices_X.svg)

| X (mm) | Tested | Position found | Pose found | Near singularity | Orientation unresolved | Colliding only | Position unresolved |
|---:|---:|---:|---:|---:|---:|---:|---:|
| [-500](slices/X_minus_500mm.svg) | 1 | 0 (0.0%) | 0 (0.0%) | 0 | 0 | 0 | 1 |
| [-475](slices/X_minus_475mm.svg) | 121 | 0 (0.0%) | 0 (0.0%) | 0 | 0 | 0 | 121 |
| [-450](slices/X_minus_450mm.svg) | 241 | 0 (0.0%) | 0 (0.0%) | 0 | 0 | 0 | 241 |
| [-425](slices/X_minus_425mm.svg) | 349 | 0 (0.0%) | 0 (0.0%) | 0 | 0 | 0 | 349 |
| [-400](slices/X_minus_400mm.svg) | 441 | 0 (0.0%) | 0 (0.0%) | 0 | 0 | 0 | 441 |
| [-375](slices/X_minus_375mm.svg) | 553 | 45 (8.1%) | 0 (0.0%) | 0 | 45 | 0 | 508 |
| [-350](slices/X_minus_350mm.svg) | 641 | 137 (21.4%) | 0 (0.0%) | 0 | 137 | 0 | 504 |
| [-325](slices/X_minus_325mm.svg) | 725 | 216 (29.8%) | 0 (0.0%) | 0 | 216 | 0 | 509 |
| [-300](slices/X_minus_300mm.svg) | 797 | 286 (35.9%) | 0 (0.0%) | 0 | 286 | 0 | 511 |
| [-275](slices/X_minus_275mm.svg) | 877 | 348 (39.7%) | 0 (0.0%) | 0 | 348 | 0 | 529 |
| [-250](slices/X_minus_250mm.svg) | 949 | 367 (38.7%) | 0 (0.0%) | 0 | 367 | 0 | 582 |
| [-225](slices/X_minus_225mm.svg) | 997 | 377 (37.8%) | 0 (0.0%) | 0 | 377 | 0 | 620 |
| [-200](slices/X_minus_200mm.svg) | 1049 | 391 (37.3%) | 0 (0.0%) | 0 | 391 | 0 | 658 |
| [-175](slices/X_minus_175mm.svg) | 1101 | 389 (35.3%) | 0 (0.0%) | 0 | 389 | 36 | 676 |
| [-150](slices/X_minus_150mm.svg) | 1137 | 402 (35.4%) | 0 (0.0%) | 0 | 402 | 66 | 669 |
| [-125](slices/X_minus_125mm.svg) | 1185 | 523 (44.1%) | 0 (0.0%) | 0 | 523 | 52 | 610 |
| [-100](slices/X_minus_100mm.svg) | 1201 | 613 (51.0%) | 0 (0.0%) | 0 | 613 | 41 | 547 |
| [-75](slices/X_minus_75mm.svg) | 1225 | 656 (53.6%) | 12 (1.0%) | 0 | 644 | 41 | 528 |
| [-50](slices/X_minus_50mm.svg) | 1237 | 702 (56.8%) | 14 (1.1%) | 0 | 688 | 26 | 509 |
| [-25](slices/X_minus_25mm.svg) | 1245 | 718 (57.7%) | 26 (2.1%) | 0 | 692 | 29 | 498 |
| [0](slices/X_plus_0mm.svg) | 1257 | 729 (58.0%) | 79 (6.3%) | 0 | 650 | 31 | 497 |
| [25](slices/X_plus_25mm.svg) | 1245 | 745 (59.8%) | 81 (6.5%) | 0 | 664 | 23 | 477 |
| [50](slices/X_plus_50mm.svg) | 1237 | 754 (61.0%) | 231 (18.7%) | 14 | 523 | 13 | 470 |
| [75](slices/X_plus_75mm.svg) | 1225 | 762 (62.2%) | 234 (19.1%) | 6 | 528 | 0 | 463 |
| [100](slices/X_plus_100mm.svg) | 1201 | 744 (61.9%) | 252 (21.0%) | 10 | 492 | 0 | 457 |
| [125](slices/X_plus_125mm.svg) | 1185 | 723 (61.0%) | 258 (21.8%) | 12 | 465 | 0 | 462 |
| [150](slices/X_plus_150mm.svg) | 1137 | 686 (60.3%) | 248 (21.8%) | 31 | 438 | 0 | 451 |
| [175](slices/X_plus_175mm.svg) | 1101 | 658 (59.8%) | 258 (23.4%) | 56 | 400 | 0 | 443 |
| [200](slices/X_plus_200mm.svg) | 1049 | 609 (58.1%) | 262 (25.0%) | 43 | 347 | 0 | 440 |
| [225](slices/X_plus_225mm.svg) | 997 | 565 (56.7%) | 261 (26.2%) | 29 | 304 | 0 | 432 |
| [250](slices/X_plus_250mm.svg) | 949 | 509 (53.6%) | 264 (27.8%) | 25 | 245 | 0 | 440 |
| [275](slices/X_plus_275mm.svg) | 877 | 441 (50.3%) | 263 (30.0%) | 26 | 178 | 0 | 436 |
| [300](slices/X_plus_300mm.svg) | 797 | 378 (47.4%) | 258 (32.4%) | 21 | 120 | 0 | 419 |
| [325](slices/X_plus_325mm.svg) | 725 | 299 (41.2%) | 225 (31.0%) | 19 | 74 | 0 | 426 |
| [350](slices/X_plus_350mm.svg) | 641 | 225 (35.1%) | 171 (26.7%) | 19 | 54 | 0 | 416 |
| [375](slices/X_plus_375mm.svg) | 553 | 137 (24.8%) | 104 (18.8%) | 18 | 33 | 0 | 416 |
| [400](slices/X_plus_400mm.svg) | 441 | 41 (9.3%) | 31 (7.0%) | 5 | 10 | 0 | 400 |
| [425](slices/X_plus_425mm.svg) | 349 | 0 (0.0%) | 0 (0.0%) | 0 | 0 | 0 | 349 |
| [450](slices/X_plus_450mm.svg) | 241 | 0 (0.0%) | 0 (0.0%) | 0 | 0 | 0 | 241 |
| [475](slices/X_plus_475mm.svg) | 121 | 0 (0.0%) | 0 (0.0%) | 0 | 0 | 0 | 121 |
| [500](slices/X_plus_500mm.svg) | 1 | 0 (0.0%) | 0 (0.0%) | 0 | 0 | 0 | 1 |

### Constant Y

![All constant-Y planes](slices_Y.svg)

| Y (mm) | Tested | Position found | Pose found | Near singularity | Orientation unresolved | Colliding only | Position unresolved |
|---:|---:|---:|---:|---:|---:|---:|---:|
| [-500](slices/Y_minus_500mm.svg) | 1 | 0 (0.0%) | 0 (0.0%) | 0 | 0 | 0 | 1 |
| [-475](slices/Y_minus_475mm.svg) | 121 | 0 (0.0%) | 0 (0.0%) | 0 | 0 | 0 | 121 |
| [-450](slices/Y_minus_450mm.svg) | 241 | 0 (0.0%) | 0 (0.0%) | 0 | 0 | 0 | 241 |
| [-425](slices/Y_minus_425mm.svg) | 349 | 0 (0.0%) | 0 (0.0%) | 0 | 0 | 0 | 349 |
| [-400](slices/Y_minus_400mm.svg) | 441 | 0 (0.0%) | 0 (0.0%) | 0 | 0 | 0 | 441 |
| [-375](slices/Y_minus_375mm.svg) | 553 | 87 (15.7%) | 26 (4.7%) | 6 | 61 | 0 | 466 |
| [-350](slices/Y_minus_350mm.svg) | 641 | 176 (27.5%) | 55 (8.6%) | 7 | 121 | 0 | 465 |
| [-325](slices/Y_minus_325mm.svg) | 725 | 258 (35.6%) | 85 (11.7%) | 7 | 173 | 0 | 467 |
| [-300](slices/Y_minus_300mm.svg) | 797 | 336 (42.2%) | 112 (14.1%) | 9 | 224 | 0 | 461 |
| [-275](slices/Y_minus_275mm.svg) | 877 | 401 (45.7%) | 117 (13.3%) | 8 | 284 | 0 | 476 |
| [-250](slices/Y_minus_250mm.svg) | 949 | 464 (48.9%) | 118 (12.4%) | 8 | 346 | 0 | 485 |
| [-225](slices/Y_minus_225mm.svg) | 997 | 517 (51.9%) | 121 (12.1%) | 9 | 396 | 0 | 480 |
| [-200](slices/Y_minus_200mm.svg) | 1049 | 570 (54.3%) | 117 (11.2%) | 9 | 453 | 0 | 479 |
| [-175](slices/Y_minus_175mm.svg) | 1101 | 601 (54.6%) | 112 (10.2%) | 7 | 489 | 0 | 500 |
| [-150](slices/Y_minus_150mm.svg) | 1137 | 628 (55.2%) | 126 (11.1%) | 9 | 502 | 0 | 509 |
| [-125](slices/Y_minus_125mm.svg) | 1185 | 636 (53.7%) | 156 (13.2%) | 8 | 480 | 1 | 548 |
| [-100](slices/Y_minus_100mm.svg) | 1201 | 652 (54.3%) | 140 (11.7%) | 14 | 512 | 12 | 537 |
| [-75](slices/Y_minus_75mm.svg) | 1225 | 659 (53.8%) | 137 (11.2%) | 17 | 522 | 24 | 542 |
| [-50](slices/Y_minus_50mm.svg) | 1237 | 657 (53.1%) | 140 (11.3%) | 21 | 517 | 40 | 540 |
| [-25](slices/Y_minus_25mm.svg) | 1245 | 625 (50.2%) | 136 (10.9%) | 19 | 489 | 65 | 555 |
| [0](slices/Y_plus_0mm.svg) | 1257 | 609 (48.4%) | 136 (10.8%) | 21 | 473 | 82 | 566 |
| [25](slices/Y_plus_25mm.svg) | 1245 | 630 (50.6%) | 138 (11.1%) | 21 | 492 | 64 | 551 |
| [50](slices/Y_plus_50mm.svg) | 1237 | 670 (54.2%) | 140 (11.3%) | 21 | 530 | 36 | 531 |
| [75](slices/Y_plus_75mm.svg) | 1225 | 675 (55.1%) | 138 (11.3%) | 18 | 537 | 23 | 527 |
| [100](slices/Y_plus_100mm.svg) | 1201 | 673 (56.0%) | 138 (11.5%) | 10 | 535 | 8 | 520 |
| [125](slices/Y_plus_125mm.svg) | 1185 | 652 (55.0%) | 155 (13.1%) | 7 | 497 | 3 | 530 |
| [150](slices/Y_plus_150mm.svg) | 1137 | 633 (55.7%) | 126 (11.1%) | 9 | 507 | 0 | 504 |
| [175](slices/Y_plus_175mm.svg) | 1101 | 600 (54.5%) | 112 (10.2%) | 7 | 488 | 0 | 501 |
| [200](slices/Y_plus_200mm.svg) | 1049 | 563 (53.7%) | 117 (11.2%) | 9 | 446 | 0 | 486 |
| [225](slices/Y_plus_225mm.svg) | 997 | 513 (51.5%) | 121 (12.1%) | 9 | 392 | 0 | 484 |
| [250](slices/Y_plus_250mm.svg) | 949 | 455 (47.9%) | 118 (12.4%) | 8 | 337 | 0 | 494 |
| [275](slices/Y_plus_275mm.svg) | 877 | 391 (44.6%) | 116 (13.2%) | 6 | 275 | 0 | 486 |
| [300](slices/Y_plus_300mm.svg) | 797 | 327 (41.0%) | 112 (14.1%) | 9 | 215 | 0 | 470 |
| [325](slices/Y_plus_325mm.svg) | 725 | 253 (34.9%) | 85 (11.7%) | 7 | 168 | 0 | 472 |
| [350](slices/Y_plus_350mm.svg) | 641 | 176 (27.5%) | 56 (8.7%) | 8 | 120 | 0 | 465 |
| [375](slices/Y_plus_375mm.svg) | 553 | 88 (15.9%) | 26 (4.7%) | 6 | 62 | 0 | 465 |
| [400](slices/Y_plus_400mm.svg) | 441 | 0 (0.0%) | 0 (0.0%) | 0 | 0 | 0 | 441 |
| [425](slices/Y_plus_425mm.svg) | 349 | 0 (0.0%) | 0 (0.0%) | 0 | 0 | 0 | 349 |
| [450](slices/Y_plus_450mm.svg) | 241 | 0 (0.0%) | 0 (0.0%) | 0 | 0 | 0 | 241 |
| [475](slices/Y_plus_475mm.svg) | 121 | 0 (0.0%) | 0 (0.0%) | 0 | 0 | 0 | 121 |
| [500](slices/Y_plus_500mm.svg) | 1 | 0 (0.0%) | 0 (0.0%) | 0 | 0 | 0 | 1 |

### Constant Z

![All constant-Z planes](slices_Z.svg)

| Z (mm) | Tested | Position found | Pose found | Near singularity | Orientation unresolved | Colliding only | Position unresolved |
|---:|---:|---:|---:|---:|---:|---:|---:|
| [-500](slices/Z_minus_500mm.svg) | 1 | 0 (0.0%) | 0 (0.0%) | 0 | 0 | 0 | 1 |
| [-475](slices/Z_minus_475mm.svg) | 121 | 0 (0.0%) | 0 (0.0%) | 0 | 0 | 0 | 121 |
| [-450](slices/Z_minus_450mm.svg) | 241 | 0 (0.0%) | 0 (0.0%) | 0 | 0 | 0 | 241 |
| [-425](slices/Z_minus_425mm.svg) | 349 | 0 (0.0%) | 0 (0.0%) | 0 | 0 | 0 | 349 |
| [-400](slices/Z_minus_400mm.svg) | 441 | 0 (0.0%) | 0 (0.0%) | 0 | 0 | 0 | 441 |
| [-375](slices/Z_minus_375mm.svg) | 553 | 0 (0.0%) | 0 (0.0%) | 0 | 0 | 0 | 553 |
| [-350](slices/Z_minus_350mm.svg) | 641 | 0 (0.0%) | 0 (0.0%) | 0 | 0 | 0 | 641 |
| [-325](slices/Z_minus_325mm.svg) | 725 | 0 (0.0%) | 0 (0.0%) | 0 | 0 | 0 | 725 |
| [-300](slices/Z_minus_300mm.svg) | 797 | 0 (0.0%) | 0 (0.0%) | 0 | 0 | 0 | 797 |
| [-275](slices/Z_minus_275mm.svg) | 877 | 0 (0.0%) | 0 (0.0%) | 0 | 0 | 0 | 877 |
| [-250](slices/Z_minus_250mm.svg) | 949 | 51 (5.4%) | 0 (0.0%) | 0 | 51 | 0 | 898 |
| [-225](slices/Z_minus_225mm.svg) | 997 | 168 (16.9%) | 0 (0.0%) | 0 | 168 | 0 | 829 |
| [-200](slices/Z_minus_200mm.svg) | 1049 | 250 (23.8%) | 0 (0.0%) | 0 | 250 | 0 | 799 |
| [-175](slices/Z_minus_175mm.svg) | 1101 | 309 (28.1%) | 0 (0.0%) | 0 | 309 | 0 | 792 |
| [-150](slices/Z_minus_150mm.svg) | 1137 | 363 (31.9%) | 0 (0.0%) | 0 | 363 | 0 | 774 |
| [-125](slices/Z_minus_125mm.svg) | 1185 | 436 (36.8%) | 0 (0.0%) | 0 | 436 | 2 | 747 |
| [-100](slices/Z_minus_100mm.svg) | 1201 | 510 (42.5%) | 0 (0.0%) | 0 | 510 | 7 | 684 |
| [-75](slices/Z_minus_75mm.svg) | 1225 | 568 (46.4%) | 66 (5.4%) | 0 | 502 | 13 | 644 |
| [-50](slices/Z_minus_50mm.svg) | 1237 | 611 (49.4%) | 95 (7.7%) | 8 | 516 | 23 | 603 |
| [-25](slices/Z_minus_25mm.svg) | 1245 | 635 (51.0%) | 113 (9.1%) | 9 | 522 | 30 | 580 |
| [0](slices/Z_plus_0mm.svg) | 1257 | 649 (51.6%) | 133 (10.6%) | 6 | 516 | 47 | 561 |
| [25](slices/Z_plus_25mm.svg) | 1245 | 670 (53.8%) | 159 (12.8%) | 4 | 511 | 51 | 524 |
| [50](slices/Z_plus_50mm.svg) | 1237 | 682 (55.1%) | 185 (15.0%) | 4 | 497 | 53 | 502 |
| [75](slices/Z_plus_75mm.svg) | 1225 | 700 (57.1%) | 210 (17.1%) | 0 | 490 | 40 | 485 |
| [100](slices/Z_plus_100mm.svg) | 1201 | 714 (59.5%) | 247 (20.6%) | 2 | 467 | 38 | 449 |
| [125](slices/Z_plus_125mm.svg) | 1185 | 732 (61.8%) | 261 (22.0%) | 8 | 471 | 31 | 422 |
| [150](slices/Z_plus_150mm.svg) | 1137 | 762 (67.0%) | 296 (26.0%) | 20 | 466 | 14 | 361 |
| [175](slices/Z_plus_175mm.svg) | 1101 | 762 (69.2%) | 342 (31.1%) | 39 | 420 | 3 | 336 |
| [200](slices/Z_plus_200mm.svg) | 1049 | 741 (70.6%) | 322 (30.7%) | 58 | 419 | 1 | 307 |
| [225](slices/Z_plus_225mm.svg) | 997 | 715 (71.7%) | 282 (28.3%) | 112 | 433 | 3 | 279 |
| [250](slices/Z_plus_250mm.svg) | 949 | 678 (71.4%) | 288 (30.3%) | 15 | 390 | 2 | 269 |
| [275](slices/Z_plus_275mm.svg) | 877 | 640 (73.0%) | 249 (28.4%) | 15 | 391 | 0 | 237 |
| [300](slices/Z_plus_300mm.svg) | 797 | 588 (73.8%) | 193 (24.2%) | 15 | 395 | 0 | 209 |
| [325](slices/Z_plus_325mm.svg) | 725 | 530 (73.1%) | 91 (12.6%) | 19 | 439 | 0 | 195 |
| [350](slices/Z_plus_350mm.svg) | 641 | 472 (73.6%) | 0 (0.0%) | 0 | 472 | 0 | 169 |
| [375](slices/Z_plus_375mm.svg) | 553 | 406 (73.4%) | 0 (0.0%) | 0 | 406 | 0 | 147 |
| [400](slices/Z_plus_400mm.svg) | 441 | 335 (76.0%) | 0 (0.0%) | 0 | 335 | 0 | 106 |
| [425](slices/Z_plus_425mm.svg) | 349 | 254 (72.8%) | 0 (0.0%) | 0 | 254 | 0 | 95 |
| [450](slices/Z_plus_450mm.svg) | 241 | 166 (68.9%) | 0 (0.0%) | 0 | 166 | 0 | 75 |
| [475](slices/Z_plus_475mm.svg) | 121 | 78 (64.5%) | 0 (0.0%) | 0 | 78 | 0 | 43 |
| [500](slices/Z_plus_500mm.svg) | 1 | 0 (0.0%) | 0 (0.0%) | 0 | 0 | 0 | 1 |

## Limitations

- No ROS commands, servo connections, or robot motion. All results describe model kinematics, not measured physical accuracy.
- Each point tests one orientation defined by the experiment settings, not every rotation in SO(3). In radial mode, yaw follows the point's azimuth.
- The bounding sphere is a conservative model sampling domain, not a guarantee that every position inside its radius is reachable. Regions below the base are also sampled if inside the sphere; a table/floor is not modeled.
- Collision checks use approximate model links at the endpoint, not precise mechanical meshes, environmental obstacles, or a verified path from a starting configuration.
- Payload, flexibility, backlash, calibration, motor torque, gravity, velocity, and acceleration are not evaluated.
- Colored markers mark sample centers; they do not fill or classify the area/volume between samples. Unsampled positions are not assessed. Green circles, yellow squares, blue triangles, purple diamonds, and red X markers distinguish the categories.
- A confirmed mechanical dead zone requires additional evidence; numerical IK failure alone is insufficient.

## Experiment parameters (summary.json snapshot)

Check `orientation`, `rpy_deg`, reference frame, position/orientation tolerances, and joint margin before comparing maps. The reporter does not change scanner parameters or data.

```json
{
"engine":"C++17 Eigen/KDL, bounded multi-start LM",
"points":33401,
"radius_mm":500,
"conservative_urdf_bound_mm":497.53921681548246,
"elapsed_seconds":36.045611200000003,
"position_found":15175,
"pose_found":3532,
"arguments":{"spacing_mm":25,"samples":2000,"seeds":4,"iterations":100,"seed":42,"orientation":"radial","base_link":"world","tip_link":"end_effector_link","joint_margin_rad":0.02,"collision_radius_mm":25,"position_tolerance_mm":1,"orientation_tolerance_deg":0.5,"length_scale_mm":300,"singular_threshold":0.01,"rpy_deg":[0,0,0]},
"hard_lower_rad":[-1.5707963267948966,-2.0420352248333655,-1.5707963267948966,-1.2566370614359172,-1.5707963267948966,-1.5707963267948966],
"hard_upper_rad":[1.5707963267948966,2.1048670779051615,2.1362830044410597,1.2566370614359172,2.1048670779051615,1.5707963267948966],
"effective_lower_rad":[-1.5507963267948965,-2.0220352248333655,-1.5507963267948965,-1.2366370614359172,-1.5507963267948965,-1.5507963267948965],
"effective_upper_rad":[1.5507963267948965,2.0848670779051615,2.1162830044410597,1.2366370614359172,2.0848670779051615,1.5507963267948965],
"counts":{"collision_candidates_only":358,"orientation_unresolved":11643,"pose_found":3198,"pose_near_singular":334,"position_unresolved":17868},
"limitations":["Finite grid and seeds; unresolved is not proven unreachable","One orientation per point; not exhaustive SO(3)","Endpoint capsule collision only, no trajectory/obstacles/dynamics","Full sphere includes negative Z; no table","Singularity refers to best tested pose witness, not all configurations"]}

```
