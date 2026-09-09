# OM6DOF Cartesian workspace analysis

This offline experiment contains **33401 discrete points**. The report shows every constant-X, constant-Y, and constant-Z plane at multiples of **25 mm**, from negative to positive coordinates within the sample range. On each plane, the other two coordinates vary; the report is not limited to the X=Y=Z=0 slices.

Solver parameters, target orientation, tolerances, model radius, seeds, and computation time are recorded in [summary.json](summary.json). Joint values and the orientation actually tested at each point are in [points.csv](points.csv). Open the [interactive 3D viewer](viewer3d.html), [all constant-coordinate planes](index.html), or [per-plane statistics](slices.csv).

## Main results

- Position found without requiring a specific orientation: **15532/33401 (46.5%)**.
- Both position and the tested orientation found: **2935/33401 (8.8%)**.
- Among the poses found, 197 points are classified as near singularity. This describes the best candidate found; it does not prove that every configuration at that point is singular.
- 909 selected position candidates have less than 1 degree of margin to an effective joint limit. This indicates sensitivity of those candidates, not that every solution lies near a limit.

| Category | Points | Percentage of all points |
|---|---:|---:|
| Pose found | 2738 | 8.20% |
| Pose near singularity | 197 | 0.59% |
| Position found; orientation unresolved | 12597 | 37.71% |
| Colliding candidates only | 90 | 0.27% |
| Position unresolved | 17779 | 53.23% |

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
| Pose rcond (dimensionless) | 2935 | 0.00002 | 0.05778 | 0.12601 |
| Position-candidate joint margin (deg) | 15532 | 0.00000 | 14.36692 | 70.85408 |
| Position error of found poses (mm) | 2935 | 0.00005 | 0.00034 | 0.96794 |
| Orientation error of found poses (deg) | 2935 | 0.00000 | 0.00005 | 0.29005 |

Example candidates with the smallest joint margins (all joint values are available in points.csv):

| X (mm) | Y (mm) | Z (mm) | Margin (deg) | Status |
|---:|---:|---:|---:|---|
| -350 | -125 | 50 | 0.0000 | Position found; orientation unresolved |
| -350 | -125 | 75 | 0.0000 | Position found; orientation unresolved |
| -350 | 100 | 50 | 0.0000 | Position found; orientation unresolved |
| -350 | 125 | 50 | 0.0000 | Position found; orientation unresolved |
| -350 | 125 | 75 | 0.0000 | Position found; orientation unresolved |
| -325 | -150 | 125 | 0.0000 | Position found; orientation unresolved |
| -325 | -150 | 225 | 0.0000 | Position found; orientation unresolved |
| -325 | -25 | -100 | 0.0000 | Position found; orientation unresolved |

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
| [-375](slices/X_minus_375mm.svg) | 553 | 21 (3.8%) | 0 (0.0%) | 0 | 21 | 0 | 532 |
| [-350](slices/X_minus_350mm.svg) | 641 | 114 (17.8%) | 0 (0.0%) | 0 | 114 | 0 | 527 |
| [-325](slices/X_minus_325mm.svg) | 725 | 202 (27.9%) | 0 (0.0%) | 0 | 202 | 0 | 523 |
| [-300](slices/X_minus_300mm.svg) | 797 | 283 (35.5%) | 0 (0.0%) | 0 | 283 | 0 | 514 |
| [-275](slices/X_minus_275mm.svg) | 877 | 353 (40.3%) | 0 (0.0%) | 0 | 353 | 0 | 524 |
| [-250](slices/X_minus_250mm.svg) | 949 | 423 (44.6%) | 0 (0.0%) | 0 | 423 | 0 | 526 |
| [-225](slices/X_minus_225mm.svg) | 997 | 482 (48.3%) | 0 (0.0%) | 0 | 482 | 0 | 515 |
| [-200](slices/X_minus_200mm.svg) | 1049 | 540 (51.5%) | 0 (0.0%) | 0 | 540 | 0 | 509 |
| [-175](slices/X_minus_175mm.svg) | 1101 | 579 (52.6%) | 0 (0.0%) | 0 | 579 | 0 | 522 |
| [-150](slices/X_minus_150mm.svg) | 1137 | 617 (54.3%) | 0 (0.0%) | 0 | 617 | 0 | 520 |
| [-125](slices/X_minus_125mm.svg) | 1185 | 647 (54.6%) | 0 (0.0%) | 0 | 647 | 0 | 538 |
| [-100](slices/X_minus_100mm.svg) | 1201 | 675 (56.2%) | 0 (0.0%) | 0 | 675 | 0 | 526 |
| [-75](slices/X_minus_75mm.svg) | 1225 | 698 (57.0%) | 10 (0.8%) | 2 | 688 | 3 | 524 |
| [-50](slices/X_minus_50mm.svg) | 1237 | 705 (57.0%) | 10 (0.8%) | 2 | 695 | 6 | 526 |
| [-25](slices/X_minus_25mm.svg) | 1245 | 695 (55.8%) | 18 (1.4%) | 2 | 677 | 22 | 528 |
| [0](slices/X_plus_0mm.svg) | 1257 | 701 (55.8%) | 66 (5.3%) | 2 | 635 | 26 | 530 |
| [25](slices/X_plus_25mm.svg) | 1245 | 711 (57.1%) | 69 (5.5%) | 2 | 642 | 20 | 514 |
| [50](slices/X_plus_50mm.svg) | 1237 | 715 (57.8%) | 204 (16.5%) | 10 | 511 | 13 | 509 |
| [75](slices/X_plus_75mm.svg) | 1225 | 725 (59.2%) | 201 (16.4%) | 13 | 524 | 0 | 500 |
| [100](slices/X_plus_100mm.svg) | 1201 | 709 (59.0%) | 199 (16.6%) | 8 | 510 | 0 | 492 |
| [125](slices/X_plus_125mm.svg) | 1185 | 682 (57.6%) | 190 (16.0%) | 3 | 492 | 0 | 503 |
| [150](slices/X_plus_150mm.svg) | 1137 | 655 (57.6%) | 207 (18.2%) | 22 | 448 | 0 | 482 |
| [175](slices/X_plus_175mm.svg) | 1101 | 620 (56.3%) | 205 (18.6%) | 33 | 415 | 0 | 481 |
| [200](slices/X_plus_200mm.svg) | 1049 | 580 (55.3%) | 201 (19.2%) | 20 | 379 | 0 | 469 |
| [225](slices/X_plus_225mm.svg) | 997 | 538 (54.0%) | 213 (21.4%) | 22 | 325 | 0 | 459 |
| [250](slices/X_plus_250mm.svg) | 949 | 480 (50.6%) | 215 (22.7%) | 14 | 265 | 0 | 469 |
| [275](slices/X_plus_275mm.svg) | 877 | 423 (48.2%) | 235 (26.8%) | 7 | 188 | 0 | 454 |
| [300](slices/X_plus_300mm.svg) | 797 | 353 (44.3%) | 250 (31.4%) | 9 | 103 | 0 | 444 |
| [325](slices/X_plus_325mm.svg) | 725 | 283 (39.0%) | 203 (28.0%) | 8 | 80 | 0 | 442 |
| [350](slices/X_plus_350mm.svg) | 641 | 198 (30.9%) | 143 (22.3%) | 5 | 55 | 0 | 443 |
| [375](slices/X_plus_375mm.svg) | 553 | 108 (19.5%) | 85 (15.4%) | 8 | 23 | 0 | 445 |
| [400](slices/X_plus_400mm.svg) | 441 | 17 (3.9%) | 11 (2.5%) | 5 | 6 | 0 | 424 |
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
| [-375](slices/Y_minus_375mm.svg) | 553 | 65 (11.8%) | 18 (3.3%) | 5 | 47 | 0 | 488 |
| [-350](slices/Y_minus_350mm.svg) | 641 | 157 (24.5%) | 49 (7.6%) | 5 | 108 | 0 | 484 |
| [-325](slices/Y_minus_325mm.svg) | 725 | 241 (33.2%) | 77 (10.6%) | 4 | 164 | 0 | 484 |
| [-300](slices/Y_minus_300mm.svg) | 797 | 315 (39.5%) | 101 (12.7%) | 1 | 214 | 0 | 482 |
| [-275](slices/Y_minus_275mm.svg) | 877 | 388 (44.2%) | 113 (12.9%) | 4 | 275 | 0 | 489 |
| [-250](slices/Y_minus_250mm.svg) | 949 | 452 (47.6%) | 105 (11.1%) | 2 | 347 | 0 | 497 |
| [-225](slices/Y_minus_225mm.svg) | 997 | 507 (50.9%) | 103 (10.3%) | 3 | 404 | 0 | 490 |
| [-200](slices/Y_minus_200mm.svg) | 1049 | 556 (53.0%) | 92 (8.8%) | 2 | 464 | 0 | 493 |
| [-175](slices/Y_minus_175mm.svg) | 1101 | 601 (54.6%) | 86 (7.8%) | 1 | 515 | 0 | 500 |
| [-150](slices/Y_minus_150mm.svg) | 1137 | 633 (55.7%) | 101 (8.9%) | 7 | 532 | 0 | 504 |
| [-125](slices/Y_minus_125mm.svg) | 1185 | 675 (57.0%) | 115 (9.7%) | 8 | 560 | 0 | 510 |
| [-100](slices/Y_minus_100mm.svg) | 1201 | 690 (57.5%) | 110 (9.2%) | 11 | 580 | 0 | 511 |
| [-75](slices/Y_minus_75mm.svg) | 1225 | 709 (57.9%) | 111 (9.1%) | 13 | 598 | 0 | 516 |
| [-50](slices/Y_minus_50mm.svg) | 1237 | 719 (58.1%) | 113 (9.1%) | 13 | 606 | 3 | 515 |
| [-25](slices/Y_minus_25mm.svg) | 1245 | 707 (56.8%) | 115 (9.2%) | 11 | 592 | 25 | 513 |
| [0](slices/Y_plus_0mm.svg) | 1257 | 704 (56.0%) | 112 (8.9%) | 13 | 592 | 29 | 524 |
| [25](slices/Y_plus_25mm.svg) | 1245 | 706 (56.7%) | 116 (9.3%) | 11 | 590 | 27 | 512 |
| [50](slices/Y_plus_50mm.svg) | 1237 | 716 (57.9%) | 115 (9.3%) | 13 | 601 | 6 | 515 |
| [75](slices/Y_plus_75mm.svg) | 1225 | 711 (58.0%) | 112 (9.1%) | 12 | 599 | 0 | 514 |
| [100](slices/Y_plus_100mm.svg) | 1201 | 691 (57.5%) | 110 (9.2%) | 12 | 581 | 0 | 510 |
| [125](slices/Y_plus_125mm.svg) | 1185 | 674 (56.9%) | 115 (9.7%) | 12 | 559 | 0 | 511 |
| [150](slices/Y_plus_150mm.svg) | 1137 | 633 (55.7%) | 101 (8.9%) | 8 | 532 | 0 | 504 |
| [175](slices/Y_plus_175mm.svg) | 1101 | 601 (54.6%) | 86 (7.8%) | 3 | 515 | 0 | 500 |
| [200](slices/Y_plus_200mm.svg) | 1049 | 556 (53.0%) | 92 (8.8%) | 2 | 464 | 0 | 493 |
| [225](slices/Y_plus_225mm.svg) | 997 | 507 (50.9%) | 103 (10.3%) | 4 | 404 | 0 | 490 |
| [250](slices/Y_plus_250mm.svg) | 949 | 452 (47.6%) | 105 (11.1%) | 2 | 347 | 0 | 497 |
| [275](slices/Y_plus_275mm.svg) | 877 | 388 (44.2%) | 113 (12.9%) | 2 | 275 | 0 | 489 |
| [300](slices/Y_plus_300mm.svg) | 797 | 315 (39.5%) | 101 (12.7%) | 1 | 214 | 0 | 482 |
| [325](slices/Y_plus_325mm.svg) | 725 | 241 (33.2%) | 77 (10.6%) | 3 | 164 | 0 | 484 |
| [350](slices/Y_plus_350mm.svg) | 641 | 157 (24.5%) | 49 (7.6%) | 3 | 108 | 0 | 484 |
| [375](slices/Y_plus_375mm.svg) | 553 | 65 (11.8%) | 19 (3.4%) | 6 | 46 | 0 | 488 |
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
| [-250](slices/Z_minus_250mm.svg) | 949 | 0 (0.0%) | 0 (0.0%) | 0 | 0 | 0 | 949 |
| [-225](slices/Z_minus_225mm.svg) | 997 | 74 (7.4%) | 0 (0.0%) | 0 | 74 | 0 | 923 |
| [-200](slices/Z_minus_200mm.svg) | 1049 | 269 (25.6%) | 0 (0.0%) | 0 | 269 | 0 | 780 |
| [-175](slices/Z_minus_175mm.svg) | 1101 | 368 (33.4%) | 0 (0.0%) | 0 | 368 | 0 | 733 |
| [-150](slices/Z_minus_150mm.svg) | 1137 | 454 (39.9%) | 0 (0.0%) | 0 | 454 | 0 | 683 |
| [-125](slices/Z_minus_125mm.svg) | 1185 | 520 (43.9%) | 0 (0.0%) | 0 | 520 | 0 | 665 |
| [-100](slices/Z_minus_100mm.svg) | 1201 | 580 (48.3%) | 0 (0.0%) | 0 | 580 | 0 | 621 |
| [-75](slices/Z_minus_75mm.svg) | 1225 | 628 (51.3%) | 0 (0.0%) | 0 | 628 | 0 | 597 |
| [-50](slices/Z_minus_50mm.svg) | 1237 | 665 (53.8%) | 82 (6.6%) | 0 | 583 | 1 | 571 |
| [-25](slices/Z_minus_25mm.svg) | 1245 | 689 (55.3%) | 109 (8.8%) | 2 | 580 | 9 | 547 |
| [0](slices/Z_plus_0mm.svg) | 1257 | 710 (56.5%) | 136 (10.8%) | 4 | 574 | 12 | 535 |
| [25](slices/Z_plus_25mm.svg) | 1245 | 730 (58.6%) | 152 (12.2%) | 4 | 578 | 18 | 497 |
| [50](slices/Z_plus_50mm.svg) | 1237 | 749 (60.5%) | 184 (14.9%) | 4 | 565 | 15 | 473 |
| [75](slices/Z_plus_75mm.svg) | 1225 | 753 (61.5%) | 189 (15.4%) | 0 | 564 | 16 | 456 |
| [100](slices/Z_plus_100mm.svg) | 1201 | 763 (63.5%) | 186 (15.5%) | 3 | 577 | 11 | 427 |
| [125](slices/Z_plus_125mm.svg) | 1185 | 758 (64.0%) | 189 (15.9%) | 7 | 569 | 8 | 419 |
| [150](slices/Z_plus_150mm.svg) | 1137 | 760 (66.8%) | 208 (18.3%) | 12 | 552 | 0 | 377 |
| [175](slices/Z_plus_175mm.svg) | 1101 | 742 (67.4%) | 219 (19.9%) | 9 | 523 | 0 | 359 |
| [200](slices/Z_plus_200mm.svg) | 1049 | 718 (68.4%) | 261 (24.9%) | 18 | 457 | 0 | 331 |
| [225](slices/Z_plus_225mm.svg) | 997 | 692 (69.4%) | 306 (30.7%) | 60 | 386 | 0 | 305 |
| [250](slices/Z_plus_250mm.svg) | 949 | 650 (68.5%) | 275 (29.0%) | 13 | 375 | 0 | 299 |
| [275](slices/Z_plus_275mm.svg) | 877 | 612 (69.8%) | 231 (26.3%) | 11 | 381 | 0 | 265 |
| [300](slices/Z_plus_300mm.svg) | 797 | 562 (70.5%) | 171 (21.5%) | 13 | 391 | 0 | 235 |
| [325](slices/Z_plus_325mm.svg) | 725 | 510 (70.3%) | 37 (5.1%) | 37 | 473 | 0 | 215 |
| [350](slices/Z_plus_350mm.svg) | 641 | 452 (70.5%) | 0 (0.0%) | 0 | 452 | 0 | 189 |
| [375](slices/Z_plus_375mm.svg) | 553 | 386 (69.8%) | 0 (0.0%) | 0 | 386 | 0 | 167 |
| [400](slices/Z_plus_400mm.svg) | 441 | 308 (69.8%) | 0 (0.0%) | 0 | 308 | 0 | 133 |
| [425](slices/Z_plus_425mm.svg) | 349 | 232 (66.5%) | 0 (0.0%) | 0 | 232 | 0 | 117 |
| [450](slices/Z_plus_450mm.svg) | 241 | 142 (58.9%) | 0 (0.0%) | 0 | 142 | 0 | 99 |
| [475](slices/Z_plus_475mm.svg) | 121 | 56 (46.3%) | 0 (0.0%) | 0 | 56 | 0 | 65 |
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
"conservative_urdf_bound_mm":491.30865204668481,
"elapsed_seconds":32.308693695999999,
"position_found":15532,
"pose_found":2935,
"arguments":{"spacing_mm":25,"samples":2000,"seeds":4,"iterations":100,"seed":42,"orientation":"radial","base_link":"world","tip_link":"end_effector_link","joint_margin_rad":0.02,"collision_radius_mm":25,"position_tolerance_mm":1,"orientation_tolerance_deg":0.5,"length_scale_mm":300,"singular_threshold":0.01,"rpy_deg":[0,0,0]},
"hard_lower_rad":[-1.5707963267948966,-2.0420352248333655,-1.5707963267948966,-1.2566370614359172,-1.5707963267948966,-1.5707963267948966],
"hard_upper_rad":[1.5707963267948966,2.1048670779051615,2.1362830044410597,1.2566370614359172,2.1048670779051615,1.5707963267948966],
"effective_lower_rad":[-1.5507963267948965,-2.0220352248333655,-1.5507963267948965,-1.2366370614359172,-1.5507963267948965,-1.5507963267948965],
"effective_upper_rad":[1.5507963267948965,2.0848670779051615,2.1162830044410597,1.2366370614359172,2.0848670779051615,1.5507963267948965],
"counts":{"collision_candidates_only":90,"orientation_unresolved":12597,"pose_found":2738,"pose_near_singular":197,"position_unresolved":17779},
"limitations":["Finite grid and seeds; unresolved is not proven unreachable","One orientation per point; not exhaustive SO(3)","Endpoint capsule collision only, no trajectory/obstacles/dynamics","Full sphere includes negative Z; no table","Singularity refers to best tested pose witness, not all configurations"]}

```
