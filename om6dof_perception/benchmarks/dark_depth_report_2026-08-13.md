# RealSense D405 dark-scene depth evidence

## Question

Can the current D405 depth stream be used as the primary perception source in
the observed dark scene?

## Method

- Device: Intel RealSense D405, firmware 5.17.0.10, USB 2.1
- Capture: 640 x 480 at 5 FPS
- Warm-up: 15 frames
- Measurement: 30 consecutive frames (11.67 seconds)
- Accepted depth interval: 0.10 to 3.00 m
- RGB and raw aligned depth were measured before software RGB enhancement.
- Temporal MAD is the median absolute deviation of each pixel over time.

The repeatable capture tool is `tools/depth_dark_benchmark.py`. The complete
machine-readable result is `dark_depth_spatial_2026-08-13.json`.

## Results

| Metric | Result |
|---|---:|
| RGB mean luminance (0-255) | 12.16 |
| RGB median luminance (0-255) | 7.0 |
| Nonzero depth pixels per frame | 5.70% |
| In-range depth pixels per frame | 4.96% |
| Pixels valid in at least 80% of frames | 2.45% |
| Stable-pixel temporal MAD, median | 1.40 mm |
| Stable-pixel temporal MAD, 95th percentile | 5.65 mm |
| Consecutive-frame absolute delta, median | 2.00 mm |
| Consecutive-frame absolute delta, 95th percentile | 21.70 mm |
| Median stable depth | 0.354 m |

Mean in-range depth coverage by spatial cell:

| | X0 | X1 | X2 | X3 |
|---|---:|---:|---:|---:|
| Y0 | 3.92% | 1.20% | 0.68% | 0.40% |
| Y1 | 4.89% | 1.27% | 1.21% | 0.99% |
| Y2 | 3.38% | 1.02% | 16.68% | 26.84% |
| Y3 | 2.54% | 2.37% | 5.35% | 6.55% |

## Evidence-based conclusion

The depth values that remain continuously available are locally stable, but
their spatial coverage is insufficient. Only 2.45% of image pixels remain
valid in at least 80% of frames. The small difference between nonzero coverage
(5.70%) and accepted-range coverage (4.96%) shows that the dominant failure is
sensor dropout, not merely objects lying beyond the selected 3 m range.

Therefore the current dark-scene depth stream must not be treated as a dense or
globally reliable primary source. It can be used conditionally inside a target
ROI only after passing explicit coverage, freshness, and temporal-consistency
gates. The best 4x4 cell still provides only 26.84% coverage, which is below a
reasonable dense-segmentation threshold.

This experiment measures availability and repeatability, not absolute accuracy.
Absolute accuracy requires a planar reference target at measured distances.

## Required follow-up controls

1. Repeat the identical capture with the room or task light on.
2. Repeat with an external structured-IR illuminator, if depth must work while
   visible light remains off.
3. Place a matte planar reference at 0.20, 0.30, and 0.40 m and report bias,
   RMSE, 95th-percentile error, valid coverage, and temporal MAD for each range.
4. Accept depth for control only when the target ROI meets a defined coverage
   threshold and robust depth dispersion threshold.
