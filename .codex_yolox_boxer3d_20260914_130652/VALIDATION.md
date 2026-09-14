# YOLOX -> Barath19 Boxer3D validation (2026-09-14)

Scope: standalone perception and RViz only. No controller restart, torque command,
motion command, DD-GNG modification, or experiment/paper output modification.

## Installed runtime

- Barath19/Boxer3D source archive SHA256:
  `b1c6924b2dd1c6a1fa2db53a1b2def476ed698b6525f0f63323a73913abff584`
- `BoxerNet.onnx` SHA256:
  `bef64dc264047f88f3a80e00d649950598b691dd86d7e5ef84d7a9f80e1b6551`
- `yolo11n.onnx` SHA256:
  `5f4b32238131e391172e8420f206193a50e60ef0217d873e3955d7b08a3a3869`
- Runtime: isolated ONNX Runtime CPU environment under `.boxer3d_runtime/`.
- Existing C++ TensorRT YOLOX is used for automatic 2D prompts.

TensorRT 10.3 parsed `yolo11n.onnx`. It rejected `BoxerNet.onnx` at
`/rope_embed/If` because its branches return incompatible shapes `[2]` and `[1]`.

## Checks

- `colcon build --packages-select om6dof_boxer --symlink-install`: PASS.
- Package tests: 14 PASS.
- Python compilation: PASS.
- Git whitespace/error check: PASS.
- Automatic snapshot transaction: one RGB-depth-TF snapshot is published to
  YOLOX and its exact timestamp is required before Boxer3D starts.

## Live D435i evidence

Launch filter: `target_classes:=bottle`.

Five consecutive automatic results completed without a manual rectangle or
Enter key:

| Boxer confidence | Included depth points | Inference time |
| ---: | ---: | ---: |
| 0.9458 | 5,458 | 8.640 s |
| 0.9485 | 5,668 | 8.638 s |
| 0.9494 | 5,497 | 8.849 s |
| 0.9477 | 5,722 | 8.311 s |
| 0.9460 | 5,689 | 8.860 s |

A later saved result recorded 3,168 stable central-ROI depth samples, 9.0 mm
depth spread, a bounded -34.0 mm front-face correction, and 5,463 included
points. Both raw and corrected centres are retained in `boxes.json`.

This validates software flow and live visualization. It does not validate object
pose accuracy for grasping or authorize robot execution.
