# BOXER-only RGB-D experiment on AGX

This is a separate manual-prompt experiment using Meta's official pretrained
[BOXER](https://github.com/facebookresearch/boxer) and DINOv3 checkpoints.
It does not launch YOLO, DD-GNG, robot controllers, motion planning, gripper
commands, or a world TF publisher. Existing experiment/paper data are untouched.

BOXER lifts 2D rectangle prompts to oriented 3D boxes. It does **not** produce an
instance segmentation mask. The preview cuts observed depth points by membership
inside each predicted OBB, and reports how many points belong to overlapping OBBs.
Points may still include background. A prompt's name is supplied by the operator;
BOXER is not classifying the object as bottle/keyboard in this manual mode.

## Run

Stop the old **perception/camera** launch with Ctrl-C first; D435i must have only
one owner. Leave controller services alone. From the AGX desktop terminal:

```bash
source /opt/ros/humble/setup.bash
source /home/kublab/ros2_ws/install/setup.bash
ros2 launch om6dof_boxer boxer_only.launch.py labels:=bottle,keyboard
```

1. Keep the camera still. SPACE freezes RGB, depth and a recent stationary IMU
   reading together. Depth/RGB timestamp difference must be <=50 ms.
2. Draw the **bottle** rectangle; Enter. Draw the **keyboard** rectangle; Enter.
   Press Esc to finish selecting rectangles. The number/order must match labels.
   Overlapping input rectangles are allowed: that is the proposed experiment.
3. Wait for BOXER. The frozen RGB shows projected 3D box edges. RViz shows box
   edges, grey observed depth, and colored depth points inside the boxes.
4. N returns to the live camera, then SPACE captures again. Q closes the preview.
   Closing during inference waits for that snapshot's CPU work to finish.

Use `labels:=bottle` for one rectangle. `launch_rviz:=false` hides RViz.
The preview is deliberately snapshot-based. Default device is CPU because the
pre-existing global PyTorch installation cannot import; a separate CPU venv was
installed without altering that environment. `device:=cuda` requires a compatible
Jetson CUDA PyTorch in the selected `boxer_python` and fails clearly if unavailable.

## Geometry and outputs

RGB remains at its original 640x480 field of view (pinhole rectification for lens
distortion only); it is not aligned/resampled into the depth camera's view. Native
depth points are deprojected by librealsense, then transformed to the RGB optical
frame with factory depth-to-color extrinsics. The actual model input is resized
to the checkpoint's 960x960, with intrinsics and 2D prompts scaled consistently.

The D435i accelerometer's stationary specific-force vector supplies local up,
using factory IMU-to-color rotation. Snapshot axes: +Z up, +X approximately camera
right, +Y approximately camera forward. `boxer_local` is **not robot world**;
its origin is the captured camera center and resets per snapshot. Each new result
replaces the previous scene. This first experiment does not perform multi-view
tracking/fusion or certify IMU/extrinsic accuracy for robot execution.

Topics (latched, standalone namespace):
- `/boxer/boxes`: predicted OBB wireframes and manual labels.
- `/boxer/points`: observed depth in the local frame.
- `/boxer/cut_points`: observed points inside accepted boxes, colored per prompt.

Outputs default to `~/ros2_ws/src/.boxer_runtime/outputs/capture_<timestamp>/`:
`input.npz`, `boxes.json`, `boxer_overlay.png`, and `cut_<index>.npy` in metres.
Confidence acceptance is >=0.5 from the official demo default. Empty cuts and
3D box overlaps are reported; they are not replaced with invented depth.

Replay a saved snapshot without the camera (same GUI):

```bash
ros2 launch om6dof_boxer boxer_only.launch.py \
  snapshot:=/absolute/path/input.npz labels:=bottle,keyboard
```

## Installation / provenance

Runtime is isolated in `~/ros2_ws/src/.boxer_runtime/` (COLCON_IGNORE):
- `boxer/`: official source, archive SHA256 and checkpoint SHA256 recorded in
  `PROVENANCE.json` alongside that directory.
- `boxer/ckpts/`: BoxerNet and DINO checkpoints; no OWLv2 needed for manual prompts.
- `venv/`: Python 3.10 system-site-packages venv with torch 2.6.0+cpu.

The upstream README tested Python 3.12 on macOS/Fedora; this Jetson Python 3.10 CPU
adapter is a local compatibility test. Upstream models/source retain Meta's
CC-BY-NC terms and component notices; see their LICENSE and NOTICE files.

## Validation on 2026-09-14

- Official weights loaded: 320/320 tensors, 85.81M/85.81M parameters.
- Three geometry tests passed: gravity rotation/roundtrip, oriented cropping,
  and non-square source-image ROI scaling/validation.
- Real model inference executed on a **synthetic** RGB/depth snapshot: 12.68 s on
  CPU; output JSON, projected image and crop file written. This is a software
  smoke test, not bottle accuracy, camera calibration or hardware validation.
- ROS package built. Live RealSense capture was not started while the user's
  existing DD-GNG camera pipeline was active.
