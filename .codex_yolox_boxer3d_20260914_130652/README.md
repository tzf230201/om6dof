# BOXER-only RGB-D experiment on AGX

This is a separate manual-prompt experiment using Meta's official pretrained
[BOXER](https://github.com/facebookresearch/boxer) and DINOv3 checkpoints.
It does not launch YOLO, DD-GNG, robot controllers, motion planning, gripper
commands, or shared robot TF. A read-only V2 state publisher supplies camera
orientation on private `/boxer/tf` topics from real `/joint_states`; it does not
start controllers or fabricate joint positions. Existing experiment/paper data are untouched.

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

1. RGB opens and observed depth is previewed in RViz before inference (about 1 Hz).
   Keep the camera still. SPACE freezes RGB, depth and camera orientation together.
   Depth/RGB must share a timestamp domain and differ by <=50 ms.
2. Draw the **bottle** rectangle in the same RGB window, then click **Konfirmasi**
   below the image (Enter also works). Repeat for **keyboard** when prompted.
   Confirming the last label starts inference immediately; no Esc is needed.
   **Gambar ulang** clears the pending rectangle. Esc cancels selection.
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

`gravity_source:=auto` uses an advertised accelerometer stream at a supported rate
when available. This D435i currently exposes only RGB and depth, so auto selects TF.
A private read-only V2 state publisher uses actual `/joint_states`; camera orientation
is looked up at the RGB host-receipt timestamp (not a stale/latest replacement).
Missing timestamped TF blocks capture and is displayed in the RGB HUD.
No calibration verification flag is bypassed; the TF orientation is nominal URDF,
not a new hand-eye calibration. Optional `gravity_source:=imu` requires a real
advertised IMU; `gravity_source:=tf` forces the TF path. Use
`publish_robot_state:=false` only when another valid source supplies `/boxer/tf`.
The IMU path uses stationary specific force and factory IMU-to-color rotation.
Snapshot axes: +Z up, +X approximately camera
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

## Startup fix and live verification (2026-09-14)

The first live launch failed because it requested an accelerometer unavailable in
the device's advertised profiles. After removing that unconditional request, two
further compatibility problems were reproduced and fixed: inverse Brown RGB with
all-zero coefficients (equivalent to pinhole), and GTK reporting unsupported
WND_PROP_VISIBLE as -1 (not a closed window).

Six geometry/profile tests pass. Startup now uses the actual RGB-D profiles and
TF gravity fallback; the RGB window remained open and a read-only subscriber
received 99,222 real depth points in boxer_local. No robot movement was commanded.
Bounding boxes still require manual rectangle prompts and inference. A preview
process exit now shuts down its own RViz instead of leaving an unexplained empty
window, and Python output is unbuffered and logged to both terminal and launch log.
