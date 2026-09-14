# BOXER-only RGB-D experiment on AGX

## Automatic YOLOX + Barath19 Boxer3D ONNX

The separate automatic launch uses the existing fast C++ TensorRT YOLOX detector
to provide up to three 2D boxes directly to Barath19's exported BoxerNet ONNX:

```bash
source /opt/ros/humble/setup.bash
source /home/kublab/ros2_ws/install/setup.bash
ros2 launch om6dof_boxer yolox_boxer3d_onnx.launch.py
```

No manual rectangle is required. Use `target_classes:=bottle` to process only
bottles; the empty default processes the three highest-confidence YOLO classes.
The BOXER RGB window shows the automatic YOLO rectangles and RViz shows the full
depth cloud, accepted 3D boxes and points inside them. A single process owns the
D435i. It publishes one frozen RGB frame to YOLOX and waits for the result before
running Boxer3D, so every 2D prompt uses the paired RGB, depth and camera
orientation from the same snapshot.

This path preserves the full 640x480 RGB field of view using a square letterbox,
rather than cropping it. On this AGX, BoxerNet ONNX Runtime CPU measured about
4.94-5.01 s per three-box standalone batch and about 6.9-9.9 s with the live
camera and RViz, versus 10.67-10.77 s for the earlier Meta PyTorch path. It is
automatic and faster, but still not real-time.
TensorRT 10.3 currently rejects the exported BoxerNet RoPE `If` node; the launch
does not claim GPU acceleration. The existing YOLOX stage remains real-time.

The exported model placed a live bottle box roughly 3-6 cm behind its observed
surface, leaving an empty 3D cut. The adapter therefore keeps the learned box
size and yaw but anchors its front face to the median depth in the central 60%
of the matching YOLO ROI. This correction is applied only with at least 30 depth
samples, depth spread <=8 cm and correction <=12 cm. The raw and corrected
centres are both recorded. A recorded live bottle changed from 0 to 5,676
included points with a 3.24 cm correction; no confidence gate is relaxed.

The older `boxer_only.launch.py` below is a separate manual-prompt experiment
using Meta's official pretrained
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

## Barath19 automatic-path validation (2026-09-14)

- `BoxerNet.onnx` SHA256:
  `bef64dc264047f88f3a80e00d649950598b691dd86d7e5ef84d7a9f80e1b6551`.
- `yolo11n.onnx` SHA256:
  `5f4b32238131e391172e8420f206193a50e60ef0217d873e3955d7b08a3a3869`.
- TensorRT 10.3 parses the YOLO model. BoxerNet parsing fails at
  `/rope_embed/If` because its branches return incompatible shapes `[2]` and
  `[1]`; ONNX Runtime CPU is used deliberately.
- Fourteen package tests pass, including YOLO timestamp/filtering, full-frame
  letterbox geometry, sparse projected depth, OBB geometry and bounded depth
  anchoring.
- Live bottle detections scored about 0.944-0.949. With depth anchoring, repeated
  accepted boxes included roughly 5,200-5,700 observed points.

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
