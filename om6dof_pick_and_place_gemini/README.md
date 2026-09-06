# om6dof_pick_and_place_gemini

OM6DOF port of the ROBOTIS OMY technical story
[*GraspNet pick and place*](https://docs.robotis.com/docs/systems/omy/resources/technical_story/graspnet_pick_and_place/):
grasp poses come out of a point cloud instead of a fiducial, and Google Gemini
does the reasoning — either naming what was picked, or finding the object you
described in plain text.

> **Control-profile compatibility:** like the other pick packages this needs
> MoveIt and the normal position-control hardware profile. It cannot share the
> U2D2 or the arm command interfaces with the Mode 0 leader stack. See the
> [leader-arm gravity-compensation record](../docs/leader_arm_gravity_compensation.md).

> **Camera ownership:** with `camera_source: realsense` this node opens the
> wrist D405 itself, and a RealSense allows exactly one owner. Stop
> `om6dof_perception`, `apriltag_detector`, `om6dof-dd-gng`, and their systemd
> units first, or set `camera_source: topic` and let one driver own the device.

---

## Pipeline

```
              wrist RealSense D405 (colour aligned into depth)
                              |
   rgbd_source.py             v   point_cloud()
                    optical-frame cloud + per-point source pixel
                              |
   transforms.py              v   world <- FK(joint_states) <- EE <- camera
                       cloud in the arm base frame
                              |
   grasp_backends.py          v   analytic | graspnet-baseline | AnyGrasp
              GraspCandidate: position + approach + closing + width + score
                              |
   gemini_client.py           v   mode "target": locate("the red cup") -> pixel
                       candidates narrowed to that object
                              |
   grasp_filter.py            v   width, clearance, tilt, workspace, collision, IK
                       accepted candidates, best score first
                              |
   gemini_pick_node.py        v   MoveGroup + GripperCommand
        pregrasp -> grasp -> close -> lift -> place-approach -> place -> home
                              |
   gemini_client.py           ^   mode "classify": classify(crop) -> place bin
```

### What differs from the OMY original

| OMY story | here | why |
|---|---|---|
| `MoveL` commands over Zenoh to Cyclo Control | `moveit_msgs/MoveGroup` + `control_msgs/GripperCommand` | this arm plans through MoveIt; there is no MoveL controller |
| GraspNet-baseline on a GPU workstation, Docker | `graspnet` or licensed `anygrasp` on this Jetson; `analytic` remains a fallback | the CUDA runtime lives in an isolated prefix; see the setup paths below |
| user PC + robot, two machines | one node on the robot | the D405 is on this wrist |
| `omy_ai_graspnet_places.yaml` | [`config/places.yaml`](config/places.yaml) | same idea, same file-order-is-fallback-order rule |

---

## Quick start

```bash
# 1. key, once (chmod 600 — it is never read from a params file)
mkdir -p ~/.config/om6dof
printf '%s\n' 'YOUR_GEMINI_API_KEY' > ~/.config/om6dof/gemini_api_key
chmod 600 ~/.config/om6dof/gemini_api_key

# Every GraspNet ROS terminal: source the isolated Jetson runtime after ROS.
source /opt/ros/humble/setup.bash
source /home/kublab/ros2_ws/install/setup.bash
source /mnt/agx_nvme/om6dof-graspnet-jp622/activate_om6dof_graspnet.sh

# Keep the CLI daemon on the same DDS implementation as the launch.  A daemon
# left running under Cyclone DDS can make services appear to hang even though
# the Fast DDS node has already completed the callback.
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset CYCLONEDDS_URI
ros2 daemon stop
ros2 daemon start

# The pick launch starts MoveGroup, not ros2_control or robot_state_publisher.
# The existing hardware service must already own the arm and publish state.
systemctl is-active om6dof-hardware.service
ros2 action info /arm_controller/follow_joint_trajectory
ros2 action info /gripper_controller/gripper_cmd
ros2 topic echo --once --qos-durability transient_local \
    /om6dof/operation_mode/state       # must say AUTONOMOUS
ros2 topic echo --once --qos-durability transient_local \
    /om6dof/remote_enabled/state       # must say false

# These direct-camera services must be inactive before this node owns the D405.
systemctl is-active om6dof-dd-gng.service om6dof-perception.service \
    apriltag-detector.service          # each should report inactive

# 2. check the key against a test card, no camera needed
ros2 run om6dof_pick_and_place_gemini gemini_probe

# 2b. or against a live frame straight off the USB RealSense — no ROS node,
#     no arm needed. Only one process may hold the camera at a time.
ros2 run om6dof_pick_and_place_gemini gemini_probe --realsense
ros2 run om6dof_pick_and_place_gemini gemini_probe --realsense \
    --describe "the coffee bottle"          # try locate() instead of classify()
ros2 run om6dof_pick_and_place_gemini gemini_probe --realsense --save /tmp/frame.jpg
ros2 run om6dof_pick_and_place_gemini gemini_probe --realsense --serial 427622271962
                                              # pick a specific camera by serial

# 3. perception-only — capture/detect/filter/publish markers, no MoveIt request
ros2 launch om6dof_pick_and_place_gemini gemini_pick.launch.py \
    execute_motion:=false place_enabled:=false
ros2 service call /gemini_pick/preflight std_srvs/srv/Trigger
ros2 service call /gemini_pick/perceive std_srvs/srv/Trigger

# 4. full MoveIt plan-only smoke — plans the whole pick path; no gripper goal
#    (the launch starts MoveIt by default; use start_moveit:=false only when an
#    existing move_group already owns it)
ros2 service call /gemini_pick/run std_srvs/srv/Trigger

# 5. Stop the plan-only launch, then start one physical pick/lift launch only
#    after every calibration gate below has passed.
ros2 launch om6dof_pick_and_place_gemini gemini_pick.launch.py \
    execute_motion:=true calibration_validated:=true \
    gripper_width_at_open_pos:="$OM6_GRIPPER_OPEN_WIDTH_M" \
    gripper_width_at_close_pos:="$OM6_GRIPPER_CLOSE_WIDTH_M" \
    gripper_calibration_validated:=true place_enabled:=false
ros2 service call /gemini_pick/run std_srvs/srv/Trigger
```

`~/perceive` never asks MoveIt to plan. `~/run` with
`execute_motion:=false` is the real plan-only check: it sends planning requests
for the observe, pregrasp, grasp and lift chain while keeping controller
execution off. Dry-run jaw positions are still carried into subsequent
collision checks, but no gripper action is sent. A successful `~/perceive` is
therefore necessary, but is not evidence that the motion path plans.

### Inspect RGB-D and world coordinates (vision only)

To validate the camera calibration before detection, launch the viewer below.
It opens the D405 directly, displays aligned RGB and colour-mapped depth, and
draws a crosshair at the centre pixel. The overlay reports that pixel's depth,
optical-camera XYZ, and `world` XYZ transformed through TF at the exact capture
time. Press `q` or `Esc` to close. It never creates an arm or gripper client.

```bash
# Stop every other direct D405 owner first (perception, dd-gng, AprilTag).
ros2 launch om6dof_pick_and_place_gemini rgbd_viewer.launch.py
```

If `World: N/A` appears, the RGB-D display is still valid but the TF chain from
`world` to `d405_depth_optical_frame` is missing or not fresh. Start/restart
the hardware `robot_state_publisher`; do not treat a missing TF as a usable
world coordinate.

### Verify Gemini target ↔ learned-grasp candidates (vision only)

The separate target viewer verifies identity before any motion. It streams the
RGB-D cloud only to RViz. When `~/run` is called, it sends the requested
description to Gemini and extracts the connected RGB-D component around its
target point for the yellow overlay and target association. With `graspnet`,
the network runs on the complete reachable-workspace scene and target
association happens afterward. With `anygrasp`, the complete post-self-
exclusion scene remains both the inference and collision scene, while that
exact component is supplied as a point-for-point region-steering mask. Neither
backend inflates the component into a synthetic fixed-size target cloud.
Candidates are filtered by the OM6DOF jaw width, table clearance, approach
angle, workspace, target bounds, and full-scene gripper envelope.
IK is deliberately not run in this vision-only viewer and nothing is sent to
MoveIt.

```bash
# Close rgbd_viewer and any other D405 owner first.
ros2 launch om6dof_pick_and_place_gemini target_grasp_viewer.launch.py \
    target:="the small red cube"
```

To select the licensed AnyGrasp backend on this machine, source the same
isolated CUDA runtime used below and add `backend:=anygrasp`:

```bash
source /opt/ros/humble/setup.bash
source /home/kublab/ros2_ws/install/setup.bash
source /mnt/agx_nvme/om6dof-graspnet-jp622/activate_om6dof_graspnet.sh

ros2 launch om6dof_pick_and_place_gemini target_grasp_viewer.launch.py \
    backend:=anygrasp target:="the small red cube"
ros2 service call /target_grasp_viewer/run std_srvs/srv/Trigger
ros2 service call /target_grasp_viewer/status std_srvs/srv/Trigger
```

This viewer is the recommended first AnyGrasp test: it creates no MoveIt,
arm, or gripper client. `backend:=anygrasp` only chooses perception; it never
enables robot motion.

This launch opens RViz as well. It shows the robot URDF from the existing
`/robot_description` publisher, colourised D405 cloud in `world`, and the
segmented target in yellow. Like ROBOTIS target mode, RViz shows only the valid
grasp selected after the safety filters. The selector ranks every safe survivor
by minimum full 3-D orientation change from `end_effector_link` at capture
time, while accounting for the equivalent finger-swapped orientation of a
parallel gripper. GraspNet score and then target distance break orientation
ties; score never excludes a safer-orientation survivor after `min_score` has
passed. If the tool TF is unavailable, the viewer falls back to the
score-band/near-vertical preference. The result is one bold,
fully opaque red parallel-jaw glyph with two fingers, palm, and rear approach stem.
A final IK/reachability check still belongs to the pick node, not this viewer.
`gemini_target_pick.launch.py` reuses this same scene-wide inference, component
association, target-bounds gate, and capture-time orientation selector
before adding that IK check; it does not independently choose the top score.
The RViz topics are `/target_grasp_viewer/world_cloud`,
`/target_grasp_viewer/target_cloud`, and `/target_grasp_viewer/markers`.

Use an English visual description. To change it without restarting:

```bash
ros2 topic pub --once /target_grasp_viewer/set_target std_msgs/msg/String \
    "{data: 'the blue cup on the right'}"
```

Run one target inference after changing target:

```bash
ros2 service call /target_grasp_viewer/run std_srvs/srv/Trigger
ros2 service call /target_grasp_viewer/status std_srvs/srv/Trigger
```

The status reports Gemini confidence and box, segmented point count, and raw,
valid, and rejected candidate counts with rejection reasons. `table_z` must be
the measured surface directly under the target in `world`. In the motion node,
that same value is `target_support_z` when an elevated support is enabled;
`table_z` remains the lower/main table.

This viewer has no 2-D OpenCV window, arm/gripper/MoveIt client, and cannot
execute a pick.

Ask for a specific object with the motion node only after closing the viewer
(both directly own the D405). First run the complete target path plan-only. Set
the geometry variables below from physical measurements in `world`; do not copy
values from another setup. If the object is directly on the main table, leave
`target_support_enabled:=false` and omit all five support box arguments.

```bash
OM6_MAIN_TABLE_Z_M="REPLACE_WITH_MEASURED_METRES"
OM6_SUPPORT_TOP_Z_M="REPLACE_WITH_MEASURED_METRES"
OM6_SUPPORT_SIZE_X_M="REPLACE_WITH_MEASURED_METRES"
OM6_SUPPORT_SIZE_Y_M="REPLACE_WITH_MEASURED_METRES"
OM6_SUPPORT_SIZE_Z_M="REPLACE_WITH_MEASURED_METRES"
OM6_SUPPORT_CENTER_X_M="REPLACE_WITH_MEASURED_METRES"
OM6_SUPPORT_CENTER_Y_M="REPLACE_WITH_MEASURED_METRES"
ros2 launch om6dof_pick_and_place_gemini gemini_target_pick.launch.py \
    target:="the red screwdriver" table_z:="$OM6_MAIN_TABLE_Z_M" \
    target_support_enabled:=true \
    target_support_z:="$OM6_SUPPORT_TOP_Z_M" \
    target_support_collision_size_x:="$OM6_SUPPORT_SIZE_X_M" \
    target_support_collision_size_y:="$OM6_SUPPORT_SIZE_Y_M" \
    target_support_collision_size_z:="$OM6_SUPPORT_SIZE_Z_M" \
    target_support_collision_center_x:="$OM6_SUPPORT_CENTER_X_M" \
    target_support_collision_center_y:="$OM6_SUPPORT_CENTER_Y_M" \
    execute_motion:=false place_enabled:=false
ros2 service call /gemini_pick/preflight std_srvs/srv/Trigger
ros2 service call /gemini_pick/perceive std_srvs/srv/Trigger
ros2 service call /gemini_pick/run std_srvs/srv/Trigger
ros2 service call /gemini_pick/status std_srvs/srv/Trigger

# or retarget while it runs
ros2 topic pub --once /gemini_pick/set_target std_msgs/msg/String "{data: 'the blue cup'}"
```

Only after that run reaches `plan-only pick path succeeded`, stop the plan-only
launch and start the first physical pick with `execute_motion:=true`,
`calibration_validated:=true`, `gripper_calibration_validated:=true`, and
both measured `gripper_width_at_*_pos` values supplied as described in Gate 2;
keep `place_enabled:=false`. It executes
pregrasp → linear grasp → close → linear lift and leaves the object held.

The supplied MoveIt RViz configuration displays the live coloured cloud on
`/gemini_pick/world_cloud`, the yellow Gemini component on
`/gemini_pick/target_cloud`, and the single selected parallel-gripper glyph on
`/gemini_pick/grasp_markers`, all in the `world` frame. If no candidate passes,
the enabled `Best Near Miss (NOT EXECUTABLE)` layer shows one rejected proposal
from `/gemini_pick/near_miss_markers`; it is diagnostic output and can never be
sent to motion. The noisy all-candidate debug layer is disabled by default.
The cloud preview runs at `cloud_preview_hz` without invoking Gemini, GraspNet,
or motion; call `~/perceive` or `~/run` when a new grasp result is wanted.
`graspnet_sampling_attempts` (default `3`) pools successive deterministic
samples from the reachable-workspace scene before the unchanged target, collision, tilt,
workspace, and IK filters select one pose.

A successful plan-only `~/run` also publishes the complete chosen
pregrasp → grasp → lift/retreat chain as one `/display_planned_path` message.
RViz loops that result as a sparse, opaque green robot trail over a dimmed world
cloud. No grasp trail is published when every candidate is rejected or when a
later segment fails: showing a partial/unsafe chain as though it were selected
would be misleading. In that case inspect `~/status` and the labelled
`Best Near Miss (NOT EXECUTABLE)` marker instead.

The upstream learned-backend collision test is followed by an OM6DOF-specific,
target-aware check against the full world cloud. Stable capture-row IDs mark the
exact segmented target; only those rows may contact the inner fingers during
final closure. Other objects in that closure band, or any point in the swept
open fingers, palm/rear, or beyond the clear open aperture, reject the grasp.
This prevents target segmentation from making its support or a neighbouring
object invisible to collision checking.

An executing node requires the measured `gripper_width_at_open_pos` and the
existing calibration acknowledgement. Plan-only mode and the vision-only
viewer remain usable before measurement by assuming configured `max_width`,
but status calls label that geometry `ASSUMED PREVIEW ... NOT EXECUTABLE`; it is
never reused by physical execution.

---

## The API key

Never put the key in `config/gemini_pick.yaml`. A params file is copied into the
build tree, shows up in `ros2 param dump`, and is easy to commit by accident.
The client looks in this order:

1. the `gemini_api_key` parameter (kept for one-off tests; a `<placeholder>` counts as unset)
2. `$GEMINI_API_KEY` — the same variable the OMY story uses
3. `~/.config/om6dof/gemini_api_key`, first line — **the recommended spot**

With no key the node still starts and still picks; classification returns the
last category from `place_categories` and `mode: target` refuses to run with a
clear message rather than failing mid-sequence.

### Which model

`gemini-2.5-flash` is no longer served to new keys — it 404s with a message
telling you to move on. The default is now **`gemini-3.5-flash-lite`**, picked
for its free-tier quota headroom during development, not because it is the
sharpest option. To see what a key can actually use, and its live quota:

```bash
curl -s -H "x-goog-api-key: $(cat ~/.config/om6dof/gemini_api_key)" \
  https://generativelanguage.googleapis.com/v1beta/models | grep '"name"'
# rate limits: https://aistudio.google.com -> your API key -> Rate limits
```

Checked against this key on 2026-09-02 with `gemini_probe` (free tier, RPM =
requests/minute, RPD = requests/day):

| model | RPM / RPD | result |
|---|---|---|
| `gemini-3.5-flash-lite` | 15 / 500 | both prompts fine — **the default**, most headroom for iterating |
| `gemini-3.6-flash` | 5 / 20 | a bit sharper, tight quota |
| `gemini-robotics-er-2-preview` | 5 / 20 | embodied-reasoning model; put the point dead on the probe target. Worth trying for `mode: target` specifically, not as the default — same tight quota as 3.6-flash |
| `gemini-3.7-flash` | — | read timeout at 20 s — raise `gemini_timeout_sec` before trying it again |

At 20 requests/day, a `3.6-flash` or `robotics-er` quota disappears after about
ten `~/run` cycles (each does a locate/classify call or two). `flash-lite`'s 500
RPD is the one that survives a real testing session; switch back to a sharper
model for a low-volume production run if accuracy matters more than headroom.

A 404, timeout, or 429 surfaces as a clear error. In `target` mode it stops
before approaching an object (the arm may already be at the observation pose).
In `classify` mode, which asks Gemini only after lift, the configured `unknown`
category is used; keep place disabled until that fallback behavior is intended.

`gemma-4-31b-it` (30 RPM / 14.4K RPD — the loosest quota on this key) also
works, tested against a live D405 frame, but it is **not recommended as the
default**: unlike the Gemini models it keeps narrating its reasoning before the
JSON answer even with `responseMimeType: application/json` set, which costs
extra tokens and latency per call, and it exposed a real bug — a greedy
first-brace-to-last-brace regex would splice the *template* JSON it quotes
while reasoning ("JSON format: `{"label": ...}`") onto the *real* answer,
producing invalid JSON. Fixed in `parse_json_payload` (balanced-bracket
scanning, last parseable object wins) so any model that reasons out loud before
answering is now handled correctly — but a Gemini model remains the steadier
choice day to day.

---

## Grasp backends

### `analytic` (lightweight fallback)

Pure numpy, no GPU. Removes the table plane, clusters what remains on a 6 mm
voxel grid, and for each cluster proposes a grasp across its narrow horizontal
axis, at `grasp_depth` below the object's top. Each cluster yields one candidate
per entry in `approach_tilts` — straight down first, then progressively tilted
away from the base, because this arm cannot reach pitch π at every radius (the
IK map in `om6dof_pick_and_place/config/tag_pick.yaml` documents the same
limitation). The reachability filter picks whichever tilt this pose actually
allows.

It handles separated objects on a table, which is what the OMY story
demonstrates. It does not reason about clutter, occlusion or stacked objects the
way a learned model does.

### `graspnet` (default on this Jetson; setup is explicit and isolated)

[graspnet-baseline](https://github.com/graspnet/graspnet-baseline) is the model
the ROBOTIS story runs. Use
[`scripts/setup_graspnet_jetson.sh`](scripts/setup_graspnet_jetson.sh) only on
the intended stack: **JetPack 6.2.2 / L4T R36.5.x, CUDA 12.6, Python 3.10,
aarch64 Jetson Orin**. It uses the PyTorch 2.8 wheel posted in the
[NVIDIA JetPack 6.2 forum thread](https://forums.developer.nvidia.com/t/pytorch-2-8-wheel-for-jetpack-6-2/341339),
the upstream [GraspNet API](https://github.com/graspnet/graspnetAPI), and the
official upstream RealSense
[`checkpoint-rs.tar`](https://drive.google.com/file/d/1hd0G8LN6tRpi4742XOTEisbTXNZ-1jmk/view?usp=sharing).

The script deliberately has no implicit destination. Its default is a
read-only preflight and plan:

```bash
cd /home/kublab/ros2_ws/src/om6dof/om6dof_pick_and_place_gemini

# Read-only: checks aarch64, Ubuntu/L4T/JetPack, Python 3.10, CUDA 12.6,
# ROS Humble, compiler tools, system cv2, and at least 12 GiB free.
bash scripts/setup_graspnet_jetson.sh \
    --prefix /mnt/agx_nvme/om6dof-graspnet-jp622 \
    --preflight-only

# Review the paths and sources printed above. This is the first mutating step;
# it writes only below the explicit prefix and never invokes a system package
# manager or touches ~/.local.
bash scripts/setup_graspnet_jetson.sh \
    --prefix /mnt/agx_nvme/om6dof-graspnet-jp622 \
    --install
```

Installation creates a Python 3.10 venv with `--system-site-packages`, verifies
the downloaded PyTorch wheel's published SHA-256, clones both upstream repos,
builds `pointnet2` and `knn` for Orin `sm_87`, downloads the official
RealSense checkpoint via `gdown` (with an HTTPS `curl` fallback), and then
checks all of the following before reporting success:

- PyTorch 2.8 sees CUDA 12.6 and executes a CUDA tensor operation;
- both compiled extensions import;
- the isolated `pyrealsense2` binding imports without exposing the broken
  user-site Torch installation;
- upstream `GraspNet`, `pred_decode`, `GraspGroup`, and the collision detector
  import;
- the checkpoint loads with restricted `weights_only=True` deserialization and
  matches the model state exactly;
- `/usr/bin/python3`, the interpreter normally embedded in ROS Python
  entrypoints, sees the same isolated runtime with user-site disabled.

The last point matters here: merely activating a venv does not change an
already-generated ROS console script's shebang. Source ROS and the workspace
first, then source the generated helper; it prepends the venv site-packages and
the two repos to `PYTHONPATH` while filtering explicit `~/.local` entries:

```bash
source /opt/ros/humble/setup.bash
source /home/kublab/ros2_ws/install/setup.bash
source /mnt/agx_nvme/om6dof-graspnet-jp622/activate_om6dof_graspnet.sh

# This intentionally tests the system interpreter used by a ROS entrypoint.
/usr/bin/python3 -s -c \
  'import torch, pointnet2._ext, knn_pytorch.knn_pytorch; print(torch.__file__, torch.cuda.is_available())'

# Read-only repeat of the complete import/CUDA/checkpoint verification.
bash /home/kublab/ros2_ws/src/om6dof/om6dof_pick_and_place_gemini/scripts/setup_graspnet_jetson.sh \
    --prefix /mnt/agx_nvme/om6dof-graspnet-jp622 \
    --verify-only
```

The shipped config now points to this robot's installed runtime:

```yaml
graspnet_repo_path: "/mnt/agx_nvme/om6dof-graspnet-jp622/src/graspnet-baseline"
graspnet_checkpoint: "/mnt/agx_nvme/om6dof-graspnet-jp622/checkpoint-rs.tar"
```

The launch also accepts `graspnet_repo_path:=...` and
`graspnet_checkpoint:=...`; their defaults come from the activation helper's
environment and fall back to the paths above. Then start with plan-only, not
controller execution:

```bash
ros2 launch om6dof_pick_and_place_gemini gemini_pick.launch.py \
    execute_motion:=false place_enabled:=false
ros2 service call /gemini_pick/run std_srvs/srv/Trigger
```

`versions.txt` under the prefix records both git commits, the wheel URL/hash,
and the locally calculated checkpoint hash. Upstream GraspNet assets are for
non-commercial use; review their repository license before redistribution.
Everything downstream — filtering, Gemini, motion — remains backend-agnostic.
`GraspNetBackend.available()` reports a concrete missing/import error at
startup instead of crashing the node.

### `anygrasp` (licensed, selectable; GraspNet remains the default)

The licensed [AnyGrasp SDK](../../anygrasp_sdk/) is available on this robot and
can be selected with `backend:=anygrasp`. It deliberately is not the default
yet, so existing GraspNet launch behaviour does not change while AnyGrasp is
commissioned against recorded and live scenes.

The expected local-only layout is:

```text
/home/kublab/ros2_ws/src/anygrasp_sdk/grasp_detection/
  checkpoint_detection.tar
  gsnet.so
  license/
    licenseCfg.json
    ... licensed key/signature files ...
```

The checkpoint, binary, and license are machine-specific/proprietary assets:
do not commit, print, copy into logs, or redistribute them. The default launch
arguments point at this layout. If it is moved, override
`anygrasp_runtime_dir:=...`; checkpoint and license then default to
`checkpoint_detection.tar` and `license/` below that directory. They can also
be overridden independently with `anygrasp_checkpoint:=...` and
`anygrasp_license_dir:=...`.

AnyGrasp uses the existing isolated Python/CUDA environment plus its
MinkowskiEngine dependency. Source it after ROS and the workspace in every
terminal:

```bash
source /opt/ros/humble/setup.bash
source /home/kublab/ros2_ws/install/setup.bash
source /mnt/agx_nvme/om6dof-graspnet-jp622/activate_om6dof_graspnet.sh
```

For target mode, Gemini segmentation is not used as a replacement point cloud.
The backend receives the full finite, post-self-exclusion optical-frame scene
and an exact boolean `region_steering` mask for the segmented target; AnyGrasp's
collision detection therefore still sees supports and surrounding objects. An
empty or misaligned mask fails closed instead of silently asking AnyGrasp to
grasp anywhere. Downstream target-bounds, table/support clearance, workspace,
tilt, conservative swept-gripper collision, IK, and MoveIt checks remain in
force.

Start with the separate RViz viewer (no motion clients):

```bash
ros2 launch om6dof_pick_and_place_gemini target_grasp_viewer.launch.py \
    backend:=anygrasp target:="the small red cube"
ros2 service call /target_grasp_viewer/run std_srvs/srv/Trigger
ros2 service call /target_grasp_viewer/status std_srvs/srv/Trigger
```

Then test the integrated target node without controller execution:

```bash
ros2 launch om6dof_pick_and_place_gemini gemini_target_pick.launch.py \
    backend:=anygrasp target:="the small red cube" \
    execute_motion:=false place_enabled:=false
ros2 service call /gemini_pick/perceive std_srvs/srv/Trigger
ros2 service call /gemini_pick/status std_srvs/srv/Trigger
```

The shipped detector settings are `anygrasp_max_width=0.065 m`,
`anygrasp_gripper_height=0.058 m`, `top_k=50`, `dense_grasp=false`, and
`collision_detection=true`. Keep the detector width at or below the global
OM6DOF `max_width`, and do not disable collision detection for a physical run.
As with every backend, physical movement remains locked behind the independent
calibration, Dynamixel-health, full-path prevalidation, and
`execute_motion:=true` gates.

---

## Interface

### Services

| service | type | what it does |
|---|---|---|
| `/gemini_pick/run` | `std_srvs/Trigger` | the whole sequence |
| `/gemini_pick/perceive` | `std_srvs/Trigger` | capture + detect + publish markers, **no motion** |
| `/gemini_pick/preflight` | `std_srvs/Trigger` | check backend, camera/TF timestamps, joint state, MoveIt, and table scene; no trajectory execution |
| `/gemini_pick/stop` | `std_srvs/Trigger` | latch stop, cancel MoveGroup, and directly cancel the owned physical controller goal |
| `/gemini_pick/status` | `std_srvs/Trigger` | JSON: stage, mode, target, last Gemini answer, last pick |

During physical execution, `/stop`, an interlock trip, and Ctrl-C keep retrying
controller cancellation. If ROS cannot confirm that cancellation, the node
intentionally refuses automatic teardown. Use the robot's hardware emergency
stop first; only then force-kill the process after visually verifying the arm
has stopped. Plan-only runs never send this direct controller cancellation.

### Topics

| topic | type | direction |
|---|---|---|
| `/gemini_pick/set_target` | `std_msgs/String` | in — target description for `mode: target` |
| `/dynamixel_hardware_interface/health` | `diagnostic_msgs/DiagnosticArray` | in — persistent read/write failure counters and current actuator health |
| `/gemini_pick/status` | `std_msgs/String` | out — one line per stage |
| `/gemini_pick/grasp_markers` | `visualization_msgs/MarkerArray` | out — the one selected, safety-filtered grasp in `world` |
| `/gemini_pick/near_miss_markers` | `visualization_msgs/MarkerArray` | out — DEBUG ONLY: one best rejected grasp with failure evidence; never consumed by motion |
| `/gemini_pick/debug_grasp_markers` | `visualization_msgs/MarkerArray` | out — read-only non-selected grasp glyphs; never consumed by motion |

### Parameters worth knowing

Everything lives in [`config/gemini_pick.yaml`](config/gemini_pick.yaml) with a
comment each. The ones that decide whether a run works:

| parameter | default | note |
|---|---|---|
| `mode` | `classify` | `classify` sorts what it finds, `target` picks what you name |
| `camera_optical_frame` | `d405_depth_optical_frame` | primary camera-pose source: looked up live in TF, published by robot_state_publisher from this robot's own URDF |
| `camera_xyz` / `camera_rpy` | this mount's URDF chain, resolved by hand | **fallback only**, used if `camera_optical_frame` is missing from TF (e.g. no robot_state_publisher running) |
| `table_z` | `0.0` | the calibrated world origin sits on the table surface |
| `target_support_enabled` | `false` | enable only when the target sits on a separately measured elevated support |
| `target_support_z` | `0.0` | top of that support; used for target segmentation and grasp clearance, without moving the main table plane |
| `target_support_collision_size_*` / `center_*` | `0.0` | explicit axis-aligned collision box; zero/unmeasured dimensions make preflight fail closed when support is enabled |
| `max_width` | `0.065` | candidate-width cap; wider GraspNet predictions are clipped to 65 mm before collision checking, but this does not calibrate jaw commands |
| `anygrasp_runtime_dir` | local `anygrasp_sdk/grasp_detection` | directory containing the licensed `gsnet.so`; checkpoint and license paths are separate parameters |
| `anygrasp_max_width` / `anygrasp_gripper_height` | `0.065 m` / `0.058 m` | physical dimensions used when constructing the AnyGrasp detector; they do not replace jaw calibration |
| `anygrasp_dense_grasp` / `anygrasp_collision_detection` | `false` / `true` | keep sparse NMS output and AnyGrasp full-scene collision rejection enabled for normal operation |
| `top_k` | `50` | maximum learned-backend candidates retained before OM6DOF safety filtering |
| `near_miss_markers_enabled` / `debug_grasp_markers_enabled` | `true` / `false` | show one rejected diagnostic by default; opt in to the cluttered all-candidate filter-tuning layer |
| `max_tilt` | `1.50` rad | how far from straight down a grasp may lean |
| `selection_score_slack` | `0.15` | used only when capture-time EE orientation is unavailable; normally every safe survivor is ranked orientation-first and score is a tie-break |
| `workspace_min` / `max` | same box as `direct_pick` | both the grasp *and* the pregrasp must be inside it |
| `ik_position_tolerance` / `ik_orientation_tolerance` | `0.003 m` / `0.05 rad` | FK residual gate used before a candidate may enter motion planning |
| `max_prevalidation_candidates` | `5` | maximum ranked candidates given a complete no-motion MoveIt chain check before exactly one physical attempt (valid range 1–20) |
| `gripper_open_pos` / `gripper_close_pos` | `0.019` / `-0.010` | configured joint endpoints; these are joint coordinates, not aperture widths |
| `gripper_width_at_open_pos` / `gripper_width_at_close_pos` | `-1.0` / `-1.0` | measured clear jaw apertures at those joint endpoints, in metres; negative means deliberately unmeasured |
| `gripper_close_bias` | `0.6` | 0 stops at the measured width, 1 closes fully |
| `dynamixel_health_topic` | `/dynamixel_hardware_interface/health` | reliable, transient-local hardware health emitted by the patched Dynamixel driver |
| `dynamixel_health_timeout_s` | `0.30` | physical motion stops/refuses when the health stream is older than this |
| `dynamixel_health_clean_window_s` | `60.0` | continuous fresh/healthy time with unchanged read/write counters required before execution |
| `execute_motion` | `false` | `false` sends MoveIt plan-only goals and never executes trajectories; gripper commands are skipped |
| `calibration_validated` | `false` | explicit operator acknowledgement required before `execute_motion:=true` |
| `gripper_calibration_validated` | `false` | separate acknowledgement that both aperture endpoints were physically measured and checked |
| `place_enabled` | `false` | leave false for the first physical pick/lift; the object remains held |
| `place_poses_validated` | `false` | required with physical execution when `place_enabled:=true` |

---

## Motion preflight and calibration gates

The acknowledgement parameters are interlocks, not automatic calibration.
Setting one to `true` means the operator has completed the corresponding gate
on this physical robot. Do not set all flags at once merely to get past a
refusal message.

### Gate 0 — ROS graph and operator safety

Clear the robot workspace, know how to stop the hardware, and keep a hand near
the stop control. Confirm that one MoveGroup server is present, joint states
are fresh, and both arm and gripper controllers report `active` before allowing
execution:

```bash
ros2 action info /move_action
ros2 control list_controllers
ros2 topic echo /joint_states --once
ros2 topic echo /dynamixel_hardware_interface/health --once
ros2 service call /gemini_pick/status std_srvs/srv/Trigger
```

The health diagnostic contains cumulative `read_failure_count` and
`write_failure_count` values. A one-cycle fault therefore remains visible even
if its error sample was dropped: any increment, driver restart/counter reset,
non-OK diagnostic, disabled actuator torque, hardware-error bit, missing
publisher, or stale sample restarts the clean window. Physical preflight stays
closed until the default 60-second window completes, and the 200 ms runtime
monitor uses the same freshness check to cancel an active action if the
publisher disappears. Plan-only operation does not require this gate.

The new topic exists only after the rebuilt hardware-interface library is
loaded by a newly started hardware service. Building the workspace does not
replace the library inside an already-running controller process.

The launch includes `om6dof_moveit_config` by default. If MoveIt was started
separately, launch this package with `start_moveit:=false`; never start a second
MoveGroup/controller stack over the same arm.

### Gate 1 — camera, TCP, and table geometry

With the arm stationary, verify the live TF while moving the wrist slowly by
hand/jog control. The camera frame must move rigidly with the wrist and its axes
must agree with the optical convention:

```bash
ros2 run tf2_ros tf2_echo world d405_depth_optical_frame
ros2 run tf2_ros tf2_echo end_effector_link d405_depth_optical_frame
```

Measure the physical pinch point and update `tcp_offset_xyz` if it is not the
URDF origin. Measure the table top in `world`, set `table_z`, and make the
planning-scene table box end at that same height. Only after these measurements
and TF checks agree may `calibration_validated:=true` be used. For an object on
an elevated pedestal, independently measure its top Z, XYZ dimensions and XY
centre; enable `target_support_enabled` only after all values are configured.

### Gate 2 — gripper aperture calibration

`gripper_open_pos` and `gripper_close_pos` are joint coordinates; neither says
how many millimetres fit between the real finger faces. Using the normal safe
commissioning procedure for this gripper, measure the clear inner-face gap at
each endpoint and enter those metre values as `gripper_width_at_open_pos` and
`gripper_width_at_close_pos`. The open width must be larger than the close
width, and the accepted GraspNet interval (`min_width` through `max_width`) must
lie inside that measured interval.

The node uses an affine interpolation between these two measurements before
applying `gripper_close_bias`. With either value left at `-1.0`, plan-only runs
continue using the old `[0, max_width]` preview and say so in the log, but
physical execution fails closed. Set `gripper_calibration_validated:=true` only
after checking the endpoint measurements and an intermediate-width command.
The two launch arguments default to `-1.0` deliberately. For a physical run,
put your own measured metre values in `OM6_GRIPPER_OPEN_WIDTH_M` and
`OM6_GRIPPER_CLOSE_WIDTH_M`, then pass them as shown below; no generic numeric
example is safe for a different gripper.

### Gate 3 — perception alignment, still no planning

Run `~/perceive` repeatedly with `execute_motion:=false`. In RViz, the thick red
gripper is the single selected candidate that passed every safety gate. When
nothing passes, `Best Near Miss (NOT EXECUTABLE)` instead shows exactly one
rejected candidate. It is chosen deterministically as the proposal that reached
the furthest safety gate, then by the smallest normalized violation. Its opaque
reason-coloured gripper, pregrasp arrow, literal rejection label, and—when the
failure is scene collision—the exact red collision points make the failed gate
inspectable without hiding the scene under dozens of glyphs. It is DEBUG ONLY:
publishing it does not weaken a gate and it can never become a motion target.

For deeper tuning, explicitly enable `debug_grasp_markers_enabled` and the RViz
`Non-selected Grasps (DEBUG ONLY)` display. That optional layer shows all
non-selected proposals as thinner opaque glyphs: green is
valid-but-unselected, magenta is scene collision, orange is reachability, cyan
is tilt, purple is width, blue is workspace/off-target, yellow is clearance,
and grey is another reason. It is disabled by default and is also never
consumed by motion. For AnyGrasp, "raw" here means the candidates returned
after the SDK's own collision filtering and NMS; the licensed API does not
expose its earlier pre-collision pool. Test a known-width object with a ruler;
a grossly wrong reported jaw width usually means the table plane or camera
extrinsic is wrong. Do not move on until at least one candidate passes
workspace, clearance, tilt, width, scene collision, and IK.

### Gate 4 — complete MoveIt plan-only path

Call `~/run` with `execute_motion:=false` and `place_enabled:=false`. This sends
actual planning requests without executing them: the joint moves use OMPL and
the chained Cartesian segments use Pilz `LIN`. The status must reach
`plan-only pick path succeeded`; inspect any failed stage before retrying.
Controllers and gripper commands remain untouched in this mode.

With `execute_motion:=true`, the node performs an additional no-motion proof
after perception and before candidate-specific motion. For at most
`max_prevalidation_candidates` ranked grasps it plans pregrasp, Pilz LIN to the
grasp, a simulated calibrated gripper close, and the post-grasp Pilz LIN path,
resetting the simulated state after every candidate. It prefers the configured
world-Z lift; if that LIN path is unavailable, a straight retreat back along
the unchanged approach axis to the exact pregrasp is allowed. Only the first
fully plannable candidate is attempted physically, and a physical failure is
never followed by a stale-candidate retry. This proves planning/collision
feasibility, not object contact, grip force, or retention.

```bash
ros2 launch om6dof_pick_and_place_gemini gemini_pick.launch.py \
    execute_motion:=false place_enabled:=false
ros2 service call /gemini_pick/run std_srvs/srv/Trigger
```

### Gate 5 — first physical pick, no place

Use one easy object near the workspace centre, keep the shipped low velocity and
acceleration scales, and execute only pick → close → lift. The object remains
held because place is disabled:

```bash
ros2 launch om6dof_pick_and_place_gemini gemini_pick.launch.py \
    execute_motion:=true calibration_validated:=true \
    gripper_width_at_open_pos:="$OM6_GRIPPER_OPEN_WIDTH_M" \
    gripper_width_at_close_pos:="$OM6_GRIPPER_CLOSE_WIDTH_M" \
    gripper_calibration_validated:=true place_enabled:=false
ros2 service call /gemini_pick/run std_srvs/srv/Trigger
```

### Gate 6 — measured place bins

Every pose in `config/places.yaml` is a physical joint target, not a semantic
label learned by Gemini. Measure and dry-run each bin independently, including
clearance on the approach and retreat. Only then enable the final two flags:

```bash
ros2 launch om6dof_pick_and_place_gemini gemini_pick.launch.py \
    execute_motion:=true calibration_validated:=true \
    gripper_width_at_open_pos:="$OM6_GRIPPER_OPEN_WIDTH_M" \
    gripper_width_at_close_pos:="$OM6_GRIPPER_CLOSE_WIDTH_M" \
    gripper_calibration_validated:=true \
    place_enabled:=true place_poses_validated:=true
```

---

## The camera extrinsic

`capture_scene()` places the camera by looking up `base_frame -> camera_optical_frame`
(default `d405_depth_optical_frame`) in TF at the **RGB-D capture timestamp**.
That frame is published by
`robot_state_publisher` from `om6dof_description`'s URDF, which encodes this
D405 mount's true extrinsic — SVD-fitted off the fused bracket+camera mesh and
cross-checked against the Intel D400 datasheet (see the long comment above
`d405_link_joint` in `om6dof.urdf.xacro`). That is measured for *this* robot's
mount, so it is the right source of truth, not a borrowed number.

The `camera_xyz` / `camera_rpy` parameters are a **fallback only**, used when
`camera_optical_frame` is not in the TF tree — `gemini_probe --realsense` has no
TF at all, for instance, since it never starts a node. Their default is the
same URDF chain resolved by hand once (`link7 -> d405_payload_link ->
d405_link -> d405_depth_optical_frame`, end_effector_link's offset backed out),
not a copy of `om6dof_pick_and_place/config/tag_pick.yaml`'s numbers — those
are explicitly commented in that package as "converted from
`google-deepmind/mujoco_menagerie`'s ALOHA model," a different mount on a
different robot. The two disagree by about 64 mm in Z and use a different
rotation convention entirely (pure pitch there vs. roll+yaw here); an earlier
version of this package copied the ALOHA numbers by mistake; a `~/perceive`
run on hardware showed the resulting grasp candidates off, which is what
surfaced this.

If TF lookup fails, the node logs a warning once (not every capture) and falls
back to the parameters for the rest of the run. To re-derive the fallback
numbers by hand after any change to the D405 mount:

```bash
ros2 run tf2_ros tf2_echo end_effector_link d405_depth_optical_frame
```

## Tuning it on real objects

`~/perceive` is the loop to live in — it never moves the arm.

* **"no usable grasp in this frame"** — check the status line's point count
  first. Near zero means the cloud is empty: `cloud_z_min`/`cloud_z_max` or the
  camera. A healthy count with nothing accepted means the filter ate everything;
  the rejection summary (`widthx8, reachabilityx4`) names which gate.
* **everything rejected on `width`** — the configured target support surface
  (`target_support_z`, or `table_z` when no support is enabled) is probably
  wrong, so clusters merge with the support and measure far too wide. Put a
  known object down and compare the reported width against a ruler.
* **everything rejected on `reachability`** — add larger entries to
  `approach_tilts`, or move the object closer; the straight-down grasp is only
  reachable at low `z` on this arm.
* **grasps land beside the object** — the camera extrinsic. Re-run the calib GUI
  and copy `camera_xyz` / `camera_rpy` over.
* **Gemini points at the right object but nothing matches** — raise
  `target_match_radius_px`, or check that the object is actually clustering (the
  markers show what was found).

---

## Tests

Pure logic, no camera, no arm, no network:

```bash
cd ~/ros2_ws/src/om6dof/om6dof_pick_and_place_gemini
python3 -m pytest test -q

# Or through the ROS package test path from the workspace root:
cd ~/ros2_ws
colcon test --packages-select om6dof_pick_and_place_gemini
colcon test-result --verbose
```

Covered: the tool-frame convention against the existing pitch=π convention,
the wrist-camera chain, deprojection, clustering and grasp geometry on synthetic
scenes, every filter gate and its rejection reason, Gemini key resolution and
reply parsing (including fenced JSON and off-list labels), and the width →
jaw-command map.

---

## Status

Written and unit-tested. The MoveIt smoke has completed in true plan-only mode,
including OMPL joint planning and the chained Pilz `LIN` segments; no trajectory
was executed in that check. Physical autonomous pick/place remains gated by the
camera/TCP/table sign-off and measured place poses above.

The shipped place values are placeholders copied from `direct_pick`'s single
place pose, not measured bins. Keep `place_enabled:=false` until every entry has
been validated on this arm. The safe progression is `~/perceive` → full
plan-only `~/run` → physical pick/lift without place → measured full loop.
