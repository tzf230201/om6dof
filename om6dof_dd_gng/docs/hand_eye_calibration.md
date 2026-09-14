# D435i eye-in-hand calibration for DD-GNG execution

## Provisional ruler correction

While the measured hand-eye artifact remains unverified, the V2 environment
profile applies `nominal_world_z_offset_m: -0.0185`. Two stationary ruler
checks at approximately 0.224 m and 0.410 m camera range showed the nominal
URDF world Z high by about 18.5 mm. The first corrected point was 0.1298 m for
a 0.1300 m reference; the second corrected point was approximately 0.1142 m
at the ruler crosshair.

The correction translates both native-depth and raw-RGB camera poses along
the world Z axis before environment graph construction and semantic
reprojection. It remains provisional: it does not change the URDF, does not
set `calibration_verified`, and does not unlock graph-pick execution. A valid
serial-bound `om6dof.hand_eye.v1` artifact supersedes the correction.

The V2 wrist-camera model starts from CAD geometry and is suitable for mapping
and path preview. Real motion stays locked until a measured calibration binds
the exact D435i serial to `end_effector_link` and passes validation.

The supported workflow uses a stationary AprilTag and at least eight distinct
robot poses. The collector does not command the robot. The operator moves the
wrist with the already commissioned controls, holds it still, and captures one
observation at each pose. Spread the poses in translation and rotation; moving
along one line does not constrain the full hand-eye transform.

Only one process may own the RealSense. Stop the DD-GNG planning launch with
Ctrl-C before starting calibration. Keep the printed AprilTag fixed for the
complete capture and measure its outer black-edge size.

Start the V2 read-only state publisher and AprilTag camera owner:

```bash
source /opt/ros/humble/setup.bash
source /home/kublab/ros2_ws/install/setup.bash
ros2 launch om6dof_pick_and_place ddgng_hand_eye_calibration.launch.py \
  camera_serial:=CAMERA_SERIAL tag_id:=1 tag_size:=0.04
```

In another terminal, start the interactive collector with the same serial:

```bash
source /opt/ros/humble/setup.bash
source /home/kublab/ros2_ws/install/setup.bash
ros2 run om6dof_pick_and_place ddgng_hand_eye_calibrate --ros-args \
  -p camera_serial:=CAMERA_SERIAL \
  -r /tf:=/om6dof_dd_gng/v2/tf \
  -r /tf_static:=/om6dof_dd_gng/v2/tf_static
```

At each stable pose press Enter. The collector rejects nearly identical poses.
After at least eight accepted poses type `selesai`. It writes
`/home/kublab/.config/om6dof/d435_hand_eye.yaml` only when translation RMSE is
at most 0.015 m and rotation RMSE is at most 0.08 rad.

Stop the calibration launch, then load the measured artifact:

```bash
ros2 launch om6dof_dd_gng ddgng_rviz_target_planning.launch.py \
  execution_enabled:=true \
  camera_calibration_file:=/home/kublab/.config/om6dof/d435_hand_eye.yaml
```

The launch validates the schema, exact camera serial, frames, sample count,
residuals, normalized quaternion, and SHA-256. The C++ camera node repeats the
runtime checks after opening the camera. It uses the measured
`end_effector_link <- d435_color_optical_frame` transform for raw-RGB YOLO and
combines it with the connected device's factory depth-to-colour extrinsic for
the native-depth DD-GNG projection. A missing, malformed, weak, or wrong-serial
artifact leaves `calibration_verified=false`; it never falls back to claiming
the nominal URDF transform as verified.
