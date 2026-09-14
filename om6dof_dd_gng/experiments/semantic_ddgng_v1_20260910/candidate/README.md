# om6dof_dd_gng

DD-GNG experiments and RealSense processing for the OM6DOF stack.

## Environment algorithm correction (2026-09-10)

The C++ world-mapping node now uses **semantic-attention DD-GNG + YOLOX**
(`semantic_dd_gng_v1`). The independent reachability node still uses
**ordinary GNG** with its existing configuration-space validation.
This correction changes the environment learner; it does not command robot motion.
See [the method and validation notes](docs/semantic_ddgng_v1.md).

Earlier C++ captures labelled "DD-GNG" used the vendored
`DynamicGrowingNeuralGas` learner without semantic density feedback.
Do not relabel those captures or frozen reachability experiments as validation
of this new implementation. The separate Python/web legacy path is unchanged.

![DD-GNG topological graph of the workspace, with the bench it was built from](../docs/assets/ddgng-graph.jpg)

The screenshot shows the **C++ world-frame mapping** path: its graph stays in
the world while the wrist camera moves. The Python/web segmentation path is
different: its graph remains camera-relative and does not apply robot FK.

The project lives under `om6dof/om6dof_dd_gng`. It is a proper ament_cmake
ROS 2 package. Its normal `colcon build` installs both the C++ ROS nodes and
the Python viewers with their native `libddgng.so`; no separate source-tree
library build is needed for the launch commands below. It contains:

- `DepthSensor_Buggy/`: the original ODE/OpenGL simulation (plain CMake).
- `realsense_ddgng/`: versioned RealSense input and Python/OpenCV overlays
  using the original DD-GNG core; this is the web monitor's "3D segmentation"
  path.
- `src/topo_gng_node.cpp`, `include/om6dof_dd_gng/`: `topo_gng_node`, the
  rclcpp TopoVLA DD-GNG + YOLO integration -- see "topo_gng_node" below.

## V1 and V2

**V1 remains the default.** V2 is selected explicitly and uses the updated
`om6dof_v2` model. V2 camera `auto` selects **D435i**, matching the installed
camera; no serial is needed when exactly one matching device is connected.
For a plain D435, pass `camera_model:=D435`. Multiple matching cameras require
`camera_serial:=<serial>`; a different model is never selected silently.

Build and source the updated packages:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select \
  om6dof_description om6dof_perception om6dof_dd_gng
source install/setup.bash
```

Choose the path appropriate for the task; these are not interchangeable:

| Path | V2 launch | Coordinates / robot model |
|---|---|---|
| Python YOLO segmentation, OpenCV and web image | `dd_gng_yolo_v2.launch.py` | Colour-camera XYZ in **metres**; no arm FK or world projection |
| C++ environment/body graph and reachability preview | `topo_gng_v2.launch.py` | World XYZ using **V2 URDF kinematics** and measured `/joint_states` |
| C++ reachability preview alone | `reachability_v2.launch.py` | V2 FK/collision model; does not open a camera |

For the desktop segmentation window and the web topics:

```bash
ros2 launch om6dof_dd_gng dd_gng_yolo_v2.launch.py
```

Use `headless:=true` to omit the OpenCV window, or `camera_model:=D435` when
the connected camera is a D435 rather than D435i. The default launch
`dd_gng_yolo.launch.py` retains V1. The installed Python scripts also accept
`--model-version v2 --camera-model auto` and optional `--camera-serial`.

For world mapping, or a camera-free roadmap preview:

```bash
ros2 launch om6dof_dd_gng topo_gng_v2.launch.py
# Alternative: no camera capture, only the reachability preview.
ros2 launch om6dof_dd_gng reachability_v2.launch.py
```

The two C++ V2 wrappers publish a read-only V2 robot state from actual
`/joint_states`. They isolate their model on `/om6dof_dd_gng/v2/tf`,
`/om6dof_dd_gng/v2/tf_static`, and `/om6dof_dd_gng/v2/robot_description`,
with graph topics/services under `/om6dof_topo_gng_v2/`. This prevents an
existing V1 TF tree from supplying the wrong geometry. Without measured
joint states, live world registration/current-state planning cannot be
validated. These launches do **not** start a hardware interface or controller;
the reachability result is a preview, not a motion command.

The V1 baseline inputs remain unchanged. After changing URDF or switching
model versions, rebuild as needed and **restart the launch**: the
`rebuild_reachability` service rebuilds its graph, not the loaded robot model.
V2 mounting extrinsics are CAD/nominal, not measured hand-eye calibration.
Capture timestamps are host receipt times, not hardware-synchronized camera
and encoder measurements. Hardware accuracy has not been validated by these
software changes.

Only **one camera capture process** may run at a time: stop an existing
perception/DD-GNG camera workload before opening another. The standalone
reachability preview does not own a camera. Python outputs explicitly report
`coordinate_space: camera_optical`, `units: metres`, and
`robot_kinematics_applied: false`; do not treat them as world coordinates.

The default ROS workspace build installs the headless AGX systemd unit and
does not build the legacy OpenGL/ODE visualizer. To build that optional
visualizer, install `libode-dev`, `freeglut3-dev`, and `libx11-dev`, then pass
`--cmake-args -DBUILD_DDGNG_VISUALIZER=ON` to `colcon build`.

## Web monitor

The user service runs DD-GNG headlessly and publishes its annotated stream on
`/application_web_monitor/ddgng/image/compressed`:

```bash
mkdir -p ~/.config/systemd/user
install -m 0644 \
  ~/ros2_ws/install/om6dof_dd_gng/share/om6dof_dd_gng/systemd/om6dof-dd-gng.service \
  ~/.config/systemd/user/om6dof-dd-gng.service
systemctl --user daemon-reload
```

Use **Start 3D segmentation** / **Stop 3D segmentation** in the Kublab web monitor.
The web service launches the installed Python DD-GNG viewer headlessly; its
3D semantic metadata is published
on `/application_web_monitor/ddgng/labels`. DD-GNG and
OM6DOF perception both own the RealSense directly, so the systemd service
declares them as conflicting workloads; starting either one stops the other.

The supplied unit defaults to `OM6DOF_MODEL_VERSION=v1` and
`OM6DOF_CAMERA_MODEL=auto`. To select V2 for the web monitor, manually create
a user-service drop-in:

```bash
systemctl --user edit om6dof-dd-gng.service
```

Add:

```ini
[Service]
Environment=OM6DOF_MODEL_VERSION=v2
Environment=OM6DOF_CAMERA_MODEL=auto
```

Then run `systemctl --user daemon-reload` and stop/start **3D segmentation**
in the web monitor. No serial is required for the single D435i. Set
`OM6DOF_CAMERA_MODEL=D435` only for a plain D435. Building the repository
does not install this user-service file, change a drop-in, or restart a
running camera service automatically.

## DD-GNG + YOLO 3D box segmentation

`realsense_ddgng/dd_gng_yolo.py` combines the DD-GNG graph with YOLOX and
registered RealSense depth. V1 retains aligned-depth preprocessing; V2 uses
native SDK depth vertices transformed into colour coordinates with the actual
device's depth-to-colour extrinsics. YOLOX provides the object name and confidence. The
depth values inside each detection segment the near foreground, then produce a
robust camera-frame axis-aligned 3D bounding box. A GNG node receives that
COCO label only when its XYZ position is inside the segmented 3D box.

The overlay draws the projected 3D box and reports its width, height, depth,
and centre distance. The JSON labels topic now also includes a `boxes` array
with each named box's `center_m`, `size_m`, and foreground `point_count`.

```bash
ros2 launch om6dof_dd_gng dd_gng_yolo_v2.launch.py headless:=true
```

The labels topic is JSON containing each matched node's index, YOLO class and
confidence, camera-frame XYZ coordinate, projected UV pixel, and camera/model
metadata. Its V2 frame is `d435_color_optical_frame` (also used for D435i's
RGB/depth geometry); V1 retains `camera_color_optical_frame`. Use
`--classes bottle,cup` to restrict labelling or `--hide-node-labels` to retain
semantic colours without drawing text beside every matched node.

## topo_gng_node

The ROS 2 perception node for the TopoVLA <-> om6dof_dd_gng integration. It
publishes the environment and current robot body for RViz, plus a typed
environment graph consumed by `reachability_graph_node`.

- `~/environment_graph` (`world` frame, grey/coloured spheres+lines): a
  Dynamic Growing Neural Gas graph (`include/om6dof_dd_gng/ddgng.hpp`,
  vendored unmodified from `TopoVLA @ 0da5050,
  native_depth_yolo/src/ddgng.hpp`) learned from the selected camera's depth, deprojected with
  the camera's own live intrinsics and transformed into `world` via tf2 at
  each frame's own timestamp -- so the graph stays put in the world as the
  wrist (and camera with it) moves, rather than following the camera.
  YOLOX (OpenCV DNN; ONNX Runtime, what TopoVLA's own code uses, is not
  available on this Jetson) runs asynchronously against the aligned colour
  frame and labels nodes whose depth, image position, and re-projected
  visibility agree with a detection box strongly enough (see
  `labelGraph()`/`enrichDepth()` in `src/topo_gng_node.cpp` for the exact
  scoring, ported from TopoVLA's `main.cpp`); labelled nodes get a
  colour-per-class instead of grey, and also go out as JSON on `~/labels`
  (index, stable `node_id`, class, confidence, world XYZ).
- `~/robot_graph` (`world` frame, blue): a constant-topology graph (fixed
  nodes/edges, not GNG) at link1..link7, end_effector_link, both gripper
  fingers, and d405_payload_link, read from tf2 every frame -- so gripper
  opening/closing and arm motion move it automatically. The same segment
  geometry (with a per-link radius, `body_radius.*` parameters, and a
  `body_mask_margin` on top) is used as a self-body mask: depth points that
  land inside the robot's own capsule graph are dropped before they can seed
  a GNG node, which is what stops the wrist camera's view of its own gripper
  fingers from becoming phantom obstacle nodes.
- `~/environment_graph_data`: the planning representation of the DD-GNG
  graph. It preserves every stable `node_id`, world XYZ, semantic class and
  confidence, plus edges expressed in stable IDs. Consumers must use this
  typed topic rather than Marker colours or the compatibility JSON topic.

```bash
systemctl --user stop om6dof-dd-gng.service om6dof-perception.service
ros2 launch om6dof_dd_gng topo_gng_node.launch.py
```

The default V1 launch needs `robot_state_publisher` (and therefore joint states) already running
for the `world -> d405_depth_optical_frame` TF chain to resolve; see
`om6dof_description/urdf/om6dof.urdf.xacro` for how that frame's pose was
derived from the D405 datasheet and the wrist-camera mesh. All parameters
(pixel step, node/update caps, YOLO model/thresholds, `target_classes`,
per-link capsule radii, etc.) are in `config/topo_gng.yaml`.
V2 uses `config/topo_gng_v2.yaml` and the isolated state publisher described
above; it must not reuse the old D405 optical extrinsic.

## End-effector reachability graph

`reachability_graph_node` is independent of the RealSense capture process and
never publishes to a hardware controller. It builds a deterministic roadmap
of the `arm` MoveIt group (`link1 -> end_effector_link`) as follows:

1. Select `graph_method:=gng` to quantize valid normalized joint samples with
   deterministic Growing Neural Gas, or `graph_method:=halton_prm` for the
   controlled Halton/PRM baseline.
2. Compute the end-effector pose with MoveIt `RobotState` FK.
3. Reject invalid/self-colliding configurations. With
   `strict_self_collision:=true`, only directly adjacent robot links are
   exempted; this intentionally does not inherit the very broad SRDF
   "Never" collision exemptions.
4. Connect nearest neighbours in normalized joint space, preserving multiple
   IK branches that may occupy the same XYZ.
5. Validate every edge by interpolating its joint values and collision-checking
   each sample.

The resulting typed graph is published on
`/om6dof_topo_gng/reachability_graph_data`; every node contains its stable ID,
full EEF pose, and six joint positions. Its RViz view is
`/om6dof_topo_gng/reachability_graph`.

At runtime the node reads `/joint_states` and the typed environment graph.
Semantically labelled environment nodes are targets; unlabelled nodes/edges
are obstacles. Every roadmap node and interpolated edge has a cached capsule
sweep for link2 through link7, the end effector, both fingers, and the D405.
An environment update only recomputes graph invalidation against these cached
sweeps; it does not repeat FK or rebuild the roadmap. The fixed-base
link1-link2 capsule is excluded by default to avoid treating the known support
surface as a collision. The node attaches the measured configuration to the
nearest valid roadmap node, selects a reachable node within
`target_intersection_radius`, and runs Dijkstra over currently unblocked
roadmap edges. The result is published on:

- `/om6dof_topo_gng/reachability_plan` (typed status, IDs, joint preview and
  EEF path), and
- `/om6dof_topo_gng/reachability_path` (`nav_msgs/Path` for RViz).

`ReachabilityGraph` reports the selected method and component count;
`ReachabilityPlan` reports blocked-node/edge counts and planning time for
repeatable PRM-vs-GNG experiments. Collision validation is two-stage:

1. cached swept capsules invalidate the complete roadmap quickly; and
2. the shortlisted route is interpolated again and checked by MoveIt/FCL
   against exact robot meshes. DD-GNG nodes are PlanningScene spheres and
   graph edges are cylinders. If FCL rejects a roadmap edge, it is disabled
   and Dijkstra runs again, up to `exact_max_replans`.

`ReachabilityPlan.exact_collision_valid` must be true before a future executor
may consider the preview. It also reports exact state-check, replan, and timing
counts. The joint trajectory remains deliberately named `joint_path_preview`:
it is not connected to `FollowJointTrajectory` and is not an execution
authorization.

Run only the roadmap (no camera, MoveIt server, controller, or motor):

```bash
ros2 launch om6dof_dd_gng reachability_graph.launch.py launch_rviz:=false
```

Run the matched baseline without editing YAML:

```bash
ros2 launch om6dof_dd_gng reachability_graph.launch.py \
  launch_rviz:=false graph_method:=halton_prm
```

Run perception and reachability together:

```bash
ros2 launch om6dof_dd_gng topo_gng_node.launch.py launch_reachability:=true
```

Rebuild the deterministic roadmap with its currently loaded model/parameters:

```bash
ros2 service call /om6dof_topo_gng/rebuild_reachability std_srvs/srv/Trigger '{}'
```

For V2 the service prefix is `/om6dof_topo_gng_v2/`. URDF changes require a
launch restart; this service is not a robot-description reload.

Run a repeatable, controller-free CSV benchmark. Every method/seed gets its
own ROS domain, identical node budget, synthetic start/target, and a dynamic
obstacle placed on the original route:

```bash
ros2 run om6dof_dd_gng reachability_benchmark.py \
  --methods gng,halton_prm --seeds 50 --sample-count 800 \
  --output ~/reachability_benchmark.csv
```

The CSV includes build time, graph size/components, clear and dynamic planning
time, capsule invalidation counts, exact FCL checks/replans/time, path length,
success, and failure reason. The benchmark never starts a camera, MoveGroup,
controller manager, or hardware interface.

## Low-light RealSense depth

DD-GNG reads the shared low-light configuration when it starts:

```text
~/.config/om6dof-realsense/low_light.json
```

Enable **RealSense low-light mode** in the web monitor, then start DD-GNG. It
uses the camera's IR emitter and configured laser power when the device exposes
those controls. D405 lacks that emitter control; D435/D435i expose it.
Low-light FPS is chosen from the rates advertised by both selected streams,
not a fixed D405-specific 5 FPS. Normal V2 operation retains the camera's
emitter setting. YOLOX object names still use the RGB camera, so a small
white LED is required if names must remain available in a dark scene.
