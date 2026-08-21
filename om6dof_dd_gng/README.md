# om6dof_dd_gng

DD-GNG experiments and RealSense processing for the OM6DOF stack.

The project lives under `om6dof/om6dof_dd_gng`. It is a proper ament_cmake
ROS 2 package (`package.xml` + `find_package(ament_cmake ...)` +
`ament_package()` in `CMakeLists.txt`) that also still carries its original
plain-CMake sub-build for the pieces that predate that; both build in the
same `colcon build` pass. It contains:

- `DepthSensor_Buggy/`: the original ODE/OpenGL simulation (plain CMake).
- `realsense_ddgng/`: the RealSense input and OpenCV overlay using the shared
  DD-GNG core (plain CMake + a separate Python/systemd deployment -- see
  below); still the thing the web monitor's "3D segmentation" button drives.
- `src/topo_gng_node.cpp`, `include/om6dof_dd_gng/`: `topo_gng_node`, the
  rclcpp TopoVLA DD-GNG + YOLO integration -- see "topo_gng_node" below.

Build the RealSense core from a clean build directory:

```bash
cmake -S realsense_ddgng -B realsense_ddgng/build_om6dof
cmake --build realsense_ddgng/build_om6dof -j
```

See `realsense_ddgng/README.md` for runtime dependencies and usage.

The default ROS workspace build installs the headless AGX systemd unit and
does not build the legacy OpenGL/ODE visualizer. To build that optional
visualizer, install `libode-dev`, `freeglut3-dev`, and `libx11-dev`, then pass
`--cmake-args -DBUILD_DDGNG_VISUALIZER=ON` to `colcon build`.

## Web monitor

The user service runs DD-GNG headlessly and publishes its annotated stream on
`/application_web_monitor/ddgng/image/compressed`:

```bash
install -m 0644 systemd/om6dof-dd-gng.service \
  ~/.config/systemd/user/om6dof-dd-gng.service
systemctl --user daemon-reload
```

Use **Start 3D segmentation** / **Stop 3D segmentation** in the Kublab web monitor.
The web service runs `dd_gng_yolo.py`; its 3D semantic metadata is published
on `/application_web_monitor/ddgng/labels`. DD-GNG and
OM6DOF perception both own the RealSense directly, so the systemd service
declares them as conflicting workloads; starting either one stops the other.

## DD-GNG + YOLO 3D box segmentation

`realsense_ddgng/dd_gng_yolo.py` combines the DD-GNG graph with YOLOX and
aligned RealSense depth. YOLOX provides the object name and confidence. The
depth values inside each detection segment the near foreground, then produce a
robust camera-frame axis-aligned 3D bounding box. A GNG node receives that
COCO label only when its XYZ position is inside the segmented 3D box.

The overlay draws the projected 3D box and reports its width, height, depth,
and centre distance. The JSON labels topic now also includes a `boxes` array
with each named box's `center_m`, `size_m`, and foreground `point_count`.

```bash
systemctl --user stop om6dof-dd-gng.service om6dof-perception.service
python3 realsense_ddgng/dd_gng_yolo.py \
  --headless \
  --ros-topic /om6dof_dd_gng_yolo/image/compressed \
  --labels-topic /om6dof_dd_gng_yolo/labels
```

The labels topic is JSON containing each matched node's index, YOLO class and
confidence, camera-frame XYZ coordinate, and projected UV pixel. Use
`--classes bottle,cup` to restrict labelling or `--hide-node-labels` to retain
semantic colours without drawing text beside every matched node.

## topo_gng_node

The ROS 2 node for the TopoVLA <-> om6dof_dd_gng integration: two graphs
published as `visualization_msgs/MarkerArray` for RViz, meant as the
perception front-end for a later robot-topology-vs-environment-topology
avoidance scheme (not implemented yet -- this node is perception and
visualization only).

- `~/environment_graph` (`world` frame, grey/coloured spheres+lines): a
  Dynamic Growing Neural Gas graph (`include/om6dof_dd_gng/ddgng.hpp`,
  vendored unmodified from `TopoVLA @ 0da5050,
  native_depth_yolo/src/ddgng.hpp`) learned from D405 depth, deprojected with
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

```bash
systemctl --user stop om6dof-dd-gng.service om6dof-perception.service
ros2 launch om6dof_dd_gng topo_gng_node.launch.py
```

Needs `robot_state_publisher` (and therefore joint states) already running
for the `world -> d405_depth_optical_frame` TF chain to resolve; see
`om6dof_description/urdf/om6dof.urdf.xacro` for how that frame's pose was
derived from the D405 datasheet and the wrist-camera mesh. All parameters
(pixel step, node/update caps, YOLO model/thresholds, `target_classes`,
per-link capsule radii, etc.) are in `config/topo_gng.yaml`.

## Low-light RealSense depth

DD-GNG reads the shared low-light configuration when it starts:

```text
~/.config/om6dof-realsense/low_light.json
```

Enable **RealSense low-light mode** in the web monitor, then start DD-GNG. It
uses the camera's IR emitter and configured laser power when the device exposes
those controls. The installed D405 does not expose those controls, so the mode
uses auto exposure. YOLOX object names still use the RGB camera, so a small
white LED is required if names must remain available in a dark scene.
