# om6dof_dd_gng

DD-GNG experiments and RealSense processing for the OM6DOF stack.

The project now lives under `om6dof/om6dof_dd_gng`; it is a standalone CMake
project rather than a ROS 2 package. It contains:

- `DepthSensor_Buggy/`: the original ODE/OpenGL simulation.
- `realsense_ddgng/`: the RealSense input and OpenCV overlay using the shared
  DD-GNG core.

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
