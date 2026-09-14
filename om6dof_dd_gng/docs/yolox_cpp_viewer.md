# YOLOX C++ RGB viewer

`yolox_viewer_node` is an isolated C++ TensorRT YOLOX visualizer. It never
opens a RealSense device. It subscribes to the RGB topic produced by the active
V2 DD-GNG node, keeps only the newest JPEG while inference is busy, and runs
TensorRT on that RGB image.

The node publishes every post-NMS COCO detection, rather than a selected target
class:

- RGB image with all boxes and class/score labels:
  `/om6dof_yolox/debug_image/compressed`
- JSON list of every box (class, score, x, y, width, height and inference time):
  `/om6dof_yolox/detections`

Run V2 first so that it is the only owner of the D435i camera:

```bash
source /opt/ros/humble/setup.bash
source /home/kublab/ros2_ws/install/setup.bash
ros2 launch om6dof_dd_gng topo_gng_v2.launch.py launch_reachability:=false launch_rviz:=false
```

Then, in another terminal, launch the C++ viewer:

```bash
source /opt/ros/humble/setup.bash
source /home/kublab/ros2_ws/install/setup.bash
ros2 launch om6dof_dd_gng yolox_viewer.launch.py
```

To show the overlay in a local window on the AGX desktop, launch it from a
terminal opened on that desktop:

```bash
ros2 launch om6dof_dd_gng yolox_viewer.launch.py display_window:=true
```

The local desktop session must have a `DISPLAY` environment variable. A normal
SSH terminal has no desktop display and therefore cannot create this window.

## DD-GNG + YOLO 2D + RViz analysis

Run the following once from an AGX desktop terminal. It starts the V2 node as
the sole D435i owner, the local C++ YOLO 2D window, and RViz with the per-object
DD-GNG cluster markers:

```bash
source /opt/ros/humble/setup.bash
source /home/kublab/ros2_ws/install/setup.bash
ros2 launch om6dof_dd_gng yolox_ddgng_analysis.launch.py
```

Do not leave an older `yolox_viewer_node` or `topo_gng_v2.launch.py` running;
the combined launch already starts both required perception nodes.

The analysis launch also displays the existing reachability GNG roadmap in
RViz. It is visualization and preview only; it does not execute a robot path.

## RViz target-planning analysis

For RViz plus a desktop object selector, with no separate YOLO 2D window:

```bash
ros2 launch om6dof_dd_gng ddgng_rviz_target_planning.launch.py
```

Choose a class in the GUI and press **Set planning target**. This publishes the
semantic target, then press **Preview EoE path in RViz**. This starts only the
preview coordinator and never executes motion.

To expose the separate **Execute planned motion** button, start the launch with
`execution_enabled:=true`. It still requires an explicit click and every live
execution guard to pass; a rejected guard sends no robot command.

The Web Monitor's **RGB + YOLO bounding boxes** tab reads the C++ overlay by
default. The node is RGB-only; it does not change DD-GNG semantics, TF,
reachability, controller, torque, or robot movement.
