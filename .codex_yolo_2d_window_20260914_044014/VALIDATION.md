# Existing YOLO viewer added to experiment launch

User requested reusing the existing analysis-launch viewer.
Final change adds the same yolox_viewer_node (RGB compressed topic, display_window=true)
with launch_yolo_2d default true. No new viewer implementation retained.
Package dependencies restored to pre-change contents; CMake viewer additions removed.
Six existing launch tests passed; package build succeeded.
Existing C++ viewer started on the active RViz desktop :1002, PID150158.
TensorRT engine loaded; X11 confirms YOLOX C++ — RealSense RGB window (960x720).
Temporary rqt window opened during inspection was stopped (only its own PID).
No controller restart, torque, robot motion or second camera owner.
The viewer uses its own RGB inference, exactly as yolox_ddgng_analysis.launch.py.
Current viewer was started separately; close it before restarting the combined launch to avoid duplicate viewers.
