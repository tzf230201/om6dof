# Empty combined RViz diagnosis/fix

Read-only live status confirmed `waiting_for_new_trajectory_preview`, phase null.
Planner PID 418713 and camera node PID 418707 were active. Combined preview
subscribes volatile and had no new preview; old RViz config displayed only
Grid/combined MarkerArray, with no robot/environment and no waiting marker.

Changes: combined launch opens preview-only target GUI, RViz displays V2 robot,
environment, object clusters and reachability with isolated TF remaps. Marker
text now explains missing input even without any trajectory. GUI has no execute
client in preview-only mode; direct execute calls return without action.

37 relevant tests passed. Three packages built. Launch --show-args and
`git diff --check` passed. Old preview PID 468887 and its two verified children
were stopped, then the updated preview/GUI/RViz launched successfully on AGX
DISPLAY=:0. Hardware/controller/camera/planning processes were not restarted.
No motion goals sent; gamepad disabled for this launch.

Preview remains non-executable; no claim of real combined robot control.
