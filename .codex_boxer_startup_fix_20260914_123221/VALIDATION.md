# BOXER startup diagnosis and fix

Original live launches 409410 and409556 exited1, leaving RViz empty.
Reproduced exact startup exception: Couldn't resolve requests.
Device243222076197 exposed only Stereo Module and RGB Camera, no accel profiles.
Removed unconditional accel63Hz request; auto chooses TF when no supported IMU.
Read-only V2 robot_state_publisher on private /boxer/tf uses real /joint_states.
Camera up comes from world->d435_color_optical_frame at RGB host-receipt time,
with a bounded wait for TF, never a stale/latest fallback. No robot commands.

Additional reproduced defects:
- RGB inverse Brown Conrady with coefficients [0,0,0,0,0] wrongly rejected. Now accepted as exact pinhole; nonzero inverse Brown still rejected.
- OpenCV GTK WND_PROP_VISIBLE returns -1 (unsupported). Old code treated <1 as closed; now only explicit zero means closed.
- At initial camera warmup one frame pair exceeded50ms; rejected until synced. Stable measurements RGB/depth global_time difference~0.058ms. Threshold not changed.

Observed point cloud now publishes before manual prompting, at approximately1Hz.
Launch closes its own RViz on preview exit; stderr/stdout captured with output=both and unbuffered Python.

Validation: package build successful,6 pytest cases passed, Python compile and diff-check passed.
Live corrected launch413140, preview RGB window960x720 confirmed on user's DISPLAY:1002.
Read-only subscriber received /boxer/points: width99222,height1,data1190664bytes,frame boxer_local.
Existing BOXER RViz409558 reused; no second camera owner. Corrected preview remains running for user.
No controller restart, torque, synthetic joints or robot/gripper command performed.
This verifies live capture/TF/depth visualization; real-object BOXER inference accuracy remains unvalidated.
