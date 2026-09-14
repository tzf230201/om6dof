# Combined-reference prototype validation — 2026-09-13

Request: automatic nominal trajectory continues; Logitech input accumulates a
persistent offset. Neutral holds offset, without AUTO/MANUAL switching.

Implemented an isolated composition preview and Logitech Twist adapter. The
requested physical combined controller/default remains unimplemented: no motor
command path was added or activated. See package config/COMBINED_REFERENCE.md
for remaining whole-body/environment, dynamics, action and hardware checks.

Evidence:
- 147 tests passed across controller and teleop, including 35 new tests.
- Both packages built successfully with symlink install.
- New launch --show-args succeeded; RViz YAML parsed.
- Installed preview executable started on ROS domain 231, localhost only,
  for five seconds. Timeout sent SIGINT (exit 124 as expected).
- No camera, robot controller, torque operation or motion goal launched.
- Reachability source/header hashes remain the protected baseline hashes.
- git diff --check passed (including existing dirty worktree).

Tests cover accumulated translation/rotation, concurrent nominal progression,
neutral retention, fresh packet intervals, expiry, duplicate/out-of-order/
invalid inputs, bounds/no windup, new references, reconnect neutral, IK/joint
limit rejection and invalid/stale feedback suppressing preview joint output.

No live camera, joystick, physical motion or obstacle clearance validation.
Green and magenta lines are requested references, not certified safe paths.
Original touched files are backed up here; before.json/after.json record hashes.
