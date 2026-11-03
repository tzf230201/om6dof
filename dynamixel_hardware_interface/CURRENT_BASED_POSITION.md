# Normal stack: current-based position control

The normal position-only description (`current_control=false`) defaults joint1–6
to Mode 5. Joint1/ID31, joint4/ID24, and joint6/ID26 override Position P Gain=150
and Position D Gain=1400 only in Mode 5. Their I gains and joint2/3/5 P/I/D gains
are not overridden.
The existing motion profiles are retained. Explicit Mode 3 has no PID overrides.
It initializes arm Goal Current from each stored Current Limit. The separate
effort description and its arm_operating_mode argument remain unchanged.
Existing running motors retain their old RAM gains
until hardware is restarted/reinitialized; merely editing xacro is not a live
gain update. Verify mode-specific gains after initialization before motion.
The driver resolves `Goal Current=current_limit` during torque-off initialization. It reads the motor's
stored Current Limit, copies it to Goal Current, and verifies the readback. It
never writes Current Limit. Failed transactions, wrong mode, zero or invalid
limits abort initialization. An explicit Current Limit override is rejected.

This is a current budget, not a demand for continuous maximum current, and is not
the gravity-relative extra-torque budget of impedance control. Mode 5 does not
enforce Mode 3's servo Min/Max Position Limit; retain software URDF limits and
bounded targets. Existing limits do not prove supply/mechanical load capacity.

Gripper remains Mode 5, Goal Current 45. Supervised impedance READY preparation
explicitly uses Mode 3; torque teaching/impedance remains Mode 0 with zero initial
current. Torque-off diagnostics retain arm Mode 3.

Descriptions using `current_limit`, including the normal Mode 5 description,
require the updated driver. Stop active hardware under physical support
before replacing its library. This source change does not restart hardware.

Reference: https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/
