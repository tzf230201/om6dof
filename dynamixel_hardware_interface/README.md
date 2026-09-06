# **Dynamixel Hardware Interface User Guide**

## **1. Introduction**

ROS 2 package providing a hardware interface for controlling [Dynamixel](https://www.dynamixel.com/) motors via the [ros2_control framework](https://github.com/ros-controls/ros2_control). This repository includes the **dynamixel_hardware_interface plugin** for seamless integration with ROS 2 control, along with the [dynamixel_interfaces](https://github.com/ROBOTIS-GIT/dynamixel_interfaces) package containing custom message definitions used by the interface

## OM6DOF local safety patch

This vendored package contains local changes used by the OM6DOF Mode 0 leader
arm. It is not byte-identical to the upstream ROBOTIS package. The complete
system context and commissioning evidence are documented in
[`../docs/leader_arm_gravity_compensation.md`](../docs/leader_arm_gravity_compensation.md).

The local patch adds or strengthens:

- optional startup with automatic Torque Enable disabled;
- validation that Current Mode has an explicit zero Goal Current;
- checked initial/cyclic writes and write-failure handling;
- finite command rejection and torque-off requests on critical failures;
- read-before-write behavior for EEPROM limit values;
- Goal Current readback and corrected torque-state reporting;
- restriction of generic writes to safety-critical control-table items;
- current-command zero checks before a torque-enable request;
- command/state count validation on either-side mismatch;
- replacement of leaked per-cycle raw buffers with managed storage.

For the leader profile, hardware parameters must include at least:

```xml
<param name="disable_torque_at_init">true</param>
<param name="auto_enable_torque_on_start">false</param>
<param name="restrict_critical_write_service">true</param>
```

Operating Mode 0 also requires an explicit Goal Current of zero after the mode
change and before Torque Enable. ROBOTIS documents that changing Operating Mode
resets Goal Current to Current Limit, so relying on an uninitialized first
control-loop write is unsafe.

Important limitations remain:

- A service response may confirm that a request was queued, not that every
  servo register has changed. Verify `dxl_state` feedback.
- The ROS interface name `effort` does not define physical units. In the
  OM6DOF leader profile it carries mA; N·m-to-mA conversion occurs in the
  controller.
- Bus Watchdog protects against lost instruction traffic, not against a wrong
  command that is still transmitted.
- Managed buffers remove a leak but do not make every cyclic path hard-real-time.
- These interlocks are not a certified safety function. Maintain mechanical
  support and reachable power removal during commissioning.


## 2. **Prerequisites**

This package currently supports ROS 2 Humble, Jazzy, Rolling. Ensure that ROS 2 is properly installed.

- Hardware Requirements:

  - Dynamixel servos
  - USB2 Dynamixel or U2D2 adapter
  - Proper power supply for Dynamixel motors


## **3. Installation**

1. Clone the repository into your ROS workspace:

   ```bash
   cd ~/${WORKSPACE}/src
   git clone -b ${ROS_DISTRO} https://github.com/ROBOTIS-GIT/DynamixelSDK.git
   git clone -b ${ROS_DISTRO} https://github.com/ROBOTIS-GIT/dynamixel_hardware_interface.git
   git clone -b ${ROS_DISTRO} https://github.com/ROBOTIS-GIT/dynamixel_interfaces.git
   ```

2. Build the package:

   ```bash
   cd ~/${WORKSPACE}
   colcon build
   ```

3. Source your workspace:

   ```bash
   source ~/${WORKSPACE}/install/setup.bash
   ```


## 4. Currently Used Packages

This project integrates with the following ROS 2 packages to provide extended functionality:

- **[open_manipulator](https://github.com/ROBOTIS-GIT/open_manipulator)**
  A ROS-based open-source software package designed for the **OpenManipulator-X and OMY**. It provides essential features like motion planning, kinematics, and control utilities for seamless integration with ROS 2 environments.

## 5. Configuration

To effectively use the **Dynamixel Hardware Interface** in a ROS 2 control system, you need to configure specific parameters in your `ros2_control` hardware description file. Below is a concise explanation of the key parameters, illustrated with examples from the **OpenManipulator-X ROS 2 control.xacro** file.

1. **Port Settings**: Define serial port and baud rate for communication.
2. **Hardware Setup**: Configure joints and transmissions.
3. **Joints**: Control and monitor robot joints.
4. **GPIO**: Define and control Dynamixel motors.

#### **1. Port and Communication Settings**

These parameters define how the interface communicates with the Dynamixel motors:

- **`port_name`**: Serial port for communication.

- **`baud_rate`**: Communication baud rate.

- **`error_timeout_ms`**: Timeout for communication errors (in milliseconds).

- **`read_packet_timeout_ms`**: Receive deadline for one complete
  read response, in milliseconds. It is independent of the
  `ros2_control` update period and stays bounded even after a scheduler stall.
  Values must be finite and between `5` and `100` ms; invalid configuration
  stops initialization. The default is `30` ms. This is distinct from
  `error_timeout_ms`: it changes how long one packet read may wait, while the
  existing consecutive-failure and elapsed-error fail-safes remain unchanged.

- **`read_transport_mode`**: Selects `multi_sync` (the backward-compatible
  library default) or `sequential_single_sync`. The sequential mode creates
  one persistent, one-ID `GroupSyncRead` handler per communication ID. It
  acquires every response before updating any exported state pointer, so a
  failure cannot expose a mixed old/new joint snapshot. Invalid values and
  layouts that cannot use the common indirect SyncRead block abort hardware
  initialization; the driver never silently falls back to multi-ID traffic.

#### Read-only OM6DOF bus isolation

`dxl_read_diagnostic` compares four read paths without changing any Dynamixel
register:

1. Individual reads of Present Position (`132`, 4 bytes).
2. Individual reads of the driver's indirect block (`634`, 14 bytes).
3. Plain GroupSyncRead of Present Position (`132`, 4 bytes).
4. Plain GroupSyncRead of the driver's exact indirect block (`634`, 14 bytes).

Immediately before and after those trials, it also records a read-only health
snapshot for every safety ID: Hardware Error Status (`70`, 1 byte), Present
Input Voltage (`144`, 2 bytes, decoded at `0.1 V` per unit), and Present
Temperature (`146`, 1 byte, decoded in degrees Celsius). JSON output retains
both the raw register value and its decoded form per ID. Any communication or
status-packet error, or any nonzero Hardware Error Status, makes the overall
result non-pass; voltage and temperature are reported without imposing a
model-specific threshold.

It is deliberately fail-closed. The tool requires
`om6dof-hardware.service` to be conclusively `inactive`, rejects a visibly
owned port, obtains `flock` and `TIOCEXCL`, and repeats the ownership check
after locking and after the trials. Before any diagnostic phase, it reads
Torque Enable (`64`, 1 byte) from the immutable complete rig roster, IDs
`31,32,33,24,35,26,37`. Any missing/error response or any value other than zero
aborts the run. This full gate remains in force when `--ids` selects a test
subset for bisection. The tool does **not** disable torque itself and contains
no register-write API.

Support the arm mechanically before making the hardware owner inactive:
with torque already off, gravity can move the arm. After building and sourcing
the workspace, run:

```bash
ros2 run dynamixel_hardware_interface dxl_read_diagnostic \
  --trials 500 \
  > /tmp/om6dof_dxl_read_diagnostic.json
```

For a statistically useful comparison with the production 14-byte SyncRead,
skip the three unrelated phases and preserve the requested ID order:

```bash
ros2 run dynamixel_hardware_interface dxl_read_diagnostic \
  --phase driver-group-only \
  --ids 31,32,33,24,35,26,37 \
  --trials 10000 \
  > /tmp/om6dof_dxl_group_only.json
```

Each failed group transaction reports `expected_id`, `expected_index`, RX
elapsed time, and bytes still queued after failure. `expected_id` means the
response the SDK was waiting for; it is not automatically proof that this
motor transmitted the corrupt bytes. Repeat with reversed order and selected
subsets: a failure that follows the same ID points toward that actuator or its
cable path, while one that follows the same list index points toward response
ordering or aggregate timing. The immutable seven-ID torque gate is still
checked even when the test uses a subset.

The fixed defaults are Protocol 2.0, 1 Mbps, the rig's stable FTDI by-id path,
test IDs `31,32,33,24,35,26,37`, and a 30 ms receive deadline. `--port`, `--ids`,
`--phase`, `--trials`, `--timeout-ms`, and `--interval-ms` are configurable.
The guarded service name is intentionally not configurable. Any state other
than exactly `inactive`, including `unknown`, is rejected.

Output is one JSON document. Exit `0` means every read succeeded; `2` means a
service/ownership/configuration guard refused access; `3` means torque-off
could not be conclusively established; and `4` means the guarded trials ran
but recorded read or health errors. Group-only failures implicate aggregate
packet timing/USB scheduling or multi-response bus behavior; failures isolated
to one ID also direct inspection toward that servo and its upstream cable
segment.

The commissioned OM6DOF profile uses `sequential_single_sync`. Guarded testing
on 2026-09-04 recorded intermittent failures for multi-ID GroupSyncRead in
natural, reverse, and sorted orders, while seven separate one-ID GroupSyncRead
runs completed 70,000/70,000 transactions without a read error. The mode is a
software mitigation for that rig, not proof that the underlying electrical or
multi-responder timing fault has been repaired. Generic users retain
`multi_sync` unless they opt in explicitly.

#### **2. Hardware Configuration**

These parameters define the hardware setup:

- **`number_of_joints`**: Total number of joints.

- **`number_of_transmissions`**: Number of transmissions.

- **Transmission Matrices**: Define joint-to-transmission mappings.

#### **3. Joint Configuration**

Joints define the control and state interfaces for robot movement:

##### **Key Attributes**

- **`name`**: Unique joint name.
   Example: `${prefix}joint1`

##### **Sub-Elements**

1. **`<command_interface>`**: Sends commands to joints.

   ```xml
   <command_interface name="position">
   ```

2. **`<state_interface>`**: Monitors joint state data.

   ```xml
   <state_interface name="position"/>
   <state_interface name="velocity"/>
   <state_interface name="effort"/>
   ```


#### **4. GPIO Configuration**

The GPIO tag is used to define the configuration of Dynamixel motors in a robotics system. It serves as a declarative structure to set up motor-specific parameters, command interfaces, and state monitoring capabilities. This allows seamless integration of Dynamixel hardware with software frameworks.


##### **Key Attributes**

- **`name`**: A unique identifier for the motor configuration (e.g., `dxl1`).
- **`ID`**: The unique ID assigned to the motor in the Dynamixel network (e.g., `11`).


##### **Sub-Elements**

1. **`<param>`**: Specifies motor-specific settings. These parameters correspond to the properties of the Dynamixel motor, such as its type, control mode, or PID gain values.

   ```
   <param name="type">dxl</param>
   ```

2. **`<command_interface>`**: Defines the control commands that can be sent to the motor. For example, setting the desired goal position.

   ```
   <command_interface name="Goal Position"/>
   ```

3. **`<state_interface>`**: Specifies the state feedback interfaces to monitor real-time motor data, such as position, velocity, and current.

   ```
   <state_interface name="Present Position"/>
   <state_interface name="Present Velocity"/>
   <state_interface name="Present Current"/>
   ```

##### **Example GPIO Configuration**

Below is an example of a fully defined GPIO configuration for a Dynamixel motor. This example demonstrates how to configure a motor with ID `11`, define command interfaces, monitor state data, and set additional parameters such as PID gains and drive mode.

```
<gpio name="dxl1">
  <param name="type">dxl</param>
  <param name="ID">11</param>
  <command_interface name="Goal Position"/>
  <state_interface name="Present Position"/>
  <state_interface name="Present Velocity"/>
  <state_interface name="Present Current"/>
  <param name="Position P Gain">800</param> <!-- Proportional gain for position control -->
  <param name="Position I Gain">100</param> <!-- Integral gain for position control -->
  <param name="Position D Gain">100</param> <!-- Derivative gain for position control -->
  <param name="Drive Mode">0</param> <!-- 0: Clockwise, 1: Counterclockwise -->
</gpio>
```

##### **Dynamixel Control Table Reference**

The Dynamixel hardware interface uses control tables, defined in model-specific files such as `xm430_w350.model`, to configure and interact with the motor's internal settings. These control tables map hardware parameters to specific memory addresses and data types, enabling fine-grained control and monitoring.

Example from `xm430_w350.model`:

```
[Control Table]
Address   Size    Data Name
0         2       Model Number
2         4       Model Information
6         1       Firmware Version
7         1       ID
...
```

##### **Usage**

- The control table specifies the internal memory layout of the Dynamixel motor.
- For instance, you can set the motor ID at address `7`, or configure firmware-specific options at address `6`.

These settings can be defined within the GPIO configuration or dynamically updated through commands based on the control table schema.

This professional explanation highlights the flexibility and precision of the Dynamixel hardware interface, empowering developers to fully utilize their motor's capabilities within a structured framework. For further details, refer to the [official Dynamixel e-Manual](https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/#control-table-of-eeprom-area).


## **6. Usage**

Ensure the parameters are configured correctly in your `ros2_control` YAML file or XML launch file.

- Example Parameter Configuration

  ```xml
  <ros2_control>
      <param name="dynamixel_state_pub_msg_name">dynamixel_hardware_interface/dxl_state</param>
      <param name="dynamixel_health_topic">dynamixel_hardware_interface/health</param>
      <param name="dynamixel_health_status_name">dynamixel_hardware_interface/BusHealth</param>
      <param name="get_dynamixel_data_srv_name">dynamixel_hardware_interface/get_dxl_data</param>
      <param name="set_dynamixel_data_srv_name">dynamixel_hardware_interface/set_dxl_data</param>
      <param name="reboot_dxl_srv_name">dynamixel_hardware_interface/reboot_dxl</param>
      <param name="set_dxl_torque_srv_name">dynamixel_hardware_interface/set_dxl_torque</param>
  </ros2_control>
  ```

#### Topic and Service Descriptions

##### 1. **dynamixel_state_pub_msg_name**

- **Description**: Defines the topic name for publishing **the Dynamixel state.**

- **Default Value**: `dynamixel_hardware_interface/dxl_state`

##### 2. **dynamixel_health_topic / dynamixel_health_status_name**

- **Description**: Publishes a reliable, transient-local
  `diagnostic_msgs/DiagnosticArray` at up to 20 Hz. The named status contains
  persistent `read_failure_count` and `write_failure_count` counters, current
  and last communication errors, consecutive failure counts, a driver-instance
  ID, fail-safe state, aggregate hardware-error mask, all-actuator torque
  state, and input-voltage feedback.

- **Default Values**: `dynamixel_hardware_interface/health` and
  `dynamixel_hardware_interface/BusHealth`

The counters never clear after communication recovers (until the driver
process restarts), so safety consumers can require an unchanged clean window
without depending on delivery of one short-lived error frame. A new
`driver_instance_id` tells consumers to discard the old clean window.

`hardware_error_expected_count`, `hardware_error_monitored_count`, and
`hardware_error_monitoring_complete` distinguish a real all-zero set of
Hardware Error Status registers from a missing state-interface configuration.
Incomplete coverage forces the diagnostic level to `ERROR`; it can never be
reported as a healthy zero mask. Likewise, the input-voltage fields report
expected/monitored counts, a completeness flag, and the minimum voltage in
volts together with its Dynamixel ID. Voltage is already part of the cyclic
read, so these fields add no serial transaction. They expose the minimum for a
supervisory threshold without imposing one model-independent limit in this
driver.

For OM6DOF, all seven physical GPIO entries must declare both
`Present Input Voltage` and `Hardware Error Status`. With the other standard
state items this is a 14-byte indirect SyncRead block per actuator.

##### 3. **get_dynamixel_data_srv_name**

- **Description**: Specifies the service name for retrieving Dynamixel data.

- **Default Value**: `dynamixel_hardware_interface/get_dxl_data`

##### 4. **set_dynamixel_data_srv_name**

- **Description**: Specifies the service name for setting Dynamixel data.

- **Default Value**: `dynamixel_hardware_interface/set_dxl_data`

##### 5. **reboot_dxl_srv_name**

- **Description**: Specifies the service name for rebooting Dynamixel motors.

- **Default Value**: `dynamixel_hardware_interface/reboot_dxl`

##### 6. **set_dxl_torque_srv_name**

- **Description**: Specifies the service name for enabling or disabling torque on Dynamixel motors.

- **Default Value**: `dynamixel_hardware_interface/set_dxl_torque`


## **7. Contributing**

We welcome contributions! Please follow the guidelines in [CONTRIBUTING.md](CONTRIBUTING.md) to submit issues or pull requests.


## **8. License**

This project is licensed under the Apache License 2.0. See the [LICENSE](LICENSE) file for details.
