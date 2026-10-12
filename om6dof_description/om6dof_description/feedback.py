"""Decode named motor feedback and units declared by the running hardware URDF."""

from dataclasses import dataclass
import math
from xml.etree import ElementTree as ET


@dataclass(frozen=True)
class Motor:
    joint: str
    link: str
    gpio: str
    motor_id: str
    current_unit: str
    prismatic: bool


def motors_from_description(description, robot):
    """Match ros2_control joints to GPIOs through its transmission matrix."""
    motors = []
    for control in ET.fromstring(description).findall('ros2_control'):
        hardware = control.find('hardware')
        if hardware is None:
            continue
        plugin = hardware.findtext('plugin', '')
        if plugin != 'dynamixel_hardware_interface/DynamixelHardware':
            continue
        joints = control.findall('joint')
        gpios = [gpio for gpio in control.findall('gpio')
                 if gpio.find("param[@name='type']") is not None
                 and gpio.find("param[@name='type']").text.strip() == 'dxl']
        matrix_text = hardware.findtext("param[@name='transmission_to_joint_matrix']", '')
        matrix = [float(value) for value in matrix_text.replace(',', ' ').split()]
        for index, element in enumerate(joints):
            joint = robot.joint_map.get(element.get('name'))
            if joint is None or joint.type not in ('revolute', 'continuous', 'prismatic'):
                continue
            gpio_name, motor_id, current_unit = '', '?', 'driver units'
            # A current read belongs to an individual motor only when this
            # joint has exactly one transmission in the declared matrix.
            if len(matrix) == len(joints) * len(gpios):
                row = matrix[index * len(gpios):(index + 1) * len(gpios)]
                active = [column for column, value in enumerate(row) if abs(value) > 1e-12]
                if len(active) == 1:
                    gpio = gpios[active[0]]
                    gpio_name = gpio.get('name')
                    motor_id = gpio.findtext("param[@name='ID']", '?').strip()
                    # Shipped XM430 arm driver units are raw current counts.
                    # Explicit per-GPIO overrides (e.g. gripper mA) take priority.
                    current_unit = 'raw'
                    override = gpio.findtext("param[@name='[unit info]']", '')
                    for entry in override.split(';'):
                        fields = [field.strip() for field in entry.split(',')]
                        if len(fields) >= 3 and fields[0] == 'Present Current':
                            current_unit = fields[2]
            motors.append(Motor(joint.name, joint.child, gpio_name, motor_id,
                                current_unit, joint.type == 'prismatic'))
    return motors


def decode_dynamic_state(message):
    """Map by name, never by message ordering; ignore absent/nonfinite samples."""
    if len(message.joint_names) != len(message.interface_values):
        raise ValueError('dynamic_joint_states has mismatched name/value lengths')
    result = {}
    for name, interfaces in zip(message.joint_names, message.interface_values):
        if len(interfaces.interface_names) != len(interfaces.values):
            raise ValueError('dynamic_joint_states has mismatched interface lengths for ' + name)
        result[name] = {key: float(value)
                        for key, value in zip(interfaces.interface_names, interfaces.values)
                        if math.isfinite(value)}
    return result


def motor_text(motor, states):
    joint = states.get(motor.joint, {})
    gpio = states.get(motor.gpio, {})

    def value(row, key, unit):
        return ('%.3f%s' % (row[key], unit)) if key in row else 'unavailable'

    # Compact delimiters keep telemetry narrow when rendered as RViz 3D text.
    lines = ['%s[ID:%s]' % (motor.joint, motor.motor_id)]
    lines.append('q=%s,dq=%s' % (
        value(joint, 'position', 'm' if motor.prismatic else 'rad'),
        value(joint, 'velocity', 'm/s' if motor.prismatic else 'rad/s')))
    # Present Current is motor feedback, not the generic effort field or a
    # command. Missing current is displayed as unavailable, never as zero Nm.
    lines.append('I=%s,V=%s' % (value(gpio, 'Present Current', motor.current_unit),
                               value(gpio, 'Present Input Voltage', 'V')))
    torque = gpio.get('Torque Enable')
    enabled = 'unavailable' if torque is None else ('ON' if torque > 0.5 else 'OFF')
    error = gpio.get('Hardware Error Status')
    lines.append('torque=%s,error=%s' % (
        enabled, 'unavailable' if error is None else str(int(error))))
    return '\n'.join(lines)
