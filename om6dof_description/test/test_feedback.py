from control_msgs.msg import DynamicJointState, InterfaceValue
import pytest
from urdf_parser_py.urdf import URDF

from om6dof_description.feedback import decode_dynamic_state, motors_from_description, motor_text


DESCRIPTION = '''<robot name="fixture">
  <link name="base"/><link name="tip"/>
  <joint name="joint" type="continuous">
    <parent link="base"/><child link="tip"/><axis xyz="0 0 1"/>
  </joint>
  <ros2_control name="hardware" type="system">
    <hardware><plugin>dynamixel_hardware_interface/DynamixelHardware</plugin>
      <param name="transmission_to_joint_matrix">1</param>
    </hardware>
    <joint name="joint"><state_interface name="position"/></joint>
    <gpio name="dxl7"><param name="type">dxl</param><param name="ID">37</param>
      <param name="[unit info]">Present Current,2.69,mA,signed,0.0;</param>
    </gpio>
  </ros2_control>
</robot>'''


def test_motor_mapping_and_driver_current_units():
    motor, = motors_from_description(DESCRIPTION, URDF.from_xml_string(DESCRIPTION))
    assert (motor.joint, motor.link, motor.gpio, motor.motor_id, motor.current_unit) == (
        'joint', 'tip', 'dxl7', '37', 'mA')
    text = motor_text(motor, {
        'joint': {'position': 0.1, 'velocity': -0.2, 'effort': 999.0},
        'dxl7': {'Present Current': -2.69, 'Present Input Voltage': 12.0, 'Torque Enable': 0.0},
    })
    assert 'I=-2.690mA' in text
    assert 'dq=-0.200rad/s' in text
    assert 'torque=OFF' in text
    assert '999' not in text
    assert 'Nm' not in text


def test_raw_current_does_not_get_gripper_conversion():
    description = DESCRIPTION.replace('Present Current,2.69,mA,signed,0.0;', '')
    motor, = motors_from_description(description, URDF.from_xml_string(description))
    assert motor.current_unit == 'raw'
    assert 'I=-1.000raw' in motor_text(motor, {'dxl7': {'Present Current': -1.0}})


def test_missing_or_nan_data_is_unavailable_not_zero():
    motor, = motors_from_description(DESCRIPTION, URDF.from_xml_string(DESCRIPTION))
    message = DynamicJointState(
        joint_names=['dxl7', 'joint'],
        interface_values=[
            InterfaceValue(interface_names=['Present Current'], values=[float('nan')]),
            InterfaceValue(interface_names=['velocity', 'position'], values=[-0.4, 0.7]),
        ])
    states = decode_dynamic_state(message)
    assert states['joint']['position'] == 0.7
    assert states['joint']['velocity'] == -0.4
    assert 'I=unavailable' in motor_text(motor, states)


def test_malformed_feedback_is_rejected():
    message = DynamicJointState(joint_names=['joint'])
    with pytest.raises(ValueError, match='mismatched'):
        decode_dynamic_state(message)
