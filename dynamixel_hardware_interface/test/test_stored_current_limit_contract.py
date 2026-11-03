# Copyright 2026 OM6DOF maintainers.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Offline expansion: never opens a motor port or starts ROS nodes."""
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import xacro

ROOT = Path(__file__).resolve().parents[2]


@pytest.mark.parametrize('args,mode,goal', [
    ({}, '5', 'current_limit'),
    ({'arm_operating_mode': '5'}, '5', 'current_limit'),
    ({'arm_operating_mode': '3'}, '3', None),
    ({'torque_off_diagnostic_mode': 'true'}, '3', None),
    ({'current_control': 'true', 'arm_operating_mode': '0'}, '0', '0'),
])
def test_profiles(args, mode, goal):
    doc = xacro.process_file(
        str(ROOT / 'om6dof_bringup/urdf/om6dof.urdf.xacro'), mappings=args)
    root = ET.fromstring(doc.toxml())
    gpios = root.findall('./ros2_control/gpio')
    for index in range(1, 8):
        gpio = next(g for g in gpios if g.attrib['name'] == f'dxl{index}')
        params = {p.attrib['name']: p.text for p in gpio.findall('param')}
        assert params['Operating Mode'] == (mode if index < 7 else '5')
        assert params.get('Goal Current') == (goal if index < 7 else '45')
        assert 'Current Limit' not in params
        if index < 7 and 'current_control' not in args:
            for key in ('Position P Gain', 'Position I Gain', 'Position D Gain'):
                if index in (1, 4, 6) and mode == '5' and key != 'Position I Gain':
                    assert params[key] == ('150' if key == 'Position P Gain' else '1400')
                else:
                    assert key not in params
        if index < 7 and 'current_control' not in args:
            assert [c.attrib['name'] for c in gpio.findall('command_interface')] == [
                'Goal Position']
        if index == 7:
            assert 'Position P Gain' not in params


def test_supervised_preparation_retains_position_mode():
    source = (ROOT / 'om6dof_controllers/om6dof_controllers/impedance_stack.py').read_text()
    assert "'current_control:=false', 'arm_operating_mode:=3'" in source


def test_dynamic_budget_keeps_pid_and_gripper_and_starts_at_zero():
    path = str(ROOT / 'om6dof_bringup/urdf/om6dof.urdf.xacro')
    normal = ET.fromstring(xacro.process_file(path).toxml())
    dynamic = ET.fromstring(xacro.process_file(
        path, mappings={'gravity_current_budget': 'true'}).toxml())
    hw = dynamic.find('./ros2_control/hardware')
    assert hw.find("param[@name='gravity_current_budget']").text == 'true'
    for i in range(1, 8):
        a = normal.find(f"./ros2_control/gpio[@name='dxl{i}']")
        b = dynamic.find(f"./ros2_control/gpio[@name='dxl{i}']")
        pa = {p.attrib['name']: p.text for p in a.findall('param')}
        pb = {p.attrib['name']: p.text for p in b.findall('param')}
        assert {k: v for k, v in pa.items() if 'Gain' in k} == {
            k: v for k, v in pb.items() if 'Gain' in k}
        assert pb['Operating Mode'] == '5'
        assert 'Current Limit' not in pb
        assert pb['Goal Current'] == ('0' if i < 7 else '45')
        cmds = [c.attrib['name'] for c in b.findall('command_interface')]
        assert ('Goal Current' in cmds) == (i < 7)
        if i < 7:
            joint = dynamic.find(f"./ros2_control/joint[@name='joint{i}']")
            assert {c.attrib['name'] for c in joint.findall('command_interface')} == {
                'position', 'position_offset', 'effort'}


@pytest.mark.parametrize('other', [
    {'arm_operating_mode': '3'}, {'arm_operating_mode': '0'},
    {'current_control': 'true'}, {'torque_off_diagnostic_mode': 'true'},
])
def test_budget_rejects_incompatible_profiles(other):
    with pytest.raises(xacro.XacroException):
        xacro.process_file(str(ROOT / 'om6dof_bringup/urdf/om6dof.urdf.xacro'),
                           mappings={'gravity_current_budget': 'true', **other})
