"""Offline wrapper tests; no ROS nodes, publishers, serial ports, or hardware."""

import hashlib
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import pytest


EXPERIMENT = Path(__file__).resolve().parent
WRAPPER = EXPERIMENT / 'run_cpp.sh'


def run(output, *args):
    if not Path('/opt/ros/humble/setup.bash').is_file():
        pytest.skip('ROS Humble build and xacro dependencies are required')
    return subprocess.run(
        ['bash', str(WRAPPER), str(output), *args],
        text=True, capture_output=True, timeout=180,
    )


@pytest.mark.parametrize('args', [
    ['--model'], ['--model', 'unknown'], ['--spacing-mm'],
    ['--urdf', 'untrusted.xml'],
])
def test_invalid_wrapper_options_do_not_create_output(tmp_path, args):
    output = tmp_path / 'invalid'
    result = run(output, *args)
    assert result.returncode != 0
    assert not output.exists()


@pytest.mark.parametrize('model,robot,payload', [
    ('v1', 'om6dof', 'payload.yaml'),
    ('v2', 'om6dof_v2', 'payload_d435.yaml'),
])
def test_explicit_source_model_and_provenance(tmp_path, model, robot, payload):
    output = tmp_path / model
    result = run(output, '--model', model, '--radius-mm', '1',
                 '--spacing-mm', '25', '--samples', '2', '--seeds', '1',
                 '--iterations', '1')
    assert result.returncode == 0, result.stdout + result.stderr
    assert ET.parse(output / 'model.urdf').getroot().get('name') == robot
    source = EXPERIMENT.parent.parent / 'om6dof_description'
    checksums = (output / 'SHA256SUMS').read_text().splitlines()
    entries = dict(line.split('  ', 1)[::-1] for line in checksums)
    required = [output / 'model.urdf', source / 'urdf' / f'{robot}.urdf.xacro',
                source / 'config' / payload, EXPERIMENT / 'cpp' / 'scan.cpp']
    for path in required:
        assert entries[str(path)] == hashlib.sha256(path.read_bytes()).hexdigest()
    assert f'--model {model}' in (output / 'command.sh').read_text()
    assert (output / 'index.html').is_file()
