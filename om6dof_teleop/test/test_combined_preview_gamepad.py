import math
from types import SimpleNamespace

import pytest

from om6dof_teleop.combined_preview_gamepad import PreviewGamepad, joystick_twist


def test_neutral_and_trigger_mapping():
    axes = [0, 0, -1, 0, 0, -1]
    assert joystick_twist(axes, set()) == [0]*6
    axes[5] = 1
    assert joystick_twist(axes, set())[5] == .35


def test_simultaneous_inputs_respect_norm_limits():
    v = joystick_twist([1, 1, -1, 1, 1, 1], {0})
    assert math.sqrt(sum(x*x for x in v[:3])) <= .05 + 1e-12
    assert math.sqrt(sum(x*x for x in v[3:])) <= .5 + 1e-12


def test_non_motion_buttons_do_not_create_motion():
    assert joystick_twist([0, 0, -1, 0, 0, -1], {4, 5, 6, 7, 8}) == [0]*6


@pytest.mark.parametrize('axes', [[], [0]*5, [math.nan]*6])
def test_invalid_device_input_rejected(axes):
    with pytest.raises(ValueError):
        joystick_twist(axes, set())


def test_disconnect_requires_neutral_before_reconnect_motion():
    node = object.__new__(PreviewGamepad)
    published = []
    node.armed = True
    state = [False, [], set()]
    node.device = SimpleNamespace(snapshot=lambda: state)
    node.publisher = SimpleNamespace(publish=published.append)
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(
        to_msg=lambda: __import__('builtin_interfaces.msg', fromlist=['Time']).Time()))
    node._tick()
    assert not node.armed and not published
    state[:] = [True, [0, 1, -1, 0, 0, -1], set()]
    node._tick()
    assert published[-1].twist.linear.x == 0
    state[1][1] = 0
    node._tick()
    assert node.armed
    state[1][1] = 1
    node._tick()
    assert published[-1].twist.linear.x == -.03
