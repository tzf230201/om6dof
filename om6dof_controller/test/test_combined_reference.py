import math

import numpy as np
import pytest

from om6dof_controller.combined_reference import CombinedReference
from om6dof_controller.control_math import rotation_from_rotvec


def engine(**kwargs):
    model = CombinedReference(lambda q: (q[:3], rotation_from_rotvec(q[3:])), **kwargs)
    model.start([0, 10], [[0]*6, [1, 0, 0, 0, 0, 0]], 100)
    return model


def test_automatic_and_joystick_accumulate_together_then_neutral_holds_offset():
    model = engine()
    model.twist([0, .02, 0, 0, 0, 0], 100, 100)
    first = model.tick(100.1)
    assert first.position == pytest.approx([.01, .002, 0])
    model.twist([0]*6, 100.2, 100.2)
    later = model.tick(101)
    assert later.phase == 1
    assert later.nominal_position == pytest.approx([.1, 0, 0])
    assert later.offset == pytest.approx([0, .004, 0])
    assert later.position == pytest.approx([.1, .004, 0])


def test_packet_rate_does_not_drop_integration_between_timer_ticks():
    model = engine()
    for stamp in (100, 100.02, 100.04, 100.06, 100.08):
        model.twist([0, .02, 0, 0, 0, 0], stamp, stamp)
    ref = model.tick(100.1)
    assert ref.offset == pytest.approx([0, .002, 0])


def test_disconnect_expires_velocity_but_never_clears_offset_or_trajectory():
    model = engine()
    model.twist([.02, 0, 0, 0, 0, 0], 100, 100)
    model.tick(100.1)
    late = model.tick(102)
    assert late.offset == pytest.approx([.006, 0, 0])
    assert late.phase == 2
    assert model.tick(103).offset == pytest.approx(late.offset)


def test_world_rotation_offset_composes_in_world_not_tool():
    model = engine()
    model.start([0, 1], [[0, 0, 0, 0, .5, 0]]*2, 100)
    model.twist([0, 0, 0, 0, 0, .2], 100, 100)
    ref = model.tick(100.2)
    assert ref.rotation == pytest.approx(
        rotation_from_rotvec([0, 0, .04]) @ rotation_from_rotvec([0, .5, 0]))
    assert not np.allclose(ref.rotation,
        rotation_from_rotvec([0, .5, 0]) @ rotation_from_rotvec([0, 0, .04]))


@pytest.mark.parametrize('times,points', [
    ([1, 2], [[0]*6]*2), ([0, 0], [[0]*6]*2),
    ([0, math.nan], [[0]*6]*2), ([0, 1], [[0]*5]*2),
    ([0, 1], [[0]*6, [math.inf]*6]), ([0], [[0]*6]),
])
def test_invalid_trajectory_is_rejected(times, points):
    with pytest.raises(ValueError):
        engine().start(times, points, 100)


@pytest.mark.parametrize('values,stamp,frame', [
    ([0]*5, 100.1, 'world'), ([math.nan]*6, 100.1, 'world'),
    ([.1, 0, 0, 0, 0, 0], 100.1, 'world'),
    ([0, 0, 0, 0, 0, 1], 100.1, 'world'),
    ([0]*6, 99, 'world'), ([0]*6, 101, 'world'),
    ([0]*6, 100, 'world'), ([0]*6, 100.1, 'tool'),
])
def test_bad_twist_stops_old_velocity(values, stamp, frame):
    model = engine()
    model.twist([.02, 0, 0, 0, 0, 0], 100, 100)
    before = model.tick(100.05).offset
    with pytest.raises(ValueError):
        model.twist(values, stamp, 100.1, frame)
    assert model.tick(100.2).offset == pytest.approx(before)


def test_offset_bounds_prevent_windup_and_reverse_takes_effect_immediately():
    model = engine(translation_limit=.01)
    for i in range(20):
        now = 100 + i * .1
        model.twist([.05, 0, 0, 0, 0, 0], now, now)
        model.tick(now + .09)
    assert np.linalg.norm(model.offset) == pytest.approx(.01)
    model.twist([-.05, 0, 0, 0, 0, 0], 102, 102)
    assert model.tick(102.1).offset[0] == pytest.approx(.005)


def test_new_trajectory_preserves_offset_without_replaying_old_twist():
    model = engine()
    model.twist([.02, 0, 0, 0, 0, 0], 100, 100)
    offset = model.tick(100.1).offset
    model.start([0, 1], [[0]*6]*2, 100.2)
    assert model.tick(100.3).offset == pytest.approx(offset)


def test_finishing_trajectory_does_not_disable_joystick():
    model = engine()
    assert model.tick(111).finished
    model.twist([0, .02, 0, 0, 0, 0], 111, 111)
    ref = model.tick(111.1)
    assert ref.finished
    assert ref.position == pytest.approx([1, .002, 0])


def test_reversed_clock_stops_integration():
    model = engine()
    model.tick(100.2)
    with pytest.raises(ValueError, match='clock'):
        model.tick(99)


def test_fk_invalid_rotation_rejected():
    model = CombinedReference(lambda q: (q[:3], np.zeros((3, 3))))
    model.start([0, 1], [[0]*6]*2, 100)
    with pytest.raises(ValueError, match='FK rotation'):
        model.tick(100)
