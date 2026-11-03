import pytest

from om6dof_controllers.cartesian_force_bridge import directional_wrench


def test_direction_only_selects_configured_wrench_magnitude():
    assert directional_wrench([.01, -.7, 0, 3, -2, 0], [2, 3, 4, .1, .2, .3]) == [2, -3, 0, .1, -.2, 0]


def test_wrench_limits_reject_negative_value():
    with pytest.raises(ValueError, match='nonnegative'):
        directional_wrench([0] * 6, [1, 1, 1, -1, 1, 1])


def test_directional_wrench_never_uses_deflection_as_force_magnitude():
    assert directional_wrench([0.0001, -999.0, 0, 0, 0, 0], [2, 3, 4, .1, .2, .3]) == [2, -3, 0, 0, 0, 0]
