import numpy as np
import pytest

from om6dof_gravity_comp.calibrate import separate_gravity_and_friction


def _sweeps(gravity_fn, friction, noise=0.0, seed=0):
    """Two passes over the same span, with a known gravity and friction term."""
    rng = np.random.default_rng(seed)
    positions = np.linspace(-0.3, 0.3, 200)
    forward = [(float(p), float(gravity_fn(p) + friction + noise * rng.normal()))
               for p in positions]
    reverse = [(float(p), float(gravity_fn(p) - friction + noise * rng.normal()))
               for p in reversed(positions)]
    return forward, reverse


def test_separation_recovers_a_known_gravity_and_friction():
    """The half-sum and half-difference must return what was put in."""
    forward, reverse = _sweeps(lambda p: 100.0 * np.cos(p), friction=25.0)
    centres, gravity, friction = separate_gravity_and_friction(forward, reverse, 8)
    assert centres.size == 8
    assert np.allclose(gravity, 100.0 * np.cos(centres), atol=1.0)
    assert np.allclose(friction, 25.0, atol=0.5)


def test_friction_sign_follows_the_direction_of_travel():
    forward, reverse = _sweeps(lambda p: 0.0, friction=-40.0)
    _, gravity, friction = separate_gravity_and_friction(forward, reverse, 5)
    assert np.allclose(gravity, 0.0, atol=1e-6)
    assert np.allclose(friction, -40.0, atol=1e-6)


def test_noise_averages_out_within_a_bin():
    """Binning is what makes this usable on real, noisy current readings."""
    forward, reverse = _sweeps(lambda p: 80.0 * p, friction=30.0, noise=8.0)
    centres, gravity, friction = separate_gravity_and_friction(forward, reverse, 6)
    assert np.allclose(gravity, 80.0 * centres, atol=6.0)
    assert np.allclose(friction, 30.0, atol=6.0)


def test_refuses_sweeps_that_do_not_overlap():
    forward = [(0.0, 1.0), (0.1, 1.0)]
    reverse = [(5.0, 1.0), (5.1, 1.0)]
    with pytest.raises(RuntimeError, match="overlap"):
        separate_gravity_and_friction(forward, reverse, 4)


def test_refuses_empty_sweeps():
    with pytest.raises(RuntimeError, match="no samples"):
        separate_gravity_and_friction([], [(0.0, 1.0)], 4)


def test_endpoint_is_not_silently_dropped():
    """The last bin must include its upper edge, or the extreme of the sweep
    -- often the most loaded pose -- never reaches the fit."""
    forward = [(0.0, 10.0), (1.0, 20.0)]
    reverse = [(1.0, 20.0), (0.0, 10.0)]
    centres, gravity, _ = separate_gravity_and_friction(forward, reverse, 2)
    assert centres.size == 2, "the sample at the upper edge was dropped"
    assert gravity[-1] == pytest.approx(20.0)
