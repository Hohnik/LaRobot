"""Checks for the parts of Simulation that can silently go wrong: the tick clock
and the observation contract."""

import time
from pathlib import Path

import numpy as np
import pytest

from robot import CONTROL_HZ, Observation
from robot.environment.simulation import INIT_POS, Simulation

SCENE = str(Path(__file__).parents[1] / "assets/put_bottles/put_bottle.xml")


@pytest.fixture
def sim():
    return Simulation(SCENE, cams=())


def test_reset_returns_the_init_pose_at_time_zero(sim):
    observation = sim.reset()
    assert isinstance(observation, Observation)
    assert observation.time == 0.0
    assert observation.joints.shape == (14,)
    assert observation.joints.dtype == np.float32
    assert np.allclose(observation.joints, INIT_POS, atol=1e-6)


def test_one_step_advances_exactly_one_tick(sim):
    observation = sim.step(np.array(INIT_POS))
    assert observation.time == pytest.approx(1 / CONTROL_HZ)


def test_the_arms_hold_the_init_pose(sim):
    """The position actuators must hold against gravity, or the arms sag."""
    for _ in range(CONTROL_HZ):  # one second of robot time
        observation = sim.step(np.array(INIT_POS))
    assert np.allclose(observation.joints, INIT_POS, atol=0.05)


def test_images_carry_the_camera_names():
    sim = Simulation(SCENE, cams=("top",), height=32, width=32)
    observation = sim.reset()
    assert list(observation.images) == ["top"]
    assert observation.images["top"].shape == (32, 32, 3)


def test_realtime_step_takes_one_tick_of_wall_clock():
    """The point of _wait_for_tick. Without it this loop runs far too fast."""
    sim = Simulation(SCENE, cams=(), realtime=True)
    start = time.perf_counter()
    for _ in range(10):
        sim.step(np.array(INIT_POS))
    assert time.perf_counter() - start == pytest.approx(10 / CONTROL_HZ, abs=0.05)


def test_a_slow_caller_does_not_stretch_the_tick():
    """An absolute deadline absorbs the caller's compute time. sleep(period) would not."""
    sim = Simulation(SCENE, cams=(), realtime=True)
    start = time.perf_counter()
    for _ in range(10):
        sim.step(np.array(INIT_POS))
        time.sleep(0.01)  # a policy thinking for 10 ms, a third of the tick
    assert time.perf_counter() - start == pytest.approx(10 / CONTROL_HZ, abs=0.05)
