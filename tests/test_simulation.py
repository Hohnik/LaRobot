import time
import warnings
from pathlib import Path

import numpy as np
import pytest

from robot import CONTROL_HZ
from robot.environment.simulation import INIT_POS, Simulation

SCENE = str(Path(__file__).parents[1] / "assets/put_bottles/put_bottle.xml")


def test_reset_sets_the_init_pose():
    sim = Simulation(SCENE)
    assert np.allclose(sim.state, INIT_POS, atol=0.05)


def test_one_step_advances_one_tick():
    sim = Simulation(SCENE)
    sim.step(np.array(INIT_POS))
    assert sim.data.time == pytest.approx(1 / CONTROL_HZ)


def test_the_arms_hold_the_init_pose():
    """Position actuators must hold against gravity"""
    sim = Simulation(SCENE)
    for _ in range(CONTROL_HZ):  # one second of robot time
        sim.step(np.array(INIT_POS))
    assert np.allclose(sim.state, INIT_POS, atol=0.05)


def test_startup_delay_does_not_count_as_lateness():
    """The realtime clock arms on the first step, so slow setup warns no overrun."""
    sim = Simulation(SCENE, realtime=True)
    time.sleep(0.3)  # viewer and renderer setup take about this long
    start = time.perf_counter()
    with warnings.catch_warnings():
        warnings.simplefilter("error")  # an overrun warning fails the test
        for _ in range(6):
            sim.step(np.array(INIT_POS))
    assert time.perf_counter() - start == pytest.approx(6 / CONTROL_HZ, abs=0.05)
