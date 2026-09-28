from pathlib import Path

import numpy as np
import pytest

from robot.cameras.sim_camera import SimCamera
from robot.environment.simulation import Simulation

SCENE = str(Path(__file__).parents[1] / "assets/put_bottles/put_bottle.xml")


def test_read_returns_the_cached_frame_until_the_interval_passed():
    sim = Simulation(SCENE)
    with SimCamera(sim, name="overhead", width=32, height=32, fps=10) as cam:
        first = cam.read()
        assert first.rgb.shape == (32, 32, 3)
        assert first.rgb.dtype == np.uint8
        assert first.timestamp_ns_capture == int(sim.data.time * 1e9)
        assert isinstance(first.timestamp_ns_capture, int)
        assert isinstance(first.timestamp_ns_host, int)
        assert first.timestamp_ns_host > 0
        assert cam.read() is first  # same tick: no re-render
        for _ in range(4):  # 4/30 s of sim time is more than 1/10 s
            action = np.zeros(sim.model.nu)
            left, right = action[0:7], action[7:]
            sim.step(left, right)
        latest = cam.read()
        assert latest is not first
        assert latest.timestamp_ns_capture == int(sim.data.time * 1e9)
        assert latest.timestamp_ns_capture > first.timestamp_ns_capture
        assert latest.timestamp_ns_host >= first.timestamp_ns_host


def test_read_outside_the_connect_close_cycle_fails():
    """A disconnected camera must fail loudly, never serve stale frames."""
    sim = Simulation(SCENE)
    cam = SimCamera(sim, name="overhead", width=32, height=32)
    with pytest.raises(RuntimeError):
        _ = cam.read()  # before connect
    cam.connect()
    cam.close()
    with pytest.raises(RuntimeError):
        _ = cam.read()  # after close, even though a cached frame exists
