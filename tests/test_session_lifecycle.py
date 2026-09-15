"""Exercise the application's real startup/cleanup path with injected devices.

No hardware is opened. Failures occur before the control loop starts. These tests
cover resource ownership, rather than copying the application's stopping policy.
"""
from __future__ import annotations

from contextlib import ExitStack, redirect_stdout
import importlib.util
import io
from pathlib import Path
import signal
import sys
from tempfile import TemporaryDirectory
import threading
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

REPO = Path(__file__).resolve().parent.parent
spec = importlib.util.spec_from_file_location("lifecycle_app", REPO / "apps/teleop_session.py")
app = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = app
spec.loader.exec_module(app)


def exercise(*, failure="build_G", sim=True, dry=False, missing_puck=False,
             bad_puck=None, bad_close=None, bad_shutdown=None, incomplete_shutdown=False,
             camera_failure=False):
    events = []
    incidents = []
    output = io.StringIO()

    def forbidden(*args, **kwargs):
        raise AssertionError("Physical devices are forbidden in this test")

    def camera_stop():
        events.append("camera_stop")
        if camera_failure:
            raise OSError("camera stop failed")

    def camera_open(*args):
        events.append("camera_open")
        return SimpleNamespace(stop=camera_stop), []

    class Puck:
        def __init__(self, name):
            self.name = name

        def set_nonblocking(self, enabled):
            if self.name == bad_puck:
                raise OSError("puck configuration failed")

        def close(self):
            events.append("puck_close_" + self.name)
            if self.name == bad_close:
                raise OSError("puck close failed")

    def build(name, **kwargs):
        if failure == "build_" + name:
            raise RuntimeError("injected build failure " + name)
        events.append("build_" + name)

        def read():
            if failure == "read_" + name:
                raise OSError("initial state read failed")
            return np.zeros(7)

        return SimpleNamespace(name=name, get_joint_pos=read), "mock robot"

    def refuse_loop():
        raise RuntimeError("injected failure before entering the control loop")

    def session(robot, **kwargs):
        name = kwargs["name"]
        if failure == "session_" + name:
            raise RuntimeError("session constructor failed")
        if failure == "interrupt_" + name:
            raise KeyboardInterrupt
        return SimpleNamespace(
            name=name, robot=robot, frame=kwargs["frame"],
            axis_map=kwargs["axis_map"], axis_map_at_start=kwargs["axis_map"].copy(),
            base_pose=[0.0] * 7, alive=lambda: True, enter_hold=refuse_loop,
            thermal=SimpleNamespace(max_seen=28, max_jaw_seen=28))

    def shutdown(robot):
        events.append("shutdown_" + robot.name)
        if bad_shutdown == robot.name:
            raise OSError("shutdown failed")
        return [1] if incomplete_shutdown else list(range(1, 8))

    class RefuseLoop:
        def __enter__(self):
            raise RuntimeError("injected failure before entering the control loop")

        def __exit__(self, *args):
            return False

    argv = ["teleop_session.py", "--arms", "B,G", "--start-mode", "hold", "--cameras", "mock"]
    if sim:
        argv.append("--sim")
    if not dry:
        argv.append("--yes")
    old_signal = signal.getsignal(signal.SIGINT)
    old_hook = threading.excepthook
    with TemporaryDirectory() as directory, ExitStack() as stack:
        stack.enter_context(patch.object(sys, "argv", argv))
        stack.enter_context(patch.object(app, "REPO", Path(directory)))
        stack.enter_context(patch.object(app, "sim_camera_error", lambda *args: None))
        stack.enter_context(patch.object(app, "open_session_cameras", camera_open))
        stack.enter_context(patch.object(app, "build_fake_robot", build))
        stack.enter_context(patch.object(app, "build_robot", forbidden if sim else build))
        stack.enter_context(patch.object(app, "ArmSession", session))
        stack.enter_context(patch.object(app, "shutdown_robot", shutdown))
        stack.enter_context(patch.object(app, "KeyReader", RefuseLoop))
        stack.enter_context(patch.object(app, "TAKES_DIR", Path(directory) / "recordings"))
        stack.enter_context(patch.object(app, "write_incident", lambda *a, **k: incidents.append((a, k))))
        stack.enter_context(patch.object(app, "find_all_devices", lambda: [{"path": "B"}, {"path": "G"}]))
        stack.enter_context(patch.object(app, "pick_device_by_wiggle", lambda label, **kw: None if missing_puck else {"path": label}))
        stack.enter_context(patch.object(app, "open_device", lambda info: Puck(info["path"])))
        stack.enter_context(patch.object(app, "countdown_hands_off", lambda *a: None))
        stack.enter_context(patch.object(app, "TwistReader", lambda handle: object()))
        # No test may persist calibration or settings to the repository.
        stack.enter_context(patch.object(app, "MAP_FILE", Path(directory) / "map.json"))
        stack.enter_context(patch.object(app, "BACKUP_FILE", Path(directory) / "map.prev.json"))
        stack.enter_context(patch.object(app, "save_defaults", forbidden))
        with redirect_stdout(output):
            code = app.main()
    assert signal.getsignal(signal.SIGINT) == old_signal
    assert threading.excepthook is old_hook
    return SimpleNamespace(code=code, events=events, text=output.getvalue(), incidents=incidents)


def test_dry_run_acquires_nothing():
    r = exercise(dry=True)
    assert r.code == 0 and r.events == []


def test_first_build_failure_closes_camera_and_reports_failure():
    r = exercise(failure="build_B")
    assert r.code == 1 and r.events == ["camera_open", "camera_stop"]


def test_session_constructor_failure_shuts_down_acquired_robot():
    r = exercise(failure="session_B")
    assert r.code == 1
    assert r.events == ["camera_open", "build_B", "shutdown_B", "camera_stop"]


def test_initial_read_failure_shuts_down_acquired_robot():
    r = exercise(failure="read_B")
    assert r.code == 1 and "shutdown_B" in r.events


def test_second_build_failure_shuts_down_first_robot_before_camera():
    r = exercise()
    assert r.code == 1
    assert r.events == ["camera_open", "build_B", "shutdown_B", "camera_stop"]


def test_second_session_failure_shuts_down_both_handles():
    r = exercise(failure="session_G")
    assert r.events == ["camera_open", "build_B", "build_G", "shutdown_B", "shutdown_G", "camera_stop"]


def test_interrupt_during_session_construction_cleans_up():
    r = exercise(failure="interrupt_B")
    assert r.code == 130 and "shutdown_B" in r.events


def test_one_failed_shutdown_does_not_skip_second_robot():
    r = exercise(failure="session_G", bad_shutdown="B")
    assert r.code == 1 and "shutdown_G" in r.events and r.events[-1] == "camera_stop"


def test_missing_disable_confirmations_are_named():
    r = exercise(incomplete_shutdown=True)
    assert "could not confirm motors [2, 3, 4, 5, 6, 7] disabled" in r.text


def test_camera_failure_happens_after_robot_cleanup():
    r = exercise(camera_failure=True)
    assert r.events.index("shutdown_B") < r.events.index("camera_stop")
    assert r.code == 1 and "could not clean up camera readers" in r.text


def test_missing_puck_stops_camera_before_returning():
    r = exercise(sim=False, missing_puck=True)
    assert r.code == 1 and r.events == ["camera_open", "camera_stop"]


def test_puck_is_owned_before_configuration():
    r = exercise(sim=False, bad_puck="B")
    assert r.code == 1 and r.events == ["camera_open", "puck_close_B", "camera_stop"]


def test_one_puck_close_failure_does_not_skip_other_puck_or_camera():
    r = exercise(sim=False, bad_close="B")
    assert r.events[-3:] == ["puck_close_B", "puck_close_G", "camera_stop"]
    assert r.code == 1


def test_both_handles_cleaned_after_later_startup_failure():
    r = exercise(failure="before_loop")
    assert r.code == 1 and "injected failure before entering" in r.text
    assert r.events[-3:] == ["shutdown_B", "shutdown_G", "camera_stop"]


def test_startup_failure_is_written_as_incident_for_acquired_robot():
    r = exercise(failure="session_B")
    assert r.incidents, r.text
    args, kwargs = r.incidents[0]
    assert "session constructor failed" in repr((args, kwargs))
    assert "acquired_robots" in repr((args, kwargs))


def test_help_renders_without_starting_devices():
    output = io.StringIO()
    with patch.object(sys, "argv", ["teleop_session.py", "--help"]), redirect_stdout(output):
        try:
            app.main()
        except SystemExit as exc:
            assert exc.code == 0
        else:
            raise AssertionError("argparse did not finish help")
    assert "71% of the arm" in output.getvalue()
    assert "--arms" in output.getvalue()


def main():
    tests = [v for k, v in globals().items() if k.startswith("test_") and callable(v)]
    passed = 0
    for test in tests:
        try:
            test()
            passed += 1
            print("✓", test.__name__)
        except Exception as exc:
            print("✗", test.__name__, type(exc).__name__, str(exc))
    print(f"{passed}/{len(tests)} passed")
    return int(passed != len(tests))


if __name__ == "__main__":
    sys.exit(main())
