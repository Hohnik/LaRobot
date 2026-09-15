"""Run both real camera factories with fake handles and partial-startup failures."""
from contextlib import ExitStack, redirect_stdout
import importlib.util
import io
from pathlib import Path
import sys
from types import SimpleNamespace
from unittest.mock import patch

import cv2
from yam import platform
from yam.cameras import identity

REPO = Path(__file__).resolve().parent.parent
spec = importlib.util.spec_from_file_location("camera_startup_app", REPO / "apps/teleop_session.py")
app = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = app
spec.loader.exec_module(app)


def exercise(*, linux=True, failure=None, specs="0,1", stop_error=False):
    events = []

    class Capture:
        def __init__(self, index):
            self.index = index
            events.append(f"open_{index}")

        def isOpened(self):
            return not (failure == "open" and self.index == 1)

        def release(self):
            events.append(f"release_{self.index}")

    class Reader:
        def __init__(self, cap):
            if failure == "reader" and cap.index == 1:
                raise RuntimeError("reader failed")
            self.cap = cap

        def stop(self):
            events.append(f"stop_{self.cap.index}")
            self.cap.release()
            if stop_error:
                raise RuntimeError("stop failed")

    def lookup(spec, listed):
        if failure == "lookup" and spec == "1":
            raise ValueError("lookup failed")
        i = int(spec)
        return SimpleNamespace(serial="", index=i, device=f"/dev/video{i}",
                               capture_nodes=(), index_reason="colour-format")

    def measured(cap):
        if failure == "measure" and cap.index == 1:
            raise OSError("measurement failed")
        return SimpleNamespace(line=lambda: "30 fps", stepped_down=False, slow=False)

    def mac_open(index, *args):
        if failure == "open" and index == 1:
            return None
        return Capture(index)

    def forbidden(*args):
        raise AssertionError("No real camera discovery is allowed")

    fake_mac = SimpleNamespace(
        CameraLookupError=ValueError, hinted_index=forbidden, mac_cameras=forbidden,
        model_discriminating_mode=forbidden, open_camera=mac_open,
        resolve_camera=forbidden)
    result, error = None, None
    with ExitStack() as stack:
        stack.enter_context(patch.object(platform, "IS_LINUX", linux))
        stack.enter_context(patch.object(platform, "read_v4l_cameras", lambda: []))
        stack.enter_context(patch.object(platform, "dynamic_framerate_allowed", lambda *a: False))
        stack.enter_context(patch.object(identity, "linux_camera_for_spec", lookup))
        stack.enter_context(patch.object(identity, "read_ioreg", forbidden))
        stack.enter_context(patch.object(cv2, "VideoCapture", Capture))
        stack.enter_context(patch.object(app, "open_measured", measured))
        stack.enter_context(patch.object(app, "FrameGrabber", Reader))
        stack.enter_context(patch.dict(sys.modules, {"camera_view": fake_mac}))
        with redirect_stdout(io.StringIO()):
            try:
                result = app.open_session_cameras(specs)
            except BaseException as exc:
                error = exc
    return result, error, events


def test_linux_success_transfers_both_readers_to_capture_set():
    result, error, events = exercise()
    assert error is None and events == ["open_0", "open_1"]
    result[0].stop()
    assert events[-4:] == ["stop_0", "release_0", "stop_1", "release_1"]


def test_linux_later_lookup_failure_stops_first_reader():
    result, error, events = exercise(failure="lookup")
    assert isinstance(error, SystemExit)
    assert events == ["open_0", "stop_0", "release_0"]


def test_linux_duplicate_name_stops_first_reader():
    result, error, events = exercise(specs="0,0")
    assert isinstance(error, SystemExit) and events == ["open_0", "stop_0", "release_0"]


def test_linux_unopened_capture_and_earlier_reader_are_released():
    result, error, events = exercise(failure="open")
    assert isinstance(error, SystemExit)
    assert events.count("release_0") == 1 and events.count("release_1") == 1


def test_linux_measurement_exception_releases_raw_capture_and_reader():
    result, error, events = exercise(failure="measure")
    assert isinstance(error, OSError)
    assert events.count("release_0") == 1 and events.count("release_1") == 1


def test_reader_constructor_failure_releases_both_devices():
    result, error, events = exercise(failure="reader")
    assert isinstance(error, RuntimeError)
    assert events.count("release_0") == 1 and events.count("release_1") == 1


def test_reader_stop_error_does_not_skip_remaining_raw_capture():
    result, error, events = exercise(failure="measure", stop_error=True)
    assert isinstance(error, OSError) and "stop failed" in str(error.__notes__)
    assert events.count("release_1") == 1


def test_mac_success_transfers_readers_without_closing_them():
    result, error, events = exercise(linux=False)
    assert error is None and events == ["open_0", "open_1"]
    result[0].stop()
    assert events.count("release_0") == 1 and events.count("release_1") == 1


def test_mac_second_open_failure_stops_first_reader():
    result, error, events = exercise(linux=False, failure="open")
    assert isinstance(error, SystemExit) and events == ["open_0", "stop_0", "release_0"]


def test_mac_duplicate_name_stops_first_reader():
    result, error, events = exercise(linux=False, specs="0,0")
    assert isinstance(error, SystemExit) and events == ["open_0", "stop_0", "release_0"]


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
