"""Exercise shared camera acquisition and application callers using fake devices."""
import ast
from contextlib import ExitStack, redirect_stdout
import importlib.util
import io
from pathlib import Path
import sys
from types import SimpleNamespace
from unittest.mock import patch

import cv2
from yam import platform
from yam.cameras import discovery, identity, session
from yam.cameras.open import open_camera

ROOT = Path(__file__).resolve().parent.parent
spec = importlib.util.spec_from_file_location('camera_probe_test_app', ROOT / 'apps/capture_probe.py')
probe = importlib.util.module_from_spec(spec)
spec.loader.exec_module(probe)


class Capture:
    def __init__(self, index, events, wrong_mode=False):
        self.index, self.events, self.wrong_mode = index, events, wrong_mode
        self.width, self.height = 0, 0
        events.append(('open', index))
    def isOpened(self):
        return True
    def set(self, prop, value):
        if prop == cv2.CAP_PROP_FRAME_WIDTH:
            self.width = value
        elif prop == cv2.CAP_PROP_FRAME_HEIGHT:
            self.height = value
    def get(self, prop):
        if prop == cv2.CAP_PROP_FRAME_WIDTH:
            return 640 if self.wrong_mode else self.width
        return self.height
    def release(self):
        self.events.append(('release', self.index))


def test_configure_failure_releases_new_handle():
    events = []
    with patch.object(cv2, 'VideoCapture', lambda idx: Capture(idx, events)), patch(
            'yam.cameras.open.configure', side_effect=OSError('configuration failed')):
        try:
            open_camera(3, 1280, 720, 30)
        except OSError:
            pass
        else:
            raise AssertionError('configuration failure hidden')
    assert events == [('open', 3), ('release', 3)]


def test_probe_later_open_failure_closes_earlier_camera():
    events = []
    def opening(idx, *args):
        return Capture(idx, events) if idx == 0 else None
    args = SimpleNamespace(indices=['0', '1'], cameras=[], width=640, height=480, fps=30)
    with patch.object(probe, 'open_camera', opening):
        try:
            probe.open_named(args)
        except SystemExit:
            pass
        else:
            raise AssertionError('second camera should refuse')
    assert events == [('open', 0), ('release', 0)]


def test_probe_duplicate_index_refuses_and_closes_first_camera():
    events = []
    args = SimpleNamespace(indices=['0', '0'], cameras=[], width=640, height=480, fps=30)
    with patch.object(probe, 'open_camera', lambda idx, *a: Capture(idx, events)):
        try:
            probe.open_named(args)
        except SystemExit:
            pass
        else:
            raise AssertionError('duplicate camera silently replaced an owned handle')
    assert events == [('open', 0), ('release', 0)]


def test_probe_reader_failure_releases_raw_and_started_cameras():
    events = []
    def reader(cap):
        if cap.index == 1:
            raise OSError('reader failed')
        return SimpleNamespace(stop=cap.release)
    with patch.object(probe, 'open_camera', lambda idx, *a: Capture(idx, events)), patch.object(
            probe, 'FrameGrabber', reader), patch.object(
            sys, 'argv', ['capture_probe.py', '--indices', '0', '1']):
        try:
            probe.main()
        except OSError:
            pass
        else:
            raise AssertionError('reader failure hidden')
    assert events.count(('release', 0)) == events.count(('release', 1)) == 1


def serial_session(*, wrong=False, hint=7, twins=False, mode=(848, 480)):
    events, output = [], io.StringIO()
    dev = {'serial': '123456', 'location_id': 1, 'vid': 2, 'pid': 3}
    uid = identity.usb_unique_id(1, 2, 3)
    target = SimpleNamespace(unique_id=uid, short='D405')
    result = failure = None
    with ExitStack() as stack:
        stack.enter_context(patch.object(platform, 'IS_LINUX', False))
        stack.enter_context(patch.object(identity, 'read_ioreg', lambda: []))
        stack.enter_context(patch.object(identity, 'devices_matching_serial', lambda *a: [dev, dev] if twins else [dev]))
        stack.enter_context(patch.object(discovery, 'hinted_index', lambda *a: hint))
        stack.enter_context(patch.object(discovery, 'mac_cameras', lambda: [target]))
        stack.enter_context(patch.object(discovery, 'model_discriminating_mode', lambda *a: mode))
        stack.enter_context(patch.object(session, 'open_camera', lambda idx, *a: Capture(idx, events, wrong)))
        stack.enter_context(patch.object(session, 'FrameGrabber', lambda cap: SimpleNamespace(stop=cap.release)))
        stack.enter_context(redirect_stdout(output))
        try:
            result = session.open_session_cameras('d405:123')
        except SystemExit as exc:
            failure = exc
    return result, failure, events, output.getvalue()


def test_serial_prefix_records_full_serial_and_reopens_after_model_check():
    result, failure, events, _ = serial_session()
    assert failure is None and result[1] == ['d405-123456']
    assert events == [('open', 7), ('release', 7), ('open', 7)]
    result[0].stop()
    assert events.count(('release', 7)) == 2


def test_stale_model_hint_refuses_and_releases_probe():
    result, failure, events, _ = serial_session(wrong=True)
    assert result is None and 'STALE' in str(failure)
    assert events == [('open', 7), ('release', 7)]


def test_missing_hint_or_ambiguous_serial_never_opens_camera():
    for kwargs in ({'hint': None}, {'twins': True}):
        result, failure, events, _ = serial_session(**kwargs)
        assert result is None and failure is not None and events == []


def test_model_with_no_distinguishing_mode_keeps_existing_warning_policy():
    result, failure, events, text = serial_session(mode=None)
    assert failure is None and 'cannot be model-checked' in text
    assert events == [('open', 7)]
    result[0].stop()


def test_camera_library_has_no_application_imports_and_hint_stays_in_repo():
    for path in (ROOT / 'src/yam/cameras').glob('*.py'):
        for node in ast.walk(ast.parse(path.read_text())):
            if isinstance(node, ast.ImportFrom):
                assert node.module != 'camera_view' and not (node.module or '').startswith('apps.'), path
            elif isinstance(node, ast.Import):
                assert not any(n.name == 'camera_view' or n.name.startswith('apps.') for n in node.names), path
    assert discovery.HINT_FILE == ROOT / 'config/camera_index_hint.json'
    assert probe.resolve_camera is discovery.resolve_camera
    assert probe.open_camera is open_camera


def main():
    tests = [v for k, v in globals().items() if k.startswith('test_') and callable(v)]
    passed = 0
    for test in tests:
        try:
            test()
            passed += 1
            print('✓', test.__name__)
        except Exception as exc:
            print('✗', test.__name__, repr(exc))
    print(f'{passed}/{len(tests)} passed')
    return int(passed != len(tests))


if __name__ == '__main__':
    sys.exit(main())
