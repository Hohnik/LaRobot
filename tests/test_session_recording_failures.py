"""Drive actual application keys through a failed save and a refused GUIDE mode."""
from contextlib import ExitStack, redirect_stdout
import importlib.util
import io
from pathlib import Path
import shutil
import sys
from tempfile import TemporaryDirectory
from unittest.mock import patch
from yam.recording import Trajectory
ROOT = Path(__file__).resolve().parent.parent
spec = importlib.util.spec_from_file_location('recording_failure_app', ROOT / 'apps/teleop_session.py')
app = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = app
spec.loader.exec_module(app)

def run_session(schedule, *, fail_save=False):
    output = io.StringIO()
    saved = []
    sessions = []
    real_save = app.save_take
    real_session = app.ArmSession

    class Keys:
        enabled = True
        cycle = 0

        def __enter__(self):
            return self

        def __exit__(self, *args):
            return False

        def drain(self):
            self.cycle += 1
            if self.cycle > 40:
                raise AssertionError('test script failed to reach quit')
            return list(schedule.get(self.cycle, ''))

        def get(self):
            return 'd'

    def record_session(*args, **kwargs):
        one = real_session(*args, **kwargs)
        sessions.append(one)
        return one

    def save(take, *args, **kwargs):
        saved.append(take)
        if fail_save and len(saved) == 1:
            raise OSError('injected disk failure')
        return real_save(take, *args, **kwargs)
    with TemporaryDirectory() as d, ExitStack() as stack:
        root = Path(d)
        shutil.copytree(ROOT / 'config', root / 'config')
        stack.enter_context(patch.object(app, 'REPO', root))
        for (name, relative) in [('MAP_FILE', 'config/spacemouse_map.json'), ('BACKUP_FILE', 'config/spacemouse_map.prev.json'), ('PARK_FILE', 'config/park_pose.json'), ('TAKES_DIR', 'recordings')]:
            stack.enter_context(patch.object(app, name, root / relative))
        stack.enter_context(patch.object(app, 'KeyReader', Keys))
        stack.enter_context(patch.object(app, 'ArmSession', record_session))
        stack.enter_context(patch.object(app, 'save_take', save))
        stack.enter_context(patch.object(sys, 'argv', ['teleop_session.py', '--sim', '--arms', 'B,G', '--start-mode', 'hold', '--yes']))
        stack.enter_context(redirect_stdout(output))
        code = app.main()
        path = root / 'recordings/sim/5.json'
        take = Trajectory.load(path) if path.exists() else None
    return (code, output.getvalue(), saved, sessions, take)

def test_failed_save_keeps_same_take_for_a_different_slot():
    (code, text, saved, _, take) = run_session({1: 'w', 5: 'w', 7: '4', 9: '5', 11: 'q'}, fail_save=True)
    assert code == 0, text
    assert len(saved) == 2 and saved[0] is saved[1]
    assert take is not None and len(take) >= 2 and (take.meta['arms'] == ['B', 'G'])
    assert 'injected disk failure' in text and 'still pending' in text
    assert 'recording 5 saved' in text

def test_refused_guide_does_not_print_success_or_change_mode():
    (code, text, _, sessions, _) = run_session({1: 'g', 3: 'q'})
    assert code == 0, text
    assert 'NOT weightless' in text
    assert 'MODE: GUIDE on' not in text
    assert all((one.mode == 'hold' for one in sessions))

def main():
    tests = [v for (k, v) in globals().items() if k.startswith('test_') and callable(v)]
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
