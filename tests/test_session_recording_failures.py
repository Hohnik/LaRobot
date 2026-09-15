"""Exercise actual operator workflows with fake arms and disposable recordings/config."""
from contextlib import ExitStack, redirect_stdout
import importlib.util
import io
from pathlib import Path
import shutil
import sys
import threading
from types import SimpleNamespace
from tempfile import TemporaryDirectory
from unittest.mock import patch
from yam.recording import Trajectory
ROOT = Path(__file__).resolve().parent.parent
spec = importlib.util.spec_from_file_location('recording_failure_app', ROOT / 'apps/teleop_session.py')
app = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = app
spec.loader.exec_module(app)

def run_session(schedule, *, fail_save=False, capture=None, sink_factory=None,
                on_cycle=None, saver=None, max_cycles=40):
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
            if self.cycle > max_cycles:
                raise AssertionError('test script failed to reach quit')
            if on_cycle is not None:
                on_cycle(self.cycle, sessions, root)
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
        return (saver or real_save)(take, *args, **kwargs)
    with TemporaryDirectory() as d, ExitStack() as stack:
        root = Path(d)
        shutil.copytree(ROOT / 'config', root / 'config')
        stack.enter_context(patch.object(app, 'REPO', root))
        for (name, relative) in [('MAP_FILE', 'config/spacemouse_map.json'), ('BACKUP_FILE', 'config/spacemouse_map.prev.json'), ('PARK_FILE', 'config/park_pose.json'), ('TAKES_DIR', 'recordings')]:
            stack.enter_context(patch.object(app, name, root / relative))
        stack.enter_context(patch.object(app, 'KeyReader', Keys))
        stack.enter_context(patch.object(app, 'ArmSession', record_session))
        stack.enter_context(patch.object(app, 'save_take', save))
        argv = ['teleop_session.py', '--sim', '--arms', 'B,G', '--start-mode', 'hold', '--yes']
        if capture is not None:
            # This bypass is confined to the fake-device test: no real discovery.
            argv += ['--cameras', 'test']
            stack.enter_context(patch.object(app, 'sim_camera_error', lambda *a: None))
            stack.enter_context(patch.object(app, 'open_session_cameras', lambda *a: (capture, ['test'])))
        if sink_factory is not None:
            stack.enter_context(patch.object(app, 'FrameSink', sink_factory))
        stack.enter_context(patch.object(sys, 'argv', argv))
        stack.enter_context(redirect_stdout(output))
        code = app.main()
        path = root / 'recordings/sim/5.json'
        take = Trajectory.load(path) if path.exists() else None
    return (code, output.getvalue(), saved, sessions, take)

def test_failed_save_keeps_same_take_for_a_different_slot():
    schedule, owners, phase = {1: 'w', 5: 'w'}, [], []
    original = app.RecordingSession
    def recording():
        owner = original()
        owners.append(owner)
        return owner
    def cycle(number, arms, root):
        owner = owners[0]
        if owner.busy:
            return
        if owner.pending is not None and not phase:
            schedule[number] = '4'
            phase.append('first save')
        elif owner.save_error is not None and phase == ['first save']:
            schedule[number] = '5'
            phase.append('retry')
        elif owner.saved is not None:
            schedule[number] = 'q'
    with patch.object(app, 'RecordingSession', recording):
        code, text, saved, _, take = run_session(schedule, fail_save=True,
                                                on_cycle=cycle, max_cycles=200)
    assert code == 0, text
    assert phase == ['first save', 'retry']
    assert len(saved) == 2 and saved[0] is saved[1]
    assert take is not None and len(take) >= 2 and take.meta['arms'] == ['B', 'G']
    assert 'injected disk failure' in text and 'still pending' in text
    assert 'recording 5 saved' in text

def test_refused_guide_does_not_print_success_or_change_mode():
    (code, text, _, sessions, _) = run_session({1: 'g', 3: 'q'})
    assert code == 0, text
    assert 'NOT weightless' in text
    assert 'MODE: GUIDE on' not in text
    assert all((one.mode == 'hold' for one in sessions))

def test_control_cycles_continue_during_blocked_slot_save():
    release, entered = threading.Event(), threading.Event()
    observed = []
    def saver(*args, **kwargs):
        entered.set()
        if not release.wait(2.):
            raise AssertionError('operator loop blocked on save')
        return real(*args, **kwargs)
    def cycle(number, arms, root):
        if number == 12:
            assert entered.is_set() and not release.is_set()
            assert len(arms) == 2 and all(one.mode == 'hold' for one in arms)
            observed.append(number)
            release.set()
    real = app.save_take
    try:
        code, text, _, _, take = run_session(
            {1: 'w', 5: 'w', 7: '5', 20: 'q'}, saver=saver, on_cycle=cycle)
    finally:
        release.set()
    assert observed == [12] and code == 0, text
    assert take is not None and 'recording 5 saved' in text


def test_camera_completion_blocks_early_save_and_discard_but_keeps_loop_running():
    state = {'finished': False, 'stopped': False}
    class Sink:
        def __init__(self, path, names):
            self.path = path
            path.mkdir(parents=True)
            (path / 'owned.txt').write_text('writer owns this')
        def start(self, names):
            pass
        @property
        def finished(self):
            return state['finished']
        def offer(self, samples):
            assert not state['stopped']
        def request_stop(self):
            state['stopped'] = True
        def poll_stop(self):
            return {'test': {'written': 1, 'dropped': 0, 'write_errors': 0, 'flushed': True}}
    def cycle(number, arms, root):
        if number == 12:
            assert state['stopped'] and all(one.mode == 'hold' for one in arms)
            assert list((root / 'recordings/sim/frames').glob('pending_*/owned.txt'))
            assert not (root / 'recordings/sim/5.json').exists()
            state['finished'] = True
    capture = SimpleNamespace(sample=lambda: {}, stop=lambda: None)
    code, text, saved, _, take = run_session(
        {1: 'w', 5: 'w', 7: '5', 9: 'x', 14: '5', 25: 'q'}, capture=capture,
        sink_factory=Sink, on_cycle=cycle)
    assert code == 0 and len(saved) == 1 and take is not None, text
    assert 'files are still finishing' in text and 'recording 5 saved' in text


def test_camera_sampling_failure_stops_take_and_preserves_arm_modes():
    state = {'stop': False}
    class Sink:
        def __init__(self, path, names):
            path.mkdir(parents=True)
        def start(self, names):
            pass
        @property
        def finished(self):
            return state['stop']
        def offer(self, samples):
            raise OSError('injected camera sampling failure')
        def request_stop(self):
            state['stop'] = True
        def poll_stop(self):
            return {'test': {'written': 0, 'dropped': 0, 'write_errors': 0, 'flushed': True}}
    code, text, saved, arms, _ = run_session(
        {1: 'w', 5: '5', 8: 'x', 15: 'q'},
        capture=SimpleNamespace(sample=lambda: {}, stop=lambda: None), sink_factory=Sink)
    assert code == 0 and not saved and all(one.mode == 'hold' for one in arms), text
    assert 'injected camera sampling failure' in text and state['stop']
    assert 'arm modes are unchanged' in text


def test_camera_writer_startup_failure_is_contained_in_recording():
    def broken(path, names):
        path.mkdir(parents=True)
        raise OSError('injected writer startup failure')
    code, text, saved, arms, _ = run_session(
        {1: 'w', 5: 'x', 15: 'q'},
        capture=SimpleNamespace(sample=lambda: {}, stop=lambda: None), sink_factory=broken)
    assert code == 0 and not saved and all(one.mode == 'hold' for one in arms), text
    assert 'injected writer startup failure' in text


def test_partial_startup_retains_owned_sink_until_its_writer_finishes():
    state = {'finished': False, 'stopped': False}
    class Sink:
        def __init__(self, path, names):
            assert names == []
            path.mkdir(parents=True)
            (path / 'partial.txt').write_text('partial writer owns this')
        def start(self, names):
            raise OSError('second writer startup failed')
        @property
        def finished(self):
            return state['finished']
        def request_stop(self):
            state['stopped'] = True
        def poll_stop(self):
            return {'test': {'written': 0, 'dropped': 0, 'write_errors': 0, 'flushed': True}}
    def cycle(number, arms, root):
        if number == 12:
            assert state['stopped']
            assert list((root / 'recordings/sim/frames').glob('pending_*/partial.txt'))
            state['finished'] = True
    code, text, saved, _, _ = run_session(
        {1: 'w', 5: 'x', 14: 'x', 22: 'q'},
        capture=SimpleNamespace(sample=lambda: {}, stop=lambda: None), sink_factory=Sink,
        on_cycle=cycle)
    assert code == 0 and not saved, text
    assert 'second writer startup failed' in text and 'files are still finishing' in text


def test_quit_preserves_unfinished_camera_files_after_motor_shutdown():
    state = {}
    class Sink:
        def __init__(self, path, names):
            path.mkdir(parents=True)
            state['path'] = path
        def start(self, names):
            pass
        finished = False
        def offer(self, samples):
            pass
        def request_stop(self):
            pass
    def reader_stop():
        assert state['path'].exists()
    code, text, _, _, _ = run_session(
        {1: 'w', 5: 'w', 7: 'q'},
        capture=SimpleNamespace(sample=lambda: {}, stop=reader_stop), sink_factory=Sink)
    assert code == 1, text
    assert text.index('motors confirmed disabled') < text.index('Recording cleanup incomplete')
    assert 'files retained' in text


def test_camera_index_failure_is_reported_without_stopping_operator():
    state = {'stopped': False}
    class Sink:
        def __init__(self, path, names):
            path.mkdir(parents=True)
        def start(self, names):
            pass
        @property
        def finished(self):
            return state['stopped']
        def offer(self, samples):
            pass
        def request_stop(self):
            state['stopped'] = True
        def poll_stop(self):
            raise OSError('injected completion failure')
    code, text, saved, arms, _ = run_session(
        {1: 'w', 5: 'w', 8: '5', 11: 'x', 20: 'q'},
        capture=SimpleNamespace(sample=lambda: {}, stop=lambda: None), sink_factory=Sink)
    assert code == 0 and not saved and all(one.mode == 'hold' for one in arms), text
    assert 'injected completion failure' in text


def test_teleop_entry_runs_workspace_guard_and_keeps_both_arms_live():
    observed = []
    def cycle(number, arms, root):
        if number == 5:
            assert all(one.mode == 'teleop' and one.alive() for one in arms)
            observed.append(number)
    code, text, _, _, _ = run_session({1: 'aat', 9: 'q'}, on_cycle=cycle)
    assert observed == [5] and code == 0, text
    assert 'NameError' not in text


def test_leaving_replay_on_selected_arm_holds_both_replay_arms():
    schedule = {1: 'aal', 2: '5', 3: '\n'}
    state = {}
    def cycle(number, arms, root):
        if number == 1:
            take = Trajectory(meta={'arms': ['B', 'G'], 'joints_per_arm': 7,
                                    'simulated': True})
            start = [v for a in arms for v in a.robot.get_joint_pos()]
            take.append(0., start)
            take.append(10., [v + .1 for v in start])
            folder = root / 'recordings/sim'
            folder.mkdir(parents=True, exist_ok=True)
            take.save(folder / '5.json')
        elif 'cancel' not in state and all(a.mode == 'replay' for a in arms):
            state['cancel'] = number
            schedule[number] = 'ah'  # BOTH -> B; only B receives the mode key.
        elif 'cancel' in state and number == state['cancel'] + 1:
            assert all(a.mode == 'hold' for a in arms)
            state['held'] = True
            schedule[number] = 'q'
    code, text, _, _, _ = run_session(schedule, on_cycle=cycle, max_cycles=200)
    assert code == 0 and state.get('held'), text
    assert 'every replay arm is HOLDING now' in text


def exercise_stop(*, fault=False, interrupt_park=False, rename_quit=False):
    events, incidents = [], []
    original_shutdown = app.shutdown_robot
    original_request = app.StopRequest
    def request(cause, message):
        if rename_quit and cause is app.StopCause.QUIT:
            message = 'operator finished this session'
        return original_request(cause, message)
    def cycle(number, arms, root):
        if fault and number == 1:
            # A fault may quote this phrase; it must still use the fault policy.
            arms[0].thermal.update = lambda *a, **kw: SimpleNamespace(
                warning=None, stop_reason='sensor reported quit requested')
    def park(arms, keys, clamp):
        events.append(('park', [a.name for a in arms]))
        assert all(a.mode == 'hold' and a.alive() for a in arms)
        if interrupt_park:
            raise KeyboardInterrupt
        return 'arrived'
    def shutdown(robot):
        events.append(('shutdown', None))
        return original_shutdown(robot)
    def incident(reason, facts):
        events.append(('incident', None))
        incidents.append((reason, facts))
        return None
    with patch.object(app, 'StopRequest', request), \
            patch.object(app, 'park_arms', park), \
            patch.object(app, 'shutdown_robot', shutdown), \
            patch.object(app, 'write_incident', incident):
        code, text, _, arms, _ = run_session({2: 'q'}, on_cycle=cycle)
    return code, text, events, incidents, arms


def test_application_fault_message_cannot_turn_into_planned_quit():
    code, text, events, incidents, arms = exercise_stop(fault=True)
    assert code == 1, text
    assert events == [('park', ['B', 'G']), ('shutdown', None),
                      ('shutdown', None), ('incident', None)]
    assert incidents[0][1]['stop_cause'] == 'fault'
    assert isinstance(incidents[0][1]['stop_reason'], str)
    assert all(not a.alive() for a in arms)


def test_second_interrupt_during_application_park_still_disables_both_arms():
    code, text, events, incidents, arms = exercise_stop(fault=True, interrupt_park=True)
    assert code == 130, text
    assert events == [('park', ['B', 'G']), ('shutdown', None),
                      ('shutdown', None), ('incident', None)]
    assert incidents[0][1]['stop_cause'] == 'interrupt'
    assert all(not a.alive() for a in arms)


def test_renamed_application_quit_keeps_menu_and_success_status():
    code, text, events, incidents, arms = exercise_stop(rename_quit=True)
    assert code == 0 and not incidents, text
    assert 'stopping: operator finished this session' in text
    assert 'Every arm is HOLDING its pose' in text
    assert events == [('shutdown', None), ('shutdown', None)]
    assert all(not a.alive() for a in arms)


def test_settings_changes_and_revert_reach_both_live_robots():
    initial, observed = {}, []
    def cycle(number, arms, root):
        if number == 1:
            initial['speed'] = arms[0].robot.max_speed
        elif number == 2:
            expected = app.adjust_setting('max_speed', initial['speed'], True)
            assert all(a.robot.max_speed == expected for a in arms)
            observed.append('changed')
        elif number == 4:
            assert all(a.robot.max_speed == initial['speed'] for a in arms)
            observed.append('reverted')
    code, text, _, _, _ = run_session({1: 'n1+', 3: '0', 5: 'q'}, on_cycle=cycle)
    assert code == 0 and observed == ['changed', 'reverted'], text
    assert 'SETTINGS closed — over to the quit menu' in text


def test_settings_mode_key_only_closes_until_pressed_again():
    observed = []
    def cycle(number, arms, root):
        if number == 2:
            assert all(a.mode == 'hold' for a in arms)
            observed.append('closed')
        elif number == 4:
            assert arms[0].mode == 'teleop' and arms[1].mode == 'hold'
            observed.append('teleop')
    code, text, _, _, _ = run_session({1: 'nt', 3: 't', 5: 'q'}, on_cycle=cycle)
    assert code == 0 and observed == ['closed', 'teleop'], text


def test_settings_explicit_save_writes_current_values_to_temporary_config():
    observed = []
    def cycle(number, arms, root):
        if number == 2:
            values = app.load_defaults(app.defaults_path(root))
            assert values['max_speed'] == arms[0].robot.max_speed
            assert 'yes' not in values and 'sim' not in values
            observed.append('saved')
    code, text, _, _, _ = run_session({1: 'n1+s', 3: 'q'}, on_cycle=cycle)
    assert code == 0 and observed == ['saved'], text


def test_cancelled_take_marker_cannot_change_next_pose_prompt_into_playback():
    calls = []
    original = app.ArmSession.begin_path
    def begin(arm, legs, *args, **kwargs):
        calls.append((arm.name, [leg.name for leg in legs]))
        return original(arm, legs, *args, **kwargs)
    def cycle(number, arms, root):
        if number == 1:
            arms[0].slots['1'] = list(arms[0].robot.get_joint_pos())
    with patch.object(app.ArmSession, 'begin_path', begin):
        code, text, _, _, _ = run_session({1: 'pw', 2: 'x', 3: 'p1\n', 5: 'q'}, on_cycle=cycle)
    assert code == 0 and calls == [('B', ['1'])], text
    assert 'COMPOSITE RUN' not in text


def test_multiple_pose_prompt_waits_for_second_confirmation():
    calls, observed = [], []
    original = app.ArmSession.begin_path
    def begin(arm, legs, *args, **kwargs):
        calls.append((arm.name, [leg.name for leg in legs]))
        return original(arm, legs, *args, **kwargs)
    def cycle(number, arms, root):
        if number == 1:
            for digit in ('1', '2'):
                arms[0].slots[digit] = list(arms[0].robot.get_joint_pos())
        elif number == 2:
            assert not calls
            observed.append('waiting')
        elif number == 4:
            assert calls == [('B', ['1', '2'])]
            observed.append('confirmed')
    with patch.object(app.ArmSession, 'begin_path', begin):
        code, text, _, _, _ = run_session({1: 'p12\n', 3: '\n', 5: 'q'}, on_cycle=cycle)
    assert code == 0 and observed == ['waiting', 'confirmed'], text


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
