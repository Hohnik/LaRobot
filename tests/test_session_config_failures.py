"""Filesystem failures in live settings/pose saves must not stop healthy fake arms."""
import copy
import sys
from unittest.mock import patch

from test_session_recording_failures import app, run_session


def test_failed_settings_save_keeps_loop_live_and_can_retry():
    attempts, observed, original = [], [], app.save_defaults
    before = {}
    def save(path, values):
        attempts.append(path)
        if len(attempts) == 1:
            raise OSError('injected settings disk failure')
        original(path, values)
    def cycle(number, arms, root):
        path = app.defaults_path(root)
        if number == 1:
            original(path, {'max_speed': arms[0].robot.max_speed})
            before['bytes'] = path.read_bytes()
        elif number == 2:
            assert path.read_bytes() == before['bytes']
            assert all(one.alive() and one.mode == 'hold' for one in arms)
            observed.append('continued')
        elif number == 3:
            assert app.load_defaults(path)['max_speed'] == arms[0].robot.max_speed
            observed.append('saved')
    with patch.object(app, 'save_defaults', save):
        code, text, *_ = run_session({1: 'n1+s', 2: 's', 4: 'q'}, on_cycle=cycle)
    assert attempts and 'injected settings disk failure' in text
    assert code == 0, f'settings failure ended the session with status {code}'
    assert observed == ['continued', 'saved'] and len(attempts) == 2
    assert 'injected settings disk failure' in text and text.count('SAVED to') == 1


def test_failed_pose_save_preserves_file_and_both_bases_until_retry():
    attempts, observed, original = [], [], app.save_json
    before = {}
    def save(path, values):
        attempts.append(path)
        if len(attempts) == 1:
            raise OSError('injected pose disk failure')
        original(path, values)
    def cycle(number, arms, root):
        if number == 1:
            before['bytes'] = app.PARK_FILE.read_bytes()
            for one in arms:
                one.base_pose = [.123] * 7
                one.slots[app.BASE_SLOT] = list(one.base_pose)
            before['bases'] = [list(one.base_pose) for one in arms]
            before['slots'] = [copy.deepcopy(one.slots) for one in arms]
        elif number == 2:
            assert app.PARK_FILE.read_bytes() == before['bytes']
            assert [one.base_pose for one in arms] == before['bases']
            assert [one.slots for one in arms] == before['slots']
            assert all(one.alive() and one.mode == 'hold' for one in arms)
            observed.append('preserved')
        elif number == 3:
            data = app.load_json(app.PARK_FILE, {})
            for one in arms:
                assert one.slots == app.park_slots(data, one.name)
                assert one.base_pose == one.slots[app.BASE_SLOT]
            assert [one.base_pose for one in arms] != before['bases']
            observed.append('saved')
    with patch.object(app, 'save_json', save):
        code, text, *_ = run_session({1: 'aas0', 2: '0', 4: 'q'}, on_cycle=cycle)
    assert code == 0, f'pose failure ended the session with status {code}'
    assert observed == ['preserved', 'saved'] and len(attempts) == 2
    assert 'injected pose disk failure' in text and text.count('BASE pose (0) saved') == 2


def main():
    tests = [v for k, v in globals().items() if k.startswith('test_') and callable(v)]
    passed = 0
    for test in tests:
        try:
            test(); passed += 1
            print('✓', test.__name__)
        except Exception as exc:
            print('✗', test.__name__, repr(exc))
    print(f'{passed}/{len(tests)} passed')
    return int(passed != len(tests))


if __name__ == '__main__':
    sys.exit(main())
