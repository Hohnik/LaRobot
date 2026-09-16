"""Exercise recording prompt transitions independently of motors and disk timing."""
from pathlib import Path
from unittest.mock import patch
import sys

from yam.recording import Trajectory
from yam.recording_store import SavedTake
from yam.ui.recording_prompt import RecordingPrompt


class RecordingDouble:
    def __init__(self):
        self.pending = Trajectory()
        self.pending.append(0, [0] * 7)
        self.pending.append(1, [0] * 7)
        self.active = self.sink = self.frame_error = self.frame_report = None
        self.saved = self.save_error = None
        self.busy = False
        self.requests = []
        self.failure = None

    def poll(self):
        pass

    def request_save(self, root, slot, *, saver):
        if self.failure:
            raise self.failure
        self.requests.append(('save', slot, self.pending, saver))
        self.busy = True

    def request_discard(self):
        self.requests.append(('discard', self.pending))
        self.busy = True


def setup():
    recording, lines = RecordingDouble(), []
    prompt = RecordingPrompt(recording, Path('/unused'), emit=lines.append)
    prompt.open()
    return recording, prompt, lines


def test_closed_prompt_passes_through_keys():
    prompt = RecordingPrompt(RecordingDouble(), Path('/unused'))
    assert not prompt.active and not prompt.handle('5')


def test_reaim_requires_confirmation_of_the_new_occupied_slot():
    recording, prompt, lines = setup()
    with patch('yam.ui.recording_prompt.describe_slot', return_value='old take'):
        assert prompt.handle('4') and prompt.handle('5')
        assert not recording.requests
        assert prompt.handle('5')
    assert recording.requests[0][:2] == ('save', '5')
    assert 'aiming at recording 5 instead' in '\n'.join(lines)


def test_reaim_to_free_slot_saves_without_extra_confirmation():
    recording, prompt, _ = setup()
    with patch('yam.ui.recording_prompt.describe_slot', side_effect=['old take', None]):
        prompt.handle('4')
        prompt.handle('5')
    assert recording.requests[0][:2] == ('save', '5')


def test_reopen_drops_previous_overwrite_confirmation():
    recording, prompt, _ = setup()
    with patch('yam.ui.recording_prompt.describe_slot', return_value='old take'):
        prompt.handle('4')
        prompt.open()
        prompt.handle('4')
    assert not recording.requests


def test_busy_keys_never_queue_save_or_discard_and_q_passes_through():
    recording, prompt, lines = setup()
    recording.busy = True
    for key in ('5', 'x', 'w'):
        assert prompt.handle(key)
    assert not recording.requests and len(lines) == 3
    assert not prompt.handle('q') and not prompt.active
    assert not recording.requests


def test_q_at_slot_choice_preserves_explicit_discard_behavior():
    recording, prompt, lines = setup()
    with patch('yam.ui.recording_prompt.describe_slot', return_value='old take'):
        prompt.handle('4')
    assert prompt.handle('q')
    assert recording.requests[0][0] == 'discard'
    assert 'kept recording 4' in '\n'.join(lines)
    assert 'recording discarded.' not in '\n'.join(lines)
    recording.busy = False
    prompt.poll()
    assert not prompt.active and 'recording discarded.' in '\n'.join(lines)


def test_start_failure_requires_fresh_overwrite_confirmation():
    recording, prompt, lines = setup()
    recording.failure = OSError('cannot start worker')
    with patch('yam.ui.recording_prompt.describe_slot', return_value='old take'):
        prompt.handle('4')
        prompt.handle('4')
        assert 'cannot start worker' in '\n'.join(lines)
        recording.failure = None
        prompt.handle('4')
        assert not recording.requests
        prompt.handle('4')
    assert len(recording.requests) == 1


def test_async_failure_retains_same_take_and_requires_new_confirmation():
    recording, prompt, lines = setup()
    original = recording.pending
    with patch('yam.ui.recording_prompt.describe_slot', return_value='old take'):
        prompt.handle('4')
        prompt.handle('4')
        recording.busy = False
        recording.save_error = OSError('publication failed')
        recording.save_error.add_note('old slot restored')
        prompt.poll()
        assert prompt.active and 'old slot restored' in '\n'.join(lines)
        prompt.handle('4')
        assert len(recording.requests) == 1
        prompt.handle('4')
    assert len(recording.requests) == 2
    assert all(request[2] is original for request in recording.requests)


def test_save_success_is_reported_once_after_completion_with_frozen_summary():
    recording, prompt, lines = setup()
    with patch('yam.ui.recording_prompt.describe_slot', return_value=None):
        prompt.handle('5')
    prompt.poll()
    assert not any('✓ recording' in line for line in lines)
    recording.busy = False
    recording.pending = None
    recording.saved = SavedTake(Path('5.json'), True, 'cleanup warning')
    prompt.poll()
    prompt.poll()
    assert not prompt.active
    assert sum('✓ recording 5 saved: 1.0s, 2 samples' in line for line in lines) == 1
    assert any('cleanup warning' in line for line in lines)


def test_failed_discard_returns_to_choice_without_reporting_success():
    recording, prompt, lines = setup()
    prompt.discard()
    recording.busy = False
    recording.save_error = OSError('directory still owned')
    prompt.poll()
    assert prompt.active and recording.pending is not None
    assert not any('recording discarded.' in line for line in lines)
    prompt.handle('x')
    assert len(recording.requests) == 2


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
