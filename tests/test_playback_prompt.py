"""Playback preview requests motion only after explicit confirmation."""
from types import SimpleNamespace
import sys

from yam.playback_session import PlaybackSession
from yam.recording import Layout, Trajectory
from yam.ui.playback_prompt import PlaybackChoice, PlaybackPrompt


def setup(speed=0.1, *, refuse=False):
    take = Trajectory(meta={'arms': ['B'], 'joints_per_arm': 7})
    take.append(0, [0] * 7)
    take.append(1, [speed] * 6 + [0])
    layout = Layout.from_meta(take.meta, take.n_joints)
    playback, hints, messages, loads = PlaybackSession(), [], [], []
    def load(slot):
        loads.append(slot)
        return None if refuse else (take, layout, [SimpleNamespace(name='B')])
    prompt = PlaybackPrompt(playback, load_take=load, hint=hints.append, emit=messages.append)
    return playback, prompt, hints, messages, loads


def test_selection_prepares_and_previews_but_does_not_start():
    playback, prompt, hints, _, loads = setup()
    assert prompt.handle('5', 1) is PlaybackChoice.STAY
    assert loads == ['5'] and playback.pending is not None and playback.active is None
    assert 'PLAY 5' in hints[-1] and 'Enter=go' in hints[-1]
    try:
        playback.start(0)
    except RuntimeError:
        pass
    else:
        raise AssertionError('preview bypassed the arrival gate')


def test_speed_keys_stay_in_preview_and_respect_both_bounds():
    playback, prompt, _, _, _ = setup(speed=2)
    prompt.handle('5', 1)
    for _ in range(50):
        assert prompt.handle('+', 1) is PlaybackChoice.STAY
    assert playback.speed == 1
    for _ in range(50):
        assert prompt.handle('-', 1) is PlaybackChoice.STAY
    assert playback.speed == .05 and playback.active is None
    assert '1.00x will lag' in prompt.plan(1)


def test_enter_requests_ordinary_playback_without_starting_it():
    playback, prompt, _, _, _ = setup()
    prompt.handle('5', 1)
    assert prompt.handle('\n', 1) is PlaybackChoice.START
    assert not playback.scrub and playback.pending is not None and playback.active is None


def test_j_requests_scrub_through_the_same_start_action():
    playback, prompt, _, _, _ = setup()
    prompt.handle('5', 1)
    assert prompt.handle('j', 1) is PlaybackChoice.START
    assert playback.scrub and playback.pending is not None and playback.active is None


def test_q_at_preview_cancels_pending_and_consumes_the_key():
    playback, prompt, hints, messages, _ = setup()
    prompt.handle('5', 1)
    assert prompt.handle('q', 1) is PlaybackChoice.CLOSE
    assert playback.pending is None and hints[-1] == '' and 'cancelled' in messages[-1]


def test_refused_or_cancelled_selection_never_prepares_motion():
    playback, prompt, hints, messages, loads = setup(refuse=True)
    assert prompt.handle('5', 1) is PlaybackChoice.CLOSE
    assert playback.pending is None and not hints
    prompt.open()
    assert prompt.handle('q', 1) is PlaybackChoice.CLOSE
    assert loads == ['5'] and 'cancelled' in messages[-1]


def test_reopening_requires_a_new_slot_instead_of_confirming_the_old_one():
    playback, prompt, _, messages, loads = setup()
    prompt.handle('5', 1)
    prompt.open()
    assert prompt.handle('\n', 1) is PlaybackChoice.CLOSE
    assert loads == ['5'] and playback.active is None and 'cancelled' in messages[-1]


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
