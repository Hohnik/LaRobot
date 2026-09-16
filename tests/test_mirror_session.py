"""Mirror confirmation and pair ownership without actual robot commands."""
from types import SimpleNamespace
import sys
from yam.mirror_session import MirrorSession


def setup():
    events, lines, hints = [], [], []
    arms = []
    for name in ('B', 'G'):
        one = SimpleNamespace(name=name, mode='hold', robot=SimpleNamespace(
            get_joint_pos=lambda: [0.] * 7, max_speed=2., limited_cycles=11))
        def hold(one=one):
            events.append(('hold', one.name))
            one.mode = 'hold'
        one.enter_hold = hold
        arms.append(one)
    mirror = MirrorSession(align_speed=.3, n_arm=6, emit=lines.append, hint=hints.append)
    return mirror, arms, events, lines, hints


def confirm(mirror, key, mode='copy'):
    return mirror.confirm(key, mode=mode, catchup=0., max_gap=.35, max_speed=1.)


def test_preview_and_toggle_do_not_engage_or_change_arm_modes():
    mirror, arms, events, lines, _ = setup()
    assert mirror.preview(arms, ['B'], 'copy')
    assert confirm(mirror, 'i') == (True, 'mirror')
    assert not events and mirror.link is None and all(a.mode == 'hold' for a in arms)
    assert any('COLLIDING' in line for line in lines)


def test_confirm_holds_only_follower_and_reads_its_actual_speed_cap():
    mirror, arms, events, _, _ = setup()
    mirror.preview(arms, ['G'], 'copy')
    assert confirm(mirror, '\n') == (False, 'copy')
    assert events == [('hold', 'B')] and arms[0].mode == 'mirror'
    assert mirror.link.follow_speed == 2. and mirror.clipped_at == 11


def test_cancel_clears_pending_pair_without_changing_modes():
    mirror, arms, events, _, hints = setup()
    mirror.preview(arms, ['B'], 'copy')
    assert confirm(mirror, 'q') == (False, 'copy')
    assert mirror.leader is mirror.follower is mirror.link is None
    assert not events and hints[-1] == ''


def test_ambiguous_selection_refuses_before_motion():
    mirror, arms, events, _, hints = setup()
    assert not mirror.preview(arms, ['B', 'G'], 'copy')
    assert not events and hints and mirror.link is None


def test_turning_off_holds_follower_and_clears_link():
    mirror, arms, events, _, _ = setup()
    mirror.preview(arms, ['B'], 'copy')
    confirm(mirror, '\n')
    mirror.turn_off(arms)
    assert events == [('hold', 'G'), ('hold', 'G')]
    assert mirror.link is None and all(a.mode == 'hold' for a in arms)


def test_leaving_mode_ends_link_without_another_hold_command():
    mirror, arms, events, _, _ = setup()
    mirror.preview(arms, ['B'], 'copy')
    confirm(mirror, '\n')
    arms[1].mode = 'hold'
    mirror.observe_modes(arms)
    assert mirror.link is None and events == [('hold', 'G')]


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
