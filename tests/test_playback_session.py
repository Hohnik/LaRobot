"""The actual replay owner's common clock, arrival gate and layout contracts."""
from types import SimpleNamespace
import sys
from yam.playback_session import PlaybackSession
from yam.recording import Layout, Trajectory


def prepared():
    take = Trajectory(meta={'arms': ['B', 'G'], 'joints_per_arm': 7, 'simulated': True})
    take.append(0., [0.] * 14)
    take.append(1., [.1] * 14)
    layout = Layout.from_meta(take.meta, take.n_joints)
    arms = [SimpleNamespace(name=n) for n in ('B', 'G')]
    p = PlaybackSession()
    p.prepare(take, layout, arms, '8', 1.)
    return p, take, layout, arms


def started():
    p, *_ = prepared()
    p.credit_arrival('B', 'replay')
    p.credit_arrival('G', 'replay')
    p.start(10.)
    return p


def test_pose_arrival_never_credits_new_replay_and_both_arms_must_arrive():
    p, take, _, _ = prepared()
    assert p.credit_arrival('B', 'composite') is None
    assert p.credit_arrival('G', 'replay') == ['B']
    try:
        p.start(10.)
    except RuntimeError:
        pass
    else:
        raise AssertionError('one arm started a two-arm replay')
    assert p.pending is take and p.active is None
    assert p.credit_arrival('B', 'replay') == []
    p.start(10.)
    assert p.active is take and p.pending is None and p.cursor == 0
    assert p.tracking.n_joints == 14


def test_one_cycle_advances_one_cursor_for_both_arms_and_excludes_both_grippers():
    p = started()
    measured = [0.] * 14
    measured[6] = measured[13] = 1.
    step = p.advance(measured, .02, n_arm=6, max_lag=.05)
    assert abs(p.cursor - .02) < 1e-9 and not step.held
    assert step.target[:7] == step.target[7:]
    assert p.tracking.cycles == 0
    p.observe(step, measured, 10.02, .02)
    assert p.tracking.cycles == 1 and p.held_seconds == 0
    assert p.progress_at == 10.02


def test_lag_holds_shared_clock_and_updates_hold_time_after_commands():
    p = started()
    measured = [1.] * 14
    step = p.advance(measured, .02, n_arm=6, max_lag=.05)
    assert step.held and p.cursor == 0 and p.held_seconds == 0
    p.observe(step, measured, 10.02, .02)
    assert p.held_seconds == .02 and p.progress_at == 10.
    assert p.worst_lag >= 1.


def test_scrub_neutral_holds_and_new_take_resets_scrub_choice():
    p = started()
    p.scrub = True
    step = p.advance([0.] * 14, .02, n_arm=6, max_lag=.05, deflection=0.)
    assert p.cursor == 0 and not step.finished
    take, layout, arms = p.active, p.layout, p.arms
    p.finish()
    p.prepare(take, layout, arms, '9', 1.)
    assert p.scrub is False and p.slot == '9'


def test_cancelled_or_unknown_arrivals_cannot_restart_pending_take():
    p, *_ = prepared()
    p.credit_arrival('B', 'replay')
    p.cancel_pending()
    assert p.credit_arrival('G', 'replay') is None
    assert p.credit_arrival('unknown', 'replay') is None
    try:
        p.start(10.)
    except RuntimeError:
        pass
    else:
        raise AssertionError('cancelled replay restarted')


def test_wrong_arm_order_is_rejected_without_replacing_pending_take():
    p, take, layout, arms = prepared()
    try:
        p.prepare(take, layout, list(reversed(arms)), '9', 1.)
    except ValueError:
        pass
    else:
        raise AssertionError('reversed arms accepted against recording layout')
    assert p.slot == '8' and [a.name for a in p.arms] == ['B', 'G']


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
