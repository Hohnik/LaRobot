"""Composite ownership: validation, all-arm arrival gates and cancellation."""
from types import SimpleNamespace
import sys
from yam.composite import CompositeRun


def rig(missing_take=None):
    events, messages = [], []
    arms = [SimpleNamespace(name=n, base_pose=[0.] * 7,
                            slots={'1': [.1] * 7, '2': [.2] * 7}) for n in ('B', 'G')]
    def load(slot):
        events.append(('load', slot))
        return None if slot == missing_take else (f'take{slot}', 'layout', arms)
    def park(arm, legs, label, *, for_composite):
        assert for_composite
        events.append(('park', arm.name, [n for n, _ in legs]))
    def take(slot, loaded, layout, ordered):
        events.append(('take', slot, [a.name for a in ordered]))
    run = CompositeRun(load_take=load, begin_path=park, start_take=take,
                       emit=messages.append, clear_hint=lambda: events.append(('clear',)))
    return run, arms, events, messages


def test_every_take_is_validated_before_first_motion():
    run, arms, events, messages = rig(missing_take='9')
    assert not run.begin(['1', 'w8', '2', 'w9'], arms)
    assert events == [('load', '8'), ('load', '9')]
    assert not run.active and any('nothing has moved' in m for m in messages)


def test_both_pose_arms_must_arrive_before_take_and_take_must_finish_before_next_pose():
    run, arms, events, messages = rig()
    assert run.begin(['1', '2', 'w8', '0'], arms)
    assert events == [('load', '8'), ('park', 'B', ['1', '2']), ('park', 'G', ['1', '2'])]
    run.arrived('B')
    run.arrived('B')
    run.arrived('unknown')
    assert not any(e[0] == 'take' for e in events)
    run.arrived('G')
    assert events[-1] == ('take', '8', ['B', 'G'])
    run.arrived('G')  # The old arrival must not finish the newly armed take leg.
    assert events[-1][0] == 'take'
    run.leg_done()
    assert events[-2:] == [('park', 'B', ['0']), ('park', 'G', ['0'])]
    run.arrived('G')
    assert run.active
    run.arrived('B')
    assert not run.active and events[-1] == ('clear',)
    assert sum('COMPOSITE RUN complete' in m for m in messages) == 1


def test_abandon_drops_queue_and_late_arrivals_cannot_restart_it():
    run, arms, events, messages = rig()
    run.begin(['1', 'w8'], arms)
    run.abandon('mode changed')
    stopped = events[:]
    run.arrived('B')
    run.arrived('G')
    run.leg_done()
    run.abandon('again')
    assert events == stopped and not run.active
    assert sum('abandoned' in m for m in messages) == 1


def test_empty_pose_group_skips_without_substituting_a_base_pose():
    run, arms, events, messages = rig()
    run.begin(['9', 'w8'], arms)
    assert events == [('load', '8'), ('take', '8', ['B', 'G'])]
    assert sum('skipping empty' in m for m in messages) == 2


def test_selection_is_copied_and_take_keeps_recording_arm_order():
    run, arms, events, _ = rig()
    selection = [arms[1]]
    run.begin(['1', 'w8', '2'], selection)
    selection[:] = [arms[0]]
    run.arrived('G')
    assert events[-1] == ('take', '8', ['B', 'G'])
    run.leg_done()
    assert events[-1] == ('park', 'G', ['2'])


def test_replacing_a_run_drops_old_pose_arrival_credits():
    run, arms, events, _ = rig()
    run.begin(['1', 'w8'], arms)
    run.begin(['w7', '2'], arms)
    after_start = events[:]
    run.arrived('B')
    run.arrived('G')
    assert events == after_start
    run.leg_done()
    assert events[-2:] == [('park', 'B', ['2']), ('park', 'G', ['2'])]


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
