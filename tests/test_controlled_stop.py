"""Execute the production quit/park flow with fake arms and scripted keys."""
from types import SimpleNamespace
import sys

from yam.lifecycle import StopCause, StopRequest, controlled_stop


class Arm:
    def __init__(self, name, *, alive=True, base=True, guide=True):
        self.name, self.live, self.can_guide = name, alive, guide
        self.base_pose = [0.] * 7 if base else None
        self.mode = 'teleop'
        self.events = []

    def alive(self):
        return self.live

    def enter_hold(self):
        self.mode = 'hold'
        self.events.append('hold')

    def enter_guide(self):
        self.mode = 'guide' if self.can_guide else 'hold'
        self.events.append('guide')
        return None if self.can_guide else 'GUIDE unavailable'


def exercise(cause, *, message='a stop', arms=None, keys=(), outcomes=('arrived',), on_sleep=None):
    arms = arms if arms is not None else [Arm('B'), Arm('G')]
    events, output = [], []
    answers, results = iter(keys), iter(outcomes)
    def get():
        key = next(answers)  # Unexpected menu entry must fail, never hang.
        events.append(('key', key))
        return key
    def park(live):
        events.append(('park', [a.name for a in live]))
        assert all(a.mode == 'hold' for a in live)
        result = next(results)
        if isinstance(result, BaseException):
            raise result
        return result
    def sleep(_):
        if on_sleep:
            on_sleep(arms)
    request = StopRequest(cause, message)
    controlled_stop(request, arms, SimpleNamespace(get=get), park, emit=output.append, sleep=sleep)
    return request, events, output, arms


def test_ctrl_c_parks():
    request, events, _, _ = exercise(StopCause.INTERRUPT)
    assert events == [('park', ['B', 'G'])] and request.exit_code == 130


def test_a_crash_parks_which_is_the_whole_point():
    for message in ('the motor chain STOPPED', 'a motor is too hot', 'IK failed'):
        request, events, _, _ = exercise(StopCause.FAULT, message=message)
        assert events == [('park', ['B', 'G'])] and request.exit_code == 1


def test_a_thermal_stop_parks_too_and_that_is_deliberate():
    _, events, _, _ = exercise(StopCause.FAULT, message='hottest motor 66°C — stopping')
    assert events == [('park', ['B', 'G'])]


def test_a_planned_q_does_NOT_auto_park():
    request, events, _, _ = exercise(StopCause.QUIT, keys='d')
    assert events == [('key', 'd')] and request.exit_code == 0


def test_display_text_cannot_change_the_stopping_policy():
    _, fault, _, _ = exercise(StopCause.FAULT, message='quit requested by failed sensor')
    _, quit_events, _, _ = exercise(StopCause.QUIT, message='operator is finished', keys='d')
    assert fault == [('park', ['B', 'G'])] and quit_events == [('key', 'd')]


def test_a_dead_chain_cannot_be_parked():
    dead = Arm('B', alive=False)
    _, events, _, _ = exercise(StopCause.FAULT, arms=[dead, Arm('G')])
    assert events == [('park', ['G'])] and dead.events == []


def test_failed_auto_park_returns_to_menu_and_retries_before_exit():
    _, events, output, arms = exercise(StopCause.FAULT, keys='q', outcomes=('stalled', 'arrived'))
    assert events == [('park', ['B', 'G']), ('key', 'q'), ('park', ['B', 'G'])]
    assert all(a.events == ['hold', 'hold'] for a in arms)
    assert any('nothing is being released' in line for line in output)


def test_failed_menu_park_requires_another_choice():
    _, events, _, arms = exercise(StopCause.QUIT, keys='qd', outcomes=('stopped',))
    assert events == [('key', 'q'), ('park', ['B', 'G']), ('key', 'd')]
    assert all(a.events == ['hold', 'hold'] for a in arms)


def test_menu_park_holds_again_instead_of_exiting():
    _, events, _, arms = exercise(StopCause.QUIT, keys='pd')
    assert events == [('key', 'p'), ('park', ['B', 'G']), ('key', 'd')]
    assert all(a.events == ['hold', 'hold'] for a in arms)


def test_guide_banner_only_names_arms_that_entered_guide():
    _, _, output, arms = exercise(StopCause.QUIT, keys='gd', arms=[Arm('B'), Arm('G', guide=False)])
    assert arms[0].mode == 'guide' and arms[1].mode == 'hold'
    assert any('weightless: B —' in line for line in output)
    assert not any('weightless: B+G' in line for line in output)


def test_no_live_arms_neither_parks_nor_waits_for_keys():
    _, events, output, _ = exercise(StopCause.FAULT, arms=[Arm('B', alive=False)])
    assert not events and any('every chain is already dead' in line for line in output)


def test_no_base_pose_requires_a_choice_and_liveness_loss_exits_menu():
    _, events, output, _ = exercise(StopCause.FAULT, arms=[Arm('B', base=False)],
        keys=[None], on_sleep=lambda arms: setattr(arms[0], 'live', False))
    assert events == [('key', None)] and any('every chain died while waiting' in line for line in output)


def test_second_interrupt_propagates_to_outer_cleanup():
    try:
        exercise(StopCause.INTERRUPT, outcomes=(KeyboardInterrupt(),))
    except KeyboardInterrupt:
        return
    raise AssertionError('the interrupt must reach the device cleanup boundary')


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
