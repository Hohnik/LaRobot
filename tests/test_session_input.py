"""Input routing, failure retention, edges and button learning without devices."""
from types import SimpleNamespace
import sys
from yam.inputs.axis_map import AxisMap
from yam.lifecycle import StopCause, StopRequest
from yam.session import ArmSelector
from yam.session_input import SessionInput


def reader(*, failure=False):
    value = SimpleNamespace(buttons=0, calls=0)
    def read():
        value.calls += 1
        if failure:
            raise OSError('gone')
        return [1., 0, 0, 0, 0, 0]
    value.read = read
    return value


def arm(name, puck=None):
    return SimpleNamespace(name=name, reader=puck or reader(), raw_axes=[], buttons_prev=0,
                           learn_button=None, axis_map=AxisMap(button_open=1, button_close=2),
                           mode='hold', gripper_value=.5, last_input_kind=None,
                           robot=SimpleNamespace(num_dofs=lambda: 7))


def setup(shared=None):
    selection = ArmSelector(['B', 'G'])
    lines = []
    service = SessionInput(shared_reader=shared, selection=selection, n_arm=6,
                           button_rate=.6, clamp=lambda v: min(.98, max(.02, v)), emit=lines.append)
    return service, selection, lines


def test_shared_puck_is_drained_once_and_follows_selection():
    shared = reader()
    service, selection, _ = setup(shared)
    arms = [arm('B'), arm('G')]
    service.poll(arms, .01, prompt_open=False)
    assert shared.calls == 1 and arms[0].raw_axes[0] == 1 and arms[1].raw_axes == [0.] * 6
    selection.cycle()
    service.poll(arms, .01, prompt_open=False)
    assert shared.calls == 2 and arms[1].raw_axes[0] == 1 and arms[0].raw_axes == [0.] * 6
    assert all(a.reader.calls == 0 for a in arms)


def test_failed_shared_reader_centers_both_and_requests_fault():
    service, _, _ = setup(reader(failure=True))
    arms = [arm('B'), arm('G')]
    stop = service.poll(arms, .01, prompt_open=False)
    assert stop.cause is StopCause.FAULT and 'shared SpaceMouse' in stop.message
    assert all(a.raw_axes == [0.] * 6 for a in arms)


def test_per_arm_failure_keeps_first_stop_and_still_reads_other_input():
    service, _, _ = setup()
    arms = [arm('B', reader(failure=True)), arm('G')]
    stop = StopRequest(StopCause.FAULT, 'existing fault')
    assert service.poll(arms, .01, prompt_open=False, stop=stop) is stop
    assert arms[0].raw_axes == [0.] * 6 and arms[1].reader.calls == 1


def test_button_learning_requires_edges_then_the_other_button():
    service, _, _ = setup()
    one = arm('B')
    one.axis_map = AxisMap()
    one.learn_button = 'open'
    one.reader.buttons = 1
    service.poll([one], .01, prompt_open=False)
    assert one.learn_button == 'close' and one.axis_map.button_open == 1
    service.poll([one], .01, prompt_open=False)
    assert one.learn_button == 'close'
    one.reader.buttons = 2
    service.poll([one], .01, prompt_open=False)
    assert one.learn_button is None and one.axis_map.button_close == 2


def test_another_prompt_cancels_learning():
    service, _, lines = setup()
    one = arm('B')
    one.learn_button = 'open'
    service.poll([one], .01, prompt_open=True)
    assert one.learn_button is None and 'CANCELLED' in lines[0]


def test_hold_buttons_move_only_in_drive_modes_and_are_clamped():
    service, _, _ = setup()
    one = arm('B')
    one.reader.buttons = 1
    service.poll([one], 1, prompt_open=False)
    assert one.gripper_value == .5
    one.mode = 'teleop'
    service.poll([one], 1, prompt_open=False)
    assert one.gripper_value == .98
    one.reader.buttons = 2
    service.poll([one], 2, prompt_open=False)
    assert one.gripper_value == .02


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
