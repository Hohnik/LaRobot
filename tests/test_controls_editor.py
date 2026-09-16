"""Exercise production mapping edits without a robot or input device."""
from types import SimpleNamespace
import sys

from yam.inputs.axis_map import AxisMap
from yam.ui.controls_editor import edit_controls


def setup(active=0):
    initial = AxisMap()
    arm = SimpleNamespace(axis_map=initial.copy(), axis_map_at_start=initial,
                          last_active_axis=active, last_active_value=-1, frame='world')
    messages = []
    def edit(key):
        return edit_controls(arm, key, emit=lambda text='': messages.append(text))
    return arm, edit, messages


def test_reverse_targets_the_robot_motion_driven_by_the_observed_axis():
    arm, edit, _ = setup(active=1)
    arm.axis_map.swap(0, 1)
    assert edit('f')
    assert arm.axis_map.sign == [-1, 1, 1, 1, 1, 1]


def test_swap_preserves_both_bindings_and_previous_motion_reverses_it():
    arm, edit, messages = setup()
    assert edit('2') and arm.axis_map.source == [1, 0, 2, 3, 4, 5]
    assert 'press 1 to swap back' in messages[-1]
    assert edit('2') and arm.axis_map.source == [1, 0, 2, 3, 4, 5]
    assert 'unchanged' in messages[-1]
    assert edit('1') and arm.axis_map == arm.axis_map_at_start


def test_unbind_then_bind_uses_observed_direction():
    arm, edit, messages = setup()
    assert edit('u') and arm.axis_map.motion_driven_by(0) is None
    assert edit('f') and 'no direction to reverse' in messages[-1]
    assert edit('2') and arm.axis_map.motion_driven_by(0) == 1
    assert arm.axis_map.sign[1] == -1


def test_revert_copies_initial_map_and_later_edits_do_not_change_snapshot():
    arm, edit, _ = setup()
    edit('f')
    edit('0')
    assert arm.axis_map == arm.axis_map_at_start and arm.axis_map is not arm.axis_map_at_start
    edit('f')
    assert arm.axis_map_at_start.sign == [1] * 6


def test_missing_axis_cannot_change_bindings():
    arm, edit, messages = setup(active=None)
    for key in ('f', '2', 'u'):
        assert edit(key)
    assert arm.axis_map == arm.axis_map_at_start and len(messages) == 3


def test_modes_speed_and_unknown_keys_are_left_to_operator():
    arm, edit, messages = setup()
    for key in ('q', 't', 'g', 'h', 'm', '+', '-', '.', ',', 'r', '?', 'x', '\n'):
        assert not edit(key)
    assert not messages and arm.axis_map == arm.axis_map_at_start


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
