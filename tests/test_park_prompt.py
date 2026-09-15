"""Prompt transitions preserve confirmation and cannot leak a take prefix."""
import sys
from yam.ui.park_prompt import ParkAction, ParkPrompt


def test_empty_and_single_pose_run_on_first_accept():
    p = ParkPrompt()
    choice = p.handle('\n')
    assert choice.action is ParkAction.RUN and choice.entries == ('0',)
    p.open(); p.handle('3'); choice = p.handle('p')
    assert choice.entries == ('3',) and not choice.confirmed


def test_multiple_legs_require_second_accept():
    for key in ('\n', '\r', ' ', 'p'):
        p = ParkPrompt(); p.handle('1'); p.handle('2')
        assert p.handle(key).action is ParkAction.CONFIRM and p.confirming
        choice = p.handle(key)
        assert choice.action is ParkAction.RUN and choice.entries == ('1', '2')
        assert choice.confirmed and not p.confirming


def test_take_prefix_applies_to_only_the_next_digit():
    p = ParkPrompt()
    assert p.handle('w').action is ParkAction.TAKE_DIGIT
    p.handle('3'); p.handle('4')
    assert p.entries == ('w3', '4') and p.shown == '▶3 → 4'


def test_cancel_clears_pending_take_prefix():
    p = ParkPrompt(); p.handle('w')
    assert p.handle('x').action is ParkAction.CANCEL
    p.open(); p.handle('1')
    assert p.handle('\n').entries == ('1',)


def test_reopening_resets_confirmation_and_take_prefix():
    p = ParkPrompt(); p.handle('1'); p.handle('2'); p.handle('\n')
    p.open()
    assert p.handle('\n').entries == ('0',)
    p.handle('w'); p.open(); p.handle('3')
    assert p.entries == ('3',)


def test_nonaccept_at_confirmation_cancels_instead_of_extending_sequence():
    p = ParkPrompt(); p.handle('1'); p.handle('2'); p.handle('\n')
    choice = p.handle('3')
    assert choice.action is ParkAction.CANCEL and choice.confirmed
    assert p.entries == () and not p.confirming


def test_completed_choice_is_an_immutable_snapshot():
    p = ParkPrompt(); p.handle('1'); choice = p.handle('\n')
    p.handle('2')
    assert choice.entries == ('1',) and p.entries == ('2',)


def test_accept_with_unused_take_prefix_preserves_existing_base_behavior_and_resets():
    p = ParkPrompt(); p.handle('w')
    assert p.handle('\n').entries == ('0',)
    p.handle('2')
    assert p.entries == ('2',)


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
