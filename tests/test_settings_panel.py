"""Exercise live-settings keys with callbacks that record changes and persistence."""
from pathlib import Path
import sys
from yam.settings import LIVE_ORDER, LADDERS, adjust
from yam.ui.settings_panel import SettingsAction, SettingsPanel


def panel():
    values = {name: LADDERS[name][1] for name in LIVE_ORDER}
    events, output = [], []
    def apply(name, value):
        values[name] = value
        events.append(('apply', name, value))
    p = SettingsPanel(values=lambda: values, apply=apply,
        save=lambda: events.append(('save', dict(values))),
        show_status=lambda: events.append(('status',)), emit=output.append,
        builtin=values, settings_file=Path('/temporary/config/session_defaults.json'),
        mode_keys={'t', 'g', 'h'})
    return p, values, events, output


def test_navigation_wraps_without_changing_values():
    p, values, events, _ = panel()
    before = dict(values)
    p.handle('\x1b[A')
    assert p.selected == LIVE_ORDER[-1]
    p.handle('\x1b[B')
    assert p.selected == LIVE_ORDER[0]
    p.handle('3')
    assert p.selected == LIVE_ORDER[2] and values == before and not events


def test_change_applies_one_selected_value_before_rendering_status():
    p, values, events, _ = panel()
    p.handle('3'); before = values['max_lag']
    assert p.handle('+') is SettingsAction.STAY
    assert events == [('apply', 'max_lag', adjust('max_lag', before, True)), ('status',)]


def test_revert_uses_copied_session_values_even_after_external_changes():
    p, values, events, _ = panel()
    original = dict(values)
    values['max_speed'] = 10.
    p.handle('0')
    assert values == original and len(events) == len(LIVE_ORDER)
    assert all(e[0] == 'apply' for e in events)


def test_mode_keys_close_without_applying_a_mode_or_saving():
    for key in ('t', 'g', 'h', 'n', '\n', '\r', ' '):
        p, _, events, _ = panel()
        assert p.handle(key) is SettingsAction.CLOSE and not events


def test_only_explicit_save_persists_current_values():
    p, values, events, _ = panel()
    p.handle('+'); events.clear()
    assert p.handle('s') is SettingsAction.CLOSE
    assert events == [('save', values)]


def test_failed_save_never_announces_success():
    p, _, _, output = panel()
    def fail():
        raise OSError('disk unavailable')
    p._save = fail
    try:
        p.handle('s')
    except OSError:
        assert not any('SAVED' in line for line in output)
        return
    raise AssertionError('a failed save must reach the caller')


def test_quit_is_a_separate_intent_without_implicit_save():
    p, _, events, _ = panel()
    assert p.handle('q') is SettingsAction.QUIT and not events


def test_help_and_unknown_arrow_preserve_selection_and_values():
    p, _, events, output = panel()
    p.handle('?'); p.handle('\x1b[C')
    assert p.selected == LIVE_ORDER[0] and not events
    assert any('right arrow does nothing' in line for line in output)


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
