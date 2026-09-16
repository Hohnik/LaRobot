"""Rate-key precedence and the actual normal/mapping operator callers."""
from types import SimpleNamespace
from unittest.mock import patch
import sys

from yam.ui.drive_controls import DriveControls
from test_session_recording_failures import app, run_session


def setup():
    values = SimpleNamespace(linear_scale=.1, scrub_max=1., teleop_speed=1.5)
    arms = [SimpleNamespace(mode='park', park_speed=.4),
            SimpleNamespace(mode='hold', park_speed=.2)]
    hints, messages = [], []
    controls = DriveControls(rotation=True, angular_scale=.6,
                             hint=hints.append, emit=messages.append)
    return controls, values, arms, hints, messages


def test_scrub_precedes_park_and_mapping_precedes_both():
    drive, values, arms, hints, _ = setup()
    assert drive.handle('+', values=values, aimed=arms, scrub=True)
    assert values.scrub_max > 1 and [a.park_speed for a in arms] == [.4, .2]
    assert values.linear_scale == .1 and 'scrub pace' in hints[-1]
    before = values.scrub_max
    drive.handle('-', values=values, aimed=arms, scrub=True, mapping=True)
    assert values.linear_scale < .1 and values.scrub_max == before
    assert [a.park_speed for a in arms] == [.4, .2]


def test_park_updates_all_selected_with_existing_caps():
    drive, values, arms, _, _ = setup()
    drive.handle('=', values=values, aimed=arms)
    assert [a.park_speed for a in arms] == [.5, .25]
    for _ in range(30):
        drive.handle('+', values=values, aimed=arms)
    assert all(a.park_speed == 1.5 for a in arms)
    for _ in range(40):
        drive.handle('-', values=values, aimed=arms)
    assert all(a.park_speed == .05 for a in arms)


def test_rotation_state_and_limits_are_shared_across_modes():
    drive, values, arms, hints, messages = setup()
    drive.handle('r', values=values, aimed=arms, mapping=True)
    assert not drive.rotation and 'will not move' in messages[-1]
    drive.handle('r', values=values, aimed=arms)
    assert drive.rotation and 'ON' in hints[-1]
    for _ in range(50):
        drive.handle('.', values=values, aimed=arms, mapping=True)
    assert drive.angular_scale == 12.
    for _ in range(80):
        drive.handle(',', values=values, aimed=arms)
    assert drive.angular_scale == .02


def test_unhandled_keys_leave_values_untouched():
    drive, values, arms, hints, messages = setup()
    for key in ('', 't', 'm', 'q', '1', 'x', '\n'):
        assert not drive.handle(key, values=values, aimed=arms)
    assert vars(values) == dict(linear_scale=.1, scrub_max=1., teleop_speed=1.5)
    assert not hints and not messages and drive.rotation


def test_real_operator_uses_one_owner_and_live_settings_in_both_modes():
    owners, calls = [], []
    class ObservedControls(DriveControls):
        def __init__(self, **kwargs):
            super().__init__(**kwargs)
            owners.append(self)
        def handle(self, key, **kwargs):
            handled = super().handle(key, **kwargs)
            if handled:
                calls.append((key, kwargs.get('mapping', False), self.angular_scale,
                              self.rotation, kwargs['values'].linear_scale))
            return handled
    # Normal controls, enter mapping, adjust, leave mapping, adjust again.
    with patch.object(app, 'DriveControls', ObservedControls):
        code, text, _, _, _ = run_session({1: '.r+', 2: 'm', 3: ',r-', 4: 'h',
                                          5: '.', 6: 'q'},
                                         extra_args=('--linear-scale', '.12'))
    assert code == 0, text
    assert len(owners) == 1
    assert [(key, mapping) for key, mapping, *_ in calls] == [
        ('.', False), ('r', False), ('+', False),
        (',', True), ('r', True), ('-', True), ('.', False)], calls
    assert calls[2][-1] > .12 and abs(calls[5][-1] - .12) < 1e-9
    assert calls[0][2] == calls[-1][2] and owners[0].rotation


def test_real_twist_uses_the_changed_rotation_gain_and_switch():
    import numpy as np
    samples = {}
    current = [0]
    real_step = app.CartesianTeleop.step
    def step(solver, twist, dt):
        samples[current[0]] = float(np.linalg.norm(twist[3:]))
        return real_step(solver, twist, dt)
    def cycle(number, arms, root):
        current[0] = number
        if number == 1:
            arms[0].reader = SimpleNamespace(
                read=lambda: [0., 0., 0., 1., 0., 0.], buttons=0)
    with patch.object(app.CartesianTeleop, 'step', step):
        code, text, _, _, _ = run_session(
            {1: 't', 3: 'r', 5: '.r', 7: 'q'}, on_cycle=cycle)
    assert code == 0, text
    assert samples[1] > 0 and samples[3] == samples[4] == 0, samples
    assert abs(samples[5] / samples[1] - 1.25) < 1e-9, samples


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
