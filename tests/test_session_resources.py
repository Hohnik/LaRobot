"""Verify cleanup ordering without creating threads or acquiring devices."""
from types import SimpleNamespace
import sys

from yam.session_resources import SessionResources


def setup(*, fail=(), confirmations=None):
    events, messages = [], []
    def action(name, result=None):
        def run(*args):
            events.append(name)
            if name in fail:
                raise OSError(name + ' failed')
            return result
        return run
    def shutdown(robot):
        return action(robot, (confirmations or {}).get(robot, list(range(1, 8))))()
    resources = SessionResources(SimpleNamespace(shutdown=action('recording')),
                                 shutdown_robot=shutdown, emit=messages.append)
    resources.own_robot('B', 'robot_B', motors=7)
    resources.own_robot('G', 'robot_G', motors=6)
    resources.own_puck(SimpleNamespace(close=action('puck_B')))
    resources.own_puck(SimpleNamespace(close=action('puck_G')))
    resources.own_capture(SimpleNamespace(stop=action('camera')))
    return resources, events, messages


def test_all_motors_precede_inputs_writers_and_readers():
    resources, events, _ = setup()
    assert resources.robot_names == ('B', 'G')
    assert resources.close()
    assert events == ['robot_B', 'robot_G', 'puck_B', 'puck_G', 'recording', 'camera']


def test_failures_do_not_skip_later_resources():
    resources, events, messages = setup(fail=('robot_B', 'puck_B', 'recording'))
    assert not resources.close()
    assert events == ['robot_B', 'robot_G', 'puck_B', 'puck_G', 'recording', 'camera']
    assert 'could not clean up recording' in '\n'.join(messages)


def test_expected_motor_ids_belong_to_each_acquired_handle():
    resources, _, messages = setup(confirmations={'robot_B': [1, 2, 3, 4, 5, 6],
                                                  'robot_G': [1, 2, 3, 4, 5, 6]})
    assert not resources.close()
    assert any('arm B: could not confirm motors [7]' in line for line in messages)
    assert not any('arm G: could not confirm' in line for line in messages)


def test_completed_cleanup_is_not_repeated_and_keeps_failure_result():
    resources, events, _ = setup(fail=('recording',))
    assert not resources.close()
    first = list(events)
    assert not resources.close() and events == first


def test_capture_closes_when_no_robot_was_acquired():
    events = []
    resources = SessionResources(SimpleNamespace(shutdown=lambda: events.append('recording')),
                                 shutdown_robot=lambda robot: (_ for _ in ()).throw(AssertionError()))
    resources.own_capture(SimpleNamespace(stop=lambda: events.append('camera')))
    assert resources.close() and resources.robot_names == ()
    assert events == ['recording', 'camera']


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
