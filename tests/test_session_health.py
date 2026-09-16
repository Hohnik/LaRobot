"""The live health coordinator uses the same per-arm guards as direct tests."""
import sys
from yam.lifecycle import StopCause, StopRequest
from yam.session import ArmSession
from yam.session_health import SessionHealth
from test_arm_session import FakeRobot, FakeState, stalling_arm


def health():
    lines = []
    return SessionHealth(n_arm=6, stall_torque=1, stall_velocity=.05,
                         stall_seconds=.4, emit=lines.append), lines


def test_pending_input_fault_survives_next_cycle_without_reading_temperatures():
    robot = FakeRobot()
    arm = ArmSession(robot, name='B')
    robot.motor_chain.read_states = lambda: (_ for _ in ()).throw(AssertionError('must stop first'))
    service, _ = health()
    stop = StopRequest(StopCause.FAULT, 'puck lost')
    assert service.check([arm], 0, stop=stop) is stop


def test_dead_chain_stops_before_thermal_reads():
    robot = FakeRobot()
    arm = ArmSession(robot, name='B')
    robot.motor_chain.running = False
    service, _ = health()
    stop = service.check([arm], 0)
    assert stop.cause is StopCause.FAULT and 'motor chain STOPPED' in stop.message
    assert arm.states is None


def test_read_failure_advances_blind_guard_and_keeps_last_temperature_snapshot():
    robot, arm = stalling_arm(eff=0)
    service, lines = health()
    service.check([arm], 0)
    old = list(arm.temps)
    arm.thermal.blind_cycles = 2
    robot.motor_chain._raises = True
    assert service.check([arm], .1) is None
    stop = service.check([arm], .2)
    assert 'unreadable for 2 cycles' in stop.message
    assert arm.states is None and arm.hottest is None and arm.temps == old
    assert 'CAN read failed' in arm.read_error and any('CAN read failed' in line for line in lines)


def test_stall_latches_measured_jaws_once_until_explicitly_opened():
    robot, arm = stalling_arm(jaw_pos=.31)
    service, lines = health()
    assert service.check([arm], 0) is None
    assert service.check([arm], .5) is None
    assert arm.gripper_value == .31 and arm.jaw_block == .31 and arm.stall_count == 1
    for t in (1., 2., 3.):
        service.check([arm], t)
    assert arm.stall_count == 1 and sum('GRIPPER STALLED' in line for line in lines) == 1
    assert arm.hold_jaw(.9) == .9
    service.check([arm], 4.)
    assert arm.jaw_block is None and arm.jaw_unblocked_from is None
    assert sum('free to close again' in line for line in lines) == 1


def test_no_gripper_never_indexes_a_seventh_motor():
    robot = FakeRobot(dofs=6)
    robot.motor_chain._states = [FakeState()] * 6
    arm = ArmSession(robot, name='B')
    service, _ = health()
    assert service.check([arm], 0) is None and arm.jaw_temp is None


def test_hot_one_arm_requests_session_stop_and_other_arm_is_still_observed():
    first, second = [ArmSession(FakeRobot(), name=n) for n in ('B', 'G')]
    first.robot.motor_chain._states = [FakeState(temp_mos=70)] * 7
    second.robot.motor_chain._states = [FakeState(temp_mos=35)] * 7
    service, _ = health()
    stop = service.check([first, second], 0)
    assert stop.cause is StopCause.FAULT and 'arm B:' in stop.message
    assert second.hottest == 35 and first.states is first._states


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
