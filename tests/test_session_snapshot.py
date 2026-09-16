"""Detached display values, read counts and both real operator display paths."""
from dataclasses import FrozenInstanceError
import sys
from unittest.mock import patch
from types import SimpleNamespace

from yam.session_snapshot import capture_status, ArmStatus
from yam.ui.session_status import status_row
from test_status_row import arm_in
from test_session_recording_failures import app, run_session


def test_snapshot_is_detached_and_rendering_never_reads_again():
    arm = arm_in('teleop')
    with patch.object(arm.robot, 'get_joint_pos', wraps=arm.robot.get_joint_pos) as joints, \
         patch.object(arm.teleop, 'ee_position', wraps=arm.teleop.ee_position) as ee, \
         patch.object(arm.teleop, 'lead', wraps=arm.teleop.lead) as lead:
        snapshot, = capture_status([arm], .6, 0)
        expected = status_row(snapshot, '')
        arm.robot.q[:] = 9
        arm.teleop._ee[:] = 9
        arm.teleop.speed_scale = .1
        arm.hottest = None
        arm.mode = 'hold'
        assert status_row(snapshot, '') == expected
        assert joints.call_count == ee.call_count == lead.call_count == 1
    try:
        snapshot.mode = 'guide'
    except FrozenInstanceError:
        pass
    else:
        raise AssertionError('snapshot is mutable')


def test_guide_reference_is_copied():
    import numpy as np
    arm = arm_in('guide')
    arm.guide_ref = np.zeros(7)
    snapshot, = capture_status([arm], .6, 0)
    arm.guide_ref[:] = 8
    assert snapshot.guide_ref == (0.,) * 7


def test_mirror_diagnostic_uses_the_displayed_poses_without_extra_reads():
    leader, follower = arm_in('hold', name='B'), arm_in('mirror', name='G')
    poses = []
    def status(q_leader, q_follower):
        poses.append((q_leader, q_follower))
        return 'FOLLOWING test'
    mirror = SimpleNamespace(link=SimpleNamespace(status=status), leader=leader)
    with patch.object(leader.robot, 'get_joint_pos', wraps=leader.robot.get_joint_pos) as b, \
         patch.object(follower.robot, 'get_joint_pos', wraps=follower.robot.get_joint_pos) as g:
        rows = capture_status([leader, follower], .6, 0, mirror)
    assert b.call_count == g.call_count == 1
    assert poses == [(rows[0].q, rows[1].q)]
    assert rows[0].note == '' and 'FOLLOWING test' in status_row(rows[1], '')


def test_settings_and_heartbeat_in_the_real_app_render_snapshots():
    leads, captures = [], []
    real = app.capture_status
    def capture(*args):
        rows = real(*args)
        captures.append(rows)
        return rows
    def render(snapshot, lead):
        assert isinstance(snapshot, ArmStatus)
        leads.append(lead)
        return status_row(snapshot, lead)
    # Real wall time crosses the 1 s heartbeat; all device handles are simulated.
    with patch.object(app, 'capture_status', capture), patch.object(app, 'status_row', render):
        code, text, _, _, _ = run_session({1: 'n', 2: '+', 3: 'n', 125: 'q'}, max_cycles=130)
    assert code == 0, text
    assert len(captures) >= 2 and all(len(rows) == 2 for rows in captures)
    assert '' in leads and any('t=' in lead for lead in leads), leads


def test_status_read_failure_retains_the_existing_app_fault_cleanup():
    stopped = []
    def stop(robot):
        stopped.append(robot)
        return list(range(1, 8))
    with patch.object(app, 'capture_status', side_effect=OSError('status read failed')), \
         patch.object(app, 'shutdown_robot', stop):
        code, text, _, arms, _ = run_session({1: 'n', 2: '+'})
    assert code != 0 and 'status read failed' in text, text
    assert len(stopped) == len(arms) == 2


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
