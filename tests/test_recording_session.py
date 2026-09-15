"""Verify the shared recording lifecycle used by the real operator loop."""
import sys
from yam.recording_session import RecordingSession


def started(simulated=True):
    r = RecordingSession()
    r.start(10.0, meta={'arms': ['B', 'G'], 'joints_per_arm': 7,
                        'simulated': simulated}, modes=['B:hold', 'G:hold'])
    return r


def test_two_arms_share_one_sample_and_clock():
    r = started()
    joints = list(range(14))
    r.sample(10.1, joints, ['B:guide', 'G:mirror'])
    take = r.freeze()
    assert len(take) == 1
    assert abs(take.samples[0].t - .1) < 1e-12
    assert list(take.samples[0].q) == joints
    assert take.meta['arms'] == ['B', 'G']


def test_frozen_take_stops_growing_while_waiting_for_slot():
    r = started()
    r.sample(10.0, [0.] * 14, ['B:hold', 'G:hold'])
    take = r.freeze()
    r.sample(12.0, [1.] * 14, ['B:guide', 'G:guide'])
    assert r.active is None and r.pending is take
    assert len(take) == 1 and take.duration == 0


def test_finalization_retains_all_observed_modes_and_simulation_stamp():
    r = started()
    r.sample(10., [0.] * 14, ['B:guide', 'G:mirror'])
    r.sample(10.1, [0.] * 14, ['B:guide', 'G:mirror'])
    take = r.freeze()  # Same method invoked by w and the sample-limit handler.
    assert take.meta['modes'] == ['B:hold', 'G:hold', 'B:guide', 'G:mirror']
    assert take.meta['method'] == 'sim:B:hold+G:hold+B:guide+G:mirror'
    assert take.meta['simulated'] is True


def test_live_provenance_stays_live():
    r = started(False)
    assert r.freeze().meta['method'] == 'live:B:hold+G:hold'


def test_labels_are_relative_to_the_recording_start():
    r = started()
    r.sample(10., [0.] * 14, [])
    assert r.toggle_label(10.2) == 'bad'
    assert r.toggle_label(10.4) == 'good'
    r.sample(10.5, [0.] * 14, [])
    assert abs(r.freeze().bad_seconds() - .2) < 1e-12


def test_start_cannot_overwrite_active_or_unsaved_take():
    r = started()
    for freeze in [False, True]:
        if freeze:
            r.freeze()
        try:
            r.start(20., meta={}, modes=[])
        except RuntimeError:
            pass
        else:
            raise AssertionError('previous take overwritten')


def test_discard_allows_fresh_label_clock_and_mode_history():
    r = started()
    r.toggle_label(10.1)
    r.freeze()
    r.discard()
    r.start(50., meta={'simulated': False}, modes=['B:teleop'])
    r.sample(50.1, [0.] * 7, ['B:teleop'])
    take = r.freeze()
    assert r.label == 'good' and not take.meta.get('marks')
    assert abs(take.samples[0].t - .1) < 1e-12
    assert take.meta['modes'] == ['B:teleop']


def test_changed_joint_count_is_rejected_without_freezing_other_state():
    r = started()
    r.sample(10., [0.] * 14, [])
    try:
        r.sample(10.1, [0.] * 7, [])
    except ValueError:
        pass
    else:
        raise AssertionError('mixed layout accepted')
    assert len(r.active) == 1 and r.pending is None
    r.discard()
    assert r.active is None and r.pending is None


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
