"""Failed slot publication preserves the previous take and pending frames."""
from pathlib import Path
from tempfile import TemporaryDirectory
from unittest.mock import patch
import json
import sys
from yam.recording import Trajectory
from yam.recording_store import save_take

def trajectory(marker):
    t = Trajectory(meta={'marker': marker, 'simulated': True})
    t.append(0.0, [0.0] * 7)
    t.append(1.0, [0.1] * 7)
    return t

def fixture(root):
    old = root / '4.json'
    trajectory('old').save(old)
    frames = root / 'frames' / '4'
    frames.mkdir(parents=True)
    (frames / 'old.jpg').write_bytes(b'old frame')
    pending = root / 'frames' / 'pending_test'
    pending.mkdir()
    (pending / 'new.jpg').write_bytes(b'new frame')
    report = {'mono0_ns': 1000, 'per_camera': {'camera': {'written': 1, 'dropped': 0, 'write_errors': 0, 'flushed': True}}}
    return (old, frames, pending, report)

def original_intact(root, pending):
    assert Trajectory.load(root / '4.json').meta['marker'] == 'old'
    assert (root / 'frames/4/old.jpg').read_bytes() == b'old frame'
    assert (pending / 'new.jpg').read_bytes() == b'new frame'
    assert not list(root.glob('.save-4-*'))

def test_success_publishes_json_and_frames():
    with TemporaryDirectory() as d:
        root = Path(d)
        (old, frames, pending, report) = fixture(root)
        take = trajectory('new')
        result = save_take(take, root, '4', pending_frames=pending, frame_report=report)
        got = Trajectory.load(old)
        assert got.meta['marker'] == 'new' and got.meta['cameras']['dir'] == 'frames/4'
        assert got.meta['cameras']['mono0_ns'] == 1000 and got.meta['commit']
        assert (frames / 'new.jpg').exists() and (not (frames / 'old.jpg').exists())
        assert result.has_frames and (not result.warning) and (not pending.exists())

def test_serialization_failure_does_not_touch_previous_slot():
    with TemporaryDirectory() as d:
        root = Path(d)
        (_, _, pending, report) = fixture(root)
        take = trajectory('new')
        with patch.object(Trajectory, 'save', side_effect=OSError('disk full')):
            try:
                save_take(take, root, '4', pending_frames=pending, frame_report=report)
            except OSError:
                pass
            else:
                raise AssertionError('failure hidden')
        original_intact(root, pending)
        assert 'cameras' not in take.meta

def test_failed_json_publish_restores_previous_slot_and_pending_frames():
    with TemporaryDirectory() as d:
        root = Path(d)
        (_, _, pending, report) = fixture(root)
        with patch.object(Path, 'replace', side_effect=OSError('publish failed')):
            try:
                save_take(trajectory('new'), root, '4', pending_frames=pending, frame_report=report)
            except OSError:
                pass
            else:
                raise AssertionError('failure hidden')
        original_intact(root, pending)

def test_interrupt_during_publish_rolls_back():
    with TemporaryDirectory() as d:
        root = Path(d)
        (_, _, pending, report) = fixture(root)
        with patch.object(Path, 'replace', side_effect=KeyboardInterrupt):
            try:
                save_take(trajectory('new'), root, '4', pending_frames=pending, frame_report=report)
            except KeyboardInterrupt:
                pass
            else:
                raise AssertionError('interrupt swallowed')
        original_intact(root, pending)

def test_old_json_is_hidden_during_frame_swap():
    with TemporaryDirectory() as d:
        root = Path(d)
        (old, _, pending, report) = fixture(root)
        rename = Path.rename
        seen = []

        def observe(path, target):
            if path == pending:
                seen.append(not old.exists())
            return rename(path, target)
        with patch.object(Path, 'rename', observe):
            save_take(trajectory('new'), root, '4', pending_frames=pending, frame_report=report)
        assert seen == [True]

def test_failed_frame_move_restores_old_json_last():
    with TemporaryDirectory() as d:
        root = Path(d)
        (_, _, pending, report) = fixture(root)
        rename = Path.rename

        def fail(path, target):
            if path == pending:
                raise OSError('frame move failed')
            return rename(path, target)
        with patch.object(Path, 'rename', fail):
            try:
                save_take(trajectory('new'), root, '4', pending_frames=pending, frame_report=report)
            except OSError:
                pass
            else:
                raise AssertionError('failure hidden')
        original_intact(root, pending)

def test_frameless_replacement_removes_stale_frames_after_success():
    with TemporaryDirectory() as d:
        root = Path(d)
        (_, frames, _, _) = fixture(root)
        result = save_take(trajectory('new'), root, '4')
        assert not frames.exists() and (not result.has_frames)
        assert 'cameras' not in Trajectory.load(root / '4.json').meta

def test_failed_rollback_retains_recovery_files_and_refuses_another_save():
    with TemporaryDirectory() as d:
        root = Path(d)
        (old, frames, pending, report) = fixture(root)
        rename = Path.rename

        def fail_restore(path, target):
            if path == frames and Path(target) == pending:
                raise OSError('rollback failed')
            return rename(path, target)
        with patch.object(Path, 'rename', fail_restore), patch.object(Path, 'replace', side_effect=OSError('publish failed')):
            try:
                save_take(trajectory('new'), root, '4', pending_frames=pending, frame_report=report)
            except OSError as exc:
                assert 'rollback failed' in str(exc.__notes__)
            else:
                raise AssertionError('failure hidden')
        stage = next(root.glob('.save-4-*'))
        assert (stage / 'previous.json').exists() and (stage / 'previous.frames/old.jpg').exists()
        assert (stage / 'new.json').exists() and (stage / 'recovery.json').exists()
        assert not old.exists()
        try:
            save_take(trajectory('another'), root, '4')
        except RuntimeError as exc:
            assert 'Recover' in str(exc)
        else:
            raise AssertionError('unresolved recovery ignored')

def test_validation_refuses_unpaired_frames_without_creating_recovery_debris():
    with TemporaryDirectory() as d:
        root = Path(d)
        (_, _, pending, _) = fixture(root)
        try:
            save_take(trajectory('new'), root, '4', pending_frames=pending)
        except ValueError:
            pass
        else:
            raise AssertionError('missing report accepted')
        original_intact(root, pending)

def main():
    tests = [v for (k, v) in globals().items() if k.startswith('test_') and callable(v)]
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
