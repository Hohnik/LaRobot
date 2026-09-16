"""Inspect real interrupted-publication layouts without repairing or deleting data."""
import json
from pathlib import Path
import subprocess
import sys
from tempfile import TemporaryDirectory
from unittest.mock import patch

from yam.recording_recovery import inspect_recovery
from yam.recording_store import save_take
from test_recording_store import fixture, trajectory

ROOT = Path(__file__).resolve().parent.parent


def snapshot(root):
    return {str(p.relative_to(root)): (p.read_bytes(), p.stat().st_mtime_ns)
            for p in root.rglob('*') if p.is_file() and not p.is_symlink()}


def test_failed_rollback_inventory_preserves_both_versions():
    with TemporaryDirectory() as d:
        root = Path(d)
        old, frames, pending, report = fixture(root)
        rename = Path.rename
        def fail_restore(path, target):
            if path == frames and Path(target) == pending:
                raise OSError('rollback failed')
            return rename(path, target)
        with patch.object(Path, 'rename', fail_restore), patch.object(Path, 'replace', side_effect=OSError('publish failed')):
            try:
                save_take(trajectory('new'), root, '4', pending_frames=pending, frame_report=report)
            except OSError:
                pass
            else:
                raise AssertionError('failure not injected')
        before = snapshot(root)
        got = inspect_recovery(root)
        assert snapshot(root) == before
        assert got['needs_inspection'] and len(got['interrupted_saves']) == 1
        stage = got['interrupted_saves'][0]
        assert stage['published']['state'] == 'absent'
        assert stage['candidate']['samples'] == stage['previous']['samples'] == 2
        assert stage['previous_frames']['images'] == stage['slot_frames']['images'] == 1
        assert got['retained_frames'][0]['path'] == 'frames/4'


def test_successful_publication_with_failed_cleanup_is_not_called_safe():
    with TemporaryDirectory() as d:
        root = Path(d)
        fixture(root)
        with patch('yam.recording_store.shutil.rmtree', side_effect=OSError('busy')):
            saved = save_take(trajectory('new'), root, '4')
        assert saved.warning
        got = inspect_recovery(root)
        stage = got['interrupted_saves'][0]
        assert stage['published']['state'] == 'readable'
        assert stage['candidate']['state'] == 'absent'
        assert stage['previous']['state'] == 'readable' and got['needs_inspection']


def test_only_pending_images_do_not_imply_recovered_joint_samples():
    with TemporaryDirectory() as d:
        root = Path(d)
        frames = root / 'frames/pending_take/front'
        frames.mkdir(parents=True)
        (frames / '0001.jpg').write_bytes(b'image')
        (frames / '._0001.jpg').write_bytes(b'sidecar')
        (frames / 'index.json').write_text('{}')
        got = inspect_recovery(root)
        assert not got['published'] and not got['interrupted_saves']
        assert got['retained_frames'][0]['images'] == 1
        assert got['retained_frames'][0]['indexes'] == 1
        assert got['needs_inspection'] and 'in-memory joint samples' in got['limits']


def test_manifest_paths_are_displayed_but_never_followed():
    with TemporaryDirectory() as d:
        root = Path(d)
        stage = root / '.save-4-test'
        stage.mkdir()
        (stage / 'recovery.json').write_text(json.dumps({'slot': '../outside', 'pending_frames': '/outside'}))
        trajectory('candidate').save(stage / 'new.json')
        got = inspect_recovery(root)['interrupted_saves'][0]
        assert 'invalid slot' in got['error'] and 'published' not in got
        assert got['candidate']['samples'] == 2
        (stage / 'recovery.json').write_text(json.dumps({'slot': '4', 'pending_frames': '/outside'}))
        got = inspect_recovery(root)['interrupted_saves'][0]
        assert got['pending_frames_recorded'] == '/outside'


def test_malformed_manifest_and_json_leave_other_evidence_visible():
    with TemporaryDirectory() as d:
        root = Path(d)
        stage = root / '.save-4-test'
        stage.mkdir()
        (stage / 'recovery.json').write_text('{broken')
        (stage / 'new.json').write_text('{broken')
        trajectory('previous').save(stage / 'previous.json')
        (root / '4.json').write_text('{broken')
        report = inspect_recovery(root)
        got = report['interrupted_saves'][0]
        assert 'error' in got and got['candidate']['state'] == 'unreadable'
        assert got['previous']['samples'] == 2
        assert report['published']['4.json']['state'] == 'unreadable'


def test_symlink_evidence_is_reported_without_traversal():
    with TemporaryDirectory() as d:
        root = Path(d)
        outside = root / 'outside'
        outside.mkdir()
        (outside / 'secret.jpg').write_bytes(b'not inspected')
        (root / 'frames').symlink_to(outside, target_is_directory=True)
        (root / '.save-4-test').symlink_to(outside, target_is_directory=True)
        (root / '4.json').symlink_to(outside / 'secret.jpg')
        got = inspect_recovery(root)
        assert got['retained_frames'] == [{'path': 'frames', 'state': 'symlink_skipped'}]
        assert got['published']['4.json']['state'] == 'symlink_skipped'
        assert 'error' in got['interrupted_saves'][0]


def test_claimed_frames_are_not_reported_as_orphans():
    with TemporaryDirectory() as d:
        root = Path(d)
        frame_dir = root / 'frames/4'
        frame_dir.mkdir(parents=True)
        take = trajectory('published')
        take.meta['cameras'] = {'dir': 'frames/4'}
        take.save(root / '4.json')
        assert not inspect_recovery(root)['needs_inspection']


def test_cli_reports_pending_evidence_even_without_published_slots():
    with TemporaryDirectory() as d:
        root = Path(d)
        (root / 'frames/pending_take').mkdir(parents=True)
        before = snapshot(root)
        for script in ('check_recording_recovery.py', 'check_recordings.py'):
            result = subprocess.run([sys.executable, str(ROOT / 'checks' / script), '--dir', d],
                                    capture_output=True, text=True)
            assert result.returncode == (1 if script == 'check_recording_recovery.py' else 0), result.stderr
            assert 'pending_take' in result.stdout and 'safe to delete' not in result.stdout
        assert snapshot(root) == before


def test_cli_distinguishes_missing_directory_and_no_recovery_evidence():
    with TemporaryDirectory() as d:
        command = [sys.executable, str(ROOT / 'checks/check_recording_recovery.py'), '--json', '--dir']
        empty = subprocess.run(command + [d], capture_output=True, text=True)
        assert empty.returncode == 0 and not json.loads(empty.stdout)['needs_inspection']
        missing = subprocess.run(command + [d + '/missing'], capture_output=True, text=True)
        assert missing.returncode == 2 and 'does not exist' in missing.stderr


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
