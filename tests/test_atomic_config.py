"""Configuration publication preserves the previous file across I/O failures."""
import json
from pathlib import Path
import stat
import sys
from tempfile import TemporaryDirectory
from unittest.mock import patch

import yam.files as files
from yam.settings import save_defaults


def fixture(root):
    path = root / 'config.json'
    path.write_text('{"previous": true}\n')
    path.chmod(0o640)
    return path


def unchanged(root, path, before):
    assert path.read_bytes() == before
    assert sorted(p.name for p in root.iterdir()) == ['config.json']


def test_success_publishes_complete_json_and_preserves_permissions():
    with TemporaryDirectory() as d:
        root = Path(d)
        path = fixture(root)
        files.write_json_atomic(path, {'value': 2})
        assert json.loads(path.read_text()) == {'value': 2}
        assert stat.S_IMODE(path.stat().st_mode) == 0o640
        assert sorted(p.name for p in root.iterdir()) == ['config.json']


def test_serialization_failure_does_not_touch_previous_file():
    with TemporaryDirectory() as d:
        root = Path(d)
        path = fixture(root)
        before = path.read_bytes()
        try:
            files.write_json_atomic(path, {'value': object()})
        except TypeError:
            pass
        else:
            raise AssertionError('unserializable value accepted')
        unchanged(root, path, before)


def test_partial_write_failure_leaves_old_config_intact():
    with TemporaryDirectory() as d:
        root = Path(d)
        path = fixture(root)
        before = path.read_bytes()
        original = files.tempfile.NamedTemporaryFile
        def temp(*args, **kwargs):
            stream = original(*args, **kwargs)
            write = stream.write
            def fail(text):
                write(text[:5]); stream.flush()
                raise OSError('disk full after partial write')
            stream.write = fail
            return stream
        with patch.object(files.tempfile, 'NamedTemporaryFile', temp):
            try:
                save_defaults(path, {'max_speed': 2.})
            except OSError as exc:
                assert 'partial write' in str(exc)
            else:
                raise AssertionError('write failure hidden')
        unchanged(root, path, before)


def test_replace_failure_preserves_old_config_and_cleans_temporary():
    with TemporaryDirectory() as d:
        root = Path(d)
        path = fixture(root)
        before = path.read_bytes()
        with patch.object(Path, 'replace', side_effect=PermissionError('replace denied')):
            try:
                files.write_json_atomic(path, {'new': True})
            except PermissionError:
                pass
            else:
                raise AssertionError('publication failure hidden')
        unchanged(root, path, before)


def test_interruption_cleans_candidate_and_propagates():
    with TemporaryDirectory() as d:
        root = Path(d)
        path = fixture(root)
        before = path.read_bytes()
        with patch.object(Path, 'replace', side_effect=KeyboardInterrupt):
            try:
                files.write_json_atomic(path, {'new': True})
            except KeyboardInterrupt:
                pass
            else:
                raise AssertionError('interrupt swallowed')
        unchanged(root, path, before)


def test_existing_symlink_stays_a_link_to_the_updated_target():
    with TemporaryDirectory() as d:
        root = Path(d)
        target = fixture(root)
        link = root / 'linked.json'
        link.symlink_to(target.name)
        files.write_json_atomic(link, {'new': True})
        assert link.is_symlink() and json.loads(target.read_text()) == {'new': True}


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
