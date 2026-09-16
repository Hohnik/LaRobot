"""The dataset falsifier must work in a fresh checkout without touching user data."""
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parent.parent


def test_fresh_checkout_runs_all_five_mutations_without_recordings():
    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp)
        checks = root / 'checks'
        checks.mkdir()
        for name in ('falsify_check_dataset.py', 'check_dataset.py'):
            shutil.copy2(ROOT / 'checks' / name, checks / name)
        result = subprocess.run([sys.executable, str(checks / 'falsify_check_dataset.py')],
                                cwd=root, text=True, capture_output=True)
        assert result.returncode == 0, result.stdout + result.stderr
        assert 'CATCHES: 5/5' in result.stdout
        assert not (root / 'recordings').exists()


def test_invalid_explicit_source_refuses_without_mutating_it():
    with tempfile.TemporaryDirectory() as tmp:
        source = Path(tmp)
        sentinel = source / 'operator-notes.txt'
        sentinel.write_bytes(b'preserve me')
        before = sentinel.stat().st_mtime_ns
        result = subprocess.run([sys.executable, str(ROOT / 'checks/falsify_check_dataset.py'),
                                 '--source', str(source)], text=True, capture_output=True)
        assert result.returncode == 1 and 'CATCHES: 0/5' in result.stdout, result.stdout
        assert sentinel.read_bytes() == b'preserve me'
        assert sentinel.stat().st_mtime_ns == before
        assert list(source.iterdir()) == [sentinel]


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
