"""Static flag checking must follow the CLI owner without executing its module."""
from contextlib import redirect_stdout
import importlib.util
import io
from pathlib import Path
import sys
from tempfile import TemporaryDirectory
from unittest.mock import patch

ROOT = Path(__file__).resolve().parent.parent
spec = importlib.util.spec_from_file_location('flag_sources_checker', ROOT / 'checks/check_flags.py')
checker = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = checker
spec.loader.exec_module(checker)


def inspect(owner, entry='from yam.cli import build_parser as parser\nap = parser()\n'):
    with TemporaryDirectory() as directory:
        root = Path(directory)
        (root / 'src/yam').mkdir(parents=True)
        app = root / 'app.py'
        app.write_text(entry)
        if owner is not None:
            (root / 'src/yam/cli.py').write_text(owner)
        with patch.object(checker, 'REPO', root), patch.object(checker, '_SRC_CONSTS', None):
            return checker.read_parser(app)


def test_called_alias_follows_owner_without_execution():
    parser = inspect('''raise AssertionError("must never execute")
def build_parser():
    ap.add_argument("--mode", choices=["hold", "teleop"])
    ap.add_argument("--speed", type=float)
    ap.add_argument("--yes", action="store_true")
    return ap
''')
    assert parser.has_parser and len(parser.flags) == 3
    assert checker.check_value(parser.flags['--mode'], 'flying') is not None
    assert checker.check_value(parser.flags['--speed'], 'fast') is not None
    assert not parser.flags['--yes'].takes_value


def test_unused_import_does_not_add_an_unrelated_parser():
    parser = inspect('ap.add_argument("--unused")', 'from yam.cli import build_parser\n')
    assert not parser.has_parser and not parser.flags


def test_local_arguments_override_imported_definitions():
    parser = inspect('ap.add_argument("--mode", choices=["old"])',
                     'from yam.cli import build_parser\nap = build_parser()\n'
                     'ap.add_argument("--mode", choices=["new"])\n')
    assert parser.flags['--mode'].choices == ['new']


def test_missing_builder_is_known_parser_with_no_accepted_flags():
    parser = inspect(None)
    assert parser.has_parser and not parser.flags


def test_syntax_error_cannot_hide_documented_flags():
    parser = inspect('def build_parser(:')
    assert parser.has_parser and not parser.flags


def test_cyclic_builder_stops_without_accepting_unknown_flags():
    parser = inspect('from yam.cli import build_parser\nap = build_parser()\n')
    assert parser.has_parser and not parser.flags


def test_missing_builder_causes_document_check_to_fail():
    with TemporaryDirectory() as directory:
        root = Path(directory)
        (root / 'app.py').write_text('from yam.missing import build_parser\nap = build_parser()\n')
        doc = root / 'commands.md'
        doc.write_text('`uv run app.py --yes`')
        commands = [(doc, 1, 'app.py', ' --yes')]
        output = io.StringIO()
        with patch.object(checker, 'REPO', root), patch.object(checker, 'DOCS', root), \
             patch.object(checker, '_SRC_CONSTS', None), \
             patch.object(checker, 'documented_commands', return_value=commands), redirect_stdout(output):
            code = checker.main()
        assert code == 1 and 'has no flag --yes' in output.getvalue()


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
