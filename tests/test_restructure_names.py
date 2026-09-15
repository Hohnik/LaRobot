"""Check lexical name resolution without executing the inspected application."""
import importlib.util
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parent.parent
spec = importlib.util.spec_from_file_location('restructure_check', ROOT / 'checks/check_restructure.py')
checker = importlib.util.module_from_spec(spec)
spec.loader.exec_module(checker)


def test_lambda_arguments_and_closures_are_bound():
    assert checker.undefined_globals('''
def main():
    offset = 2
    return lambda live: [value + offset for value in live]
''') == []


def test_missing_import_in_nested_callback_is_rejected():
    assert checker.undefined_globals('''
def main():
    return lambda live: effective_limits(live)
''') == ['effective_limits']


def test_unrelated_local_assignment_cannot_hide_missing_global():
    assert checker.undefined_globals('''
def unrelated():
    effective_limits = None

def main():
    return effective_limits()
''') == ['effective_limits']


def test_unrelated_import_cannot_hide_missing_global():
    assert checker.undefined_globals('''
def unrelated():
    from somewhere import effective_limits

def main():
    return effective_limits()
''') == ['effective_limits']


def test_actual_imports_and_nested_local_imports_are_bound():
    assert checker.undefined_globals('''
from somewhere import effective_limits

def main():
    from elsewhere import helper
    return lambda live: helper(effective_limits(live))
''') == []


def test_function_parameters_do_not_leak_into_sibling_scopes():
    assert checker.undefined_globals('''
def main():
    def helper(live):
        return live
    return live
''') == ['live']


def test_fstring_missing_names_are_checked():
    assert checker.undefined_globals('''
def main():
    return f"limit {missing_limit}"
''') == ['missing_limit']


def test_actual_application_has_no_missing_global_reads():
    assert checker.undefined_globals((ROOT / 'apps/teleop_session.py').read_text()) == []


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
