#!/usr/bin/env python3
"""Tests for `checks/run_falsifiers.py` — the command that totals the catch counts.

    uv run tests/test_run_falsifiers.py

⛔ WHY THIS EXISTS. `run_falsifiers.py` is the thing that would tell you a falsifier has gone
blind, so it is the last place that can afford to be trusted on faith. Its own docstring says
it would need falsifying "if it ever grew logic worth doubting", and it has three pieces worth
doubting: a falsifier reporting fewer catches than it expects, one that crashes before
reporting anything at all, and an empty search that must not read as success.

⭐ Every case runs against fixture files in a temp directory, through the runner's own `--dir`
flag. Nothing real is touched and no checker is actually run.
"""

from __future__ import annotations

import subprocess
import sys
import tempfile
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
RUNNER = REPO / "checks" / "run_falsifiers.py"


def fixture_dir(files: dict[str, str]) -> Path:
    """A temp directory of `falsify_*.py` fixtures. Values are the whole file body."""
    root = Path(tempfile.mkdtemp())
    for name, body in files.items():
        (root / name).write_text(body)
    return root


def run(root: Path) -> tuple[int, str]:
    proc = subprocess.run([sys.executable, str(RUNNER), "--dir", str(root)],
                          capture_output=True, text=True, cwd=REPO, check=False)
    return proc.returncode, proc.stdout + proc.stderr


HEALTHY = 'print("all good")\nprint("CATCHES: 5/5")\n'
BLIND = 'print("two got past me")\nprint("CATCHES: 3/5")\nimport sys; sys.exit(1)\n'
#: ⛔ Reports a full count and exits nonzero. A falsifier that lies about being fine.
LIAR = 'print("CATCHES: 5/5")\nimport sys; sys.exit(1)\n'
CRASHER = 'raise SystemExit("died before it could count")\n'
#: ⛔ Exits 0 and never says how many it caught. Silence must not read as success.
SILENT = 'print("looks fine to me")\n'


def test_a_healthy_falsifier_passes_and_is_totalled():
    code, out = run(fixture_dir({"falsify_a.py": HEALTHY, "falsify_b.py": HEALTHY}))
    assert code == 0, out
    assert "CATCH TOTAL: 10/10 across 2 falsifiers" in out, out


def test_a_falsifier_that_caught_FEWER_than_it_expects_is_flagged():
    # ⭐ This is the case the whole command exists for: the count dropped.
    code, out = run(fixture_dir({"falsify_a.py": HEALTHY, "falsify_b.py": BLIND}))
    assert code == 1, out
    assert "⛔ falsify_b.py" in out, out
    assert "CATCH TOTAL: 8/10" in out, out


def test_a_falsifier_that_reports_a_full_count_and_exits_NONZERO_is_still_flagged():
    # ⛔ Two independent signals, because trusting either alone is how red sits unseen.
    code, out = run(fixture_dir({"falsify_a.py": LIAR}))
    assert code == 1, out
    assert "⛔ falsify_a.py" in out, out


def test_a_falsifier_that_crashes_before_counting_is_a_FAILURE_never_a_skip():
    code, out = run(fixture_dir({"falsify_a.py": CRASHER}))
    assert code == 1, out
    assert "⛔ falsify_a.py" in out, out


def test_a_falsifier_that_never_says_its_count_is_a_FAILURE_never_a_skip():
    """⛔ Exits 0, prints something reassuring, reports no number. Silence is not evidence."""
    code, out = run(fixture_dir({"falsify_a.py": SILENT}))
    assert code == 1, out
    assert "⛔ falsify_a.py" in out, out
    assert "0/0" in out, out


def test_an_EMPTY_directory_fails_rather_than_reading_as_a_clean_run():
    # ⭐ The negative-results rule: an empty search must never read as "everything passed".
    code, out = run(fixture_dir({}))
    assert code == 1, out
    assert "no falsify_*.py found" in out, out


def test_the_verdict_survives_a_tail_because_it_prints_LAST():
    # ⛔ FINDINGS §76.13: a `| tail -3` threw away the name of a real failure once.
    _, out = run(fixture_dir({"falsify_a.py": HEALTHY, "falsify_b.py": BLIND}))
    last_four = "\n".join(out.strip().splitlines()[-4:])
    assert "this line is last on purpose" in last_four, last_four
    assert "falsify_b.py" in last_four, last_four


def test_the_real_falsifiers_all_report_a_count():
    """⭐ Runs against the real `checks/` directory, and asserts only the PROPERTY that every
    falsifier reports a machine-readable count. It does not assert the total, because a total
    written into a test is a number that goes stale (FINDINGS §33.3)."""
    from yam.files import listing  # noqa: PLC0415

    real = listing(REPO / "checks", "falsify_*.py")
    assert real, "there are no falsifiers at all, which cannot be right"
    for path in real:
        body = path.read_text()
        assert "CATCHES:" in body, (
            f"{path.name} prints no `CATCHES: n/m` line, so run_falsifiers.py cannot total it")


def main() -> int:
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failed = 0
    for fn in tests:
        try:
            fn()
        except AssertionError as e:  # noqa: PERF203
            failed += 1
            print(f"✗ {fn.__name__}: {e}")
        else:
            print(f"✓ {fn.__name__}")
    print(f"\n{len(tests) - failed}/{len(tests)} passed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
