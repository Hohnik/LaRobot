#!/usr/bin/env python3
"""Prove `run_tests.py` can actually see a failure — all four kinds.

    uv run checks/falsify_run_tests.py

⭐ A checker is trusted only after it has been fed known-broken input and caught it ([FINDINGS §59.1](../docs/FINDINGS.md)): a green run proves nothing about a checker whose rules were silently disarmed. This writes four fixture files into a temp directory — one passing, one with a failing assert, one that crashes before printing a count, one that LIES (prints `3/3 passed` but exits nonzero) — and requires the runner to fail the last three and count the first.
"""

from __future__ import annotations

import subprocess
import sys
import tempfile
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent

FIXTURES = {
    "test_good.py": 'print("2/2 passed")\n',
    "test_assert_fail.py": 'print("1/2 passed")\nraise SystemExit(1)\n',
    "test_crash.py": 'raise RuntimeError("boom before any count")\n',
    "test_liar.py": 'print("3/3 passed")\nraise SystemExit(1)\n',
}


def main() -> int:
    with tempfile.TemporaryDirectory() as d:
        for name, body in FIXTURES.items():
            (Path(d) / name).write_text(body)
        proc = subprocess.run(
            [sys.executable, str(REPO / "checks" / "run_tests.py"), "--dir", d],
            capture_output=True, text=True, check=False)
        out = proc.stdout + proc.stderr

    caught = out.count("⛔ test_")
    checks = [
        ("the runner exits nonzero on a broken suite", proc.returncode != 0),
        ("exactly 3 of 4 fixtures are caught", caught == 3),
        ("the passing fixture is not flagged", "⛔ test_good.py" not in out),
        ("the crash (no count line) is caught", "⛔ test_crash.py" in out),
        ("a lying exit code is caught despite its count", "⛔ test_liar.py" in out),
        ("the total says how many files fail", "3 file(s) FAILING" in out),
    ]
    failed = 0
    for label, ok in checks:
        print(f"{'✓' if ok else '✗'} {label}")
        failed += 0 if ok else 1
    if failed:
        print("\n──── runner output ────\n" + out)
    print(f"\n{len(checks) - failed}/{len(checks)} passed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
