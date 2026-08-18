#!/usr/bin/env python3
"""Run EVERY test file as one suite and count the catches.

    uv run checks/run_tests.py

⛔ WHY THIS EXISTS ([FINDINGS §67.5](../docs/FINDINGS.md)): each `tests/test_*.py` carries its own `main()`, nothing ran them as a whole, and two files sat red for days — one a real regression the tests had caught and nobody collected. A suite that is never run as a whole is a fixture with no counter.

⭐ THE TOTAL IS THE POINT, not a nicety: it is this repo's §59.1 remedy applied to the tests themselves. A rule-loosening that silently disarms checks shows up here as the number going DOWN, which a green run alone can never show.

A file fails if its process exits nonzero, if its last line is not `N/M passed`, or if N < M — three separate signals, because a crashed file prints a traceback and no count at all, and trusting any single signal is how red sits unseen.

⚠️ This is ROADMAP §10.5 step 2 (collection only, no moves). pytest is deliberately NOT introduced here: it is not a dependency of this repo today, and choosing it belongs with the step-3/4 package restructure, next to the team's own pytest layout in LaRobot.
"""

from __future__ import annotations

import argparse
import re
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
COUNT_LINE = re.compile(r"(\d+)\s*/\s*(\d+) passed\s*$")


def run_one(path: Path) -> tuple[Path, int, int, bool, str]:
    """Run one test file. Returns (path, passed, total, ok, output)."""
    proc = subprocess.run([sys.executable, str(path)], capture_output=True,
                          text=True, cwd=REPO, check=False)
    out = (proc.stdout + proc.stderr).strip()
    last = out.splitlines()[-1] if out else ""
    m = COUNT_LINE.search(last)
    if m is None:
        # A crash prints a traceback and no count. That is a failure, never a skip.
        return path, 0, 0, False, out
    passed, total = int(m.group(1)), int(m.group(2))
    ok = proc.returncode == 0 and passed == total
    return path, passed, total, ok, out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--dir", default=str(REPO / "tests"),
                    help="directory holding test_*.py (default: scripts/; exists so the falsifier can aim this at known-bad fixtures)")
    ap.add_argument("-j", "--jobs", type=int, default=4,
                    help="parallel test processes (default 4)")
    args = ap.parse_args()

    files = sorted(Path(args.dir).glob("test_*.py"))
    if not files:
        # ⛔ Zero files is a broken invocation, never a green suite — the
        # negative-results rule: an empty search must not read as "all passed".
        print(f"⛔ no test_*.py found in {args.dir}")
        return 1

    with ThreadPoolExecutor(max_workers=max(1, args.jobs)) as pool:
        results = list(pool.map(run_one, files))

    grand_passed = grand_total = 0
    failures = []
    width = max(len(p.name) for p, *_ in results)
    for path, passed, total, ok, out in results:
        grand_passed += passed
        grand_total += total
        mark = "✓" if ok else "⛔"
        print(f"{mark} {path.name:<{width}}  {passed}/{total}")
        if not ok:
            failures.append((path, out))

    print(f"\n{'⛔' if failures else '✓'} TOTAL: {grand_passed}/{grand_total} checks "
          f"across {len(results)} files"
          + (f", {len(failures)} file(s) FAILING" if failures else ""))
    for path, out in failures:
        print(f"\n──── {path.name} ────")
        print(out)
    # ⭐ Keep yesterday's total next to today's: a drop with everything green is a
    # silently disarmed check (§59.1), and only the reader can see it — so say it.
    print("\n⚠️  A green run proves the checks that exist still pass. Compare the TOTAL "
          "against the last committed figure: a drop means a check was disarmed.")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
