#!/usr/bin/env python3
"""Run every `falsify_*.py` and print ONE catch total. No hardware.

    uv run checks/run_falsifiers.py
    uv run checks/run_falsifiers.py --dir checks    # exists so this can be aimed elsewhere

⛔⭐⭐ WHY THIS EXISTS, and it closes a hole that had been open since the first falsifier.

[HANDOFF §4](../docs/HANDOFF.md) rule 4 says the evidence a checker works is **a green run PLUS a stable catch count**. A green run on its own went blind three times in this repo without anyone noticing ([FINDINGS §70.8](../docs/FINDINGS.md)). So five falsifiers exist, each feeding its checker known-broken input and counting the catches.

⛔ **Nothing ran them together, and nothing totalled them.** `checks/run_tests.py` collects `tests/test_*.py` only. So the catch counts had to be gathered by hand, from five different summary formats, which is exactly the friction that means nobody does it. ⚠️ And the cost was already paid: `falsify_check_dataset.py` had been unable to find its own input since the batch pipeline landed, printing "no exported episode to break" and exiting 1 into a terminal nobody was watching ([FINDINGS §76.7](../docs/FINDINGS.md)).

⭐ **So every falsifier now ends with one machine-readable line, `CATCHES: n/m`**, on both its passing and its failing path, and this command sums them. **The total is the number to compare against the last committed figure.** A total that drops while everything is green means a falsifier stopped falsifying, which means a checker's green runs stopped being evidence.

## ⚠️ Why this is a separate command instead of part of the suite

Three reasons, and the first is the real one.

- **A falsifier deliberately breaks things.** `falsify_fake_arm.py` monkey-patches a class and restores it; `falsify_check_flags.py` and `falsify_run_tests.py` write known-bad fixture files. Running that inside a **parallel** test runner invites one test file to observe another's sabotage, which produces exactly the intermittent suite this repo has just finished fixing.
- **They answer a different question.** The suite asks "does the code work". These ask "would we notice if it did not". Two questions, two totals, and mixing the numbers hides a drop in either.
- **They are cheap, so there is no excuse.** All five together take under two seconds.

⚠️ **What this cannot do.** It reads a number each falsifier reports about itself. A falsifier that miscounts its own catches lies here too. The defence for that is the same one recursing: `falsify_run_tests.py` exists because `run_tests.py` needed one, and this runner would need its own if it ever grew logic worth doubting. Today it runs subprocesses and adds up integers.
"""

from __future__ import annotations

import argparse
import re
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent

from yam.files import listing  # noqa: E402 — the OS-litter filter, FINDINGS §76

#: The one line every falsifier prints last. Both its paths print it.
CATCH_LINE = re.compile(r"^CATCHES:\s*(\d+)\s*/\s*(\d+)\s*$", re.M)


def run_one(path: Path) -> tuple[Path, int, int, bool, str]:
    """Run one falsifier. `(path, caught, expected, ok, output)`.

    ⛔ A missing `CATCHES:` line is a FAILURE, never a skip. A falsifier that crashes before
    reporting has told you nothing, and treating silence as success is how the whole
    catch-count discipline would quietly stop meaning anything.
    """
    proc = subprocess.run([sys.executable, str(path)], capture_output=True,
                          text=True, cwd=REPO, check=False)
    out = (proc.stdout + proc.stderr).strip()
    m = CATCH_LINE.search(out)
    if m is None:
        return path, 0, 0, False, out
    caught, expected = int(m.group(1)), int(m.group(2))
    ok = proc.returncode == 0 and caught == expected and expected > 0
    return path, caught, expected, ok, out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--dir", default=str(REPO / "checks"),
                    help="directory holding falsify_*.py (default: checks/)")
    args = ap.parse_args()

    files = listing(Path(args.dir), "falsify_*.py")
    if not files:
        # ⛔ Zero files is a broken invocation, never a clean run — the negative-results
        # rule: an empty search must not read as "everything passed".
        print(f"⛔ no falsify_*.py found in {args.dir}")
        return 1

    # ⚠️ SEQUENTIAL ON PURPOSE. These sabotage things. See the module docstring.
    results = [run_one(p) for p in files]

    caught_total = expected_total = 0
    failures = []
    width = max(len(p.name) for p, *_ in results)
    for path, caught, expected, ok, out in results:
        caught_total += caught
        expected_total += expected
        print(f"{'✓' if ok else '⛔'} {path.name:<{width}}  {caught}/{expected}")
        if not ok:
            failures.append((path, out))

    print(f"\n{'⛔' if failures else '✓'} CATCH TOTAL: {caught_total}/{expected_total} "
          f"across {len(results)} falsifiers"
          + (f", {len(failures)} FAILING" if failures else ""))
    for path, out in failures:
        print(f"\n──── {path.name} ────")
        print(out)
    print("\n⚠️  Compare the CATCH TOTAL against the last committed figure. A drop while "
          "everything is green\n   means a falsifier stopped falsifying, so some checker's "
          "green runs are no longer evidence.")
    # ⭐ THE VERDICT GOES LAST, for the reason FINDINGS §76.13 records: a `| tail` must not
    # be able to hide which one failed.
    if failures:
        print("\n⛔ FAILING, and this line is last on purpose so a `| tail` cannot hide it:")
        for path, _ in failures:
            print(f"   {path.name}")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
