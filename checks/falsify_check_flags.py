#!/usr/bin/env python3
"""Can `checks/check_flags.py` actually SEE a broken documented command?

    uv run checks/falsify_check_flags.py

⭐⭐ WHY THIS EXISTS. `check_flags.py` came up green on the real docs the first time it
ran, and a green checker that has never caught anything is indistinguishable from a
checker that cannot catch anything. So this writes a throwaway document full of
deliberately broken commands and insists each one is reported.

⛔⭐ IT HAS ALREADY PAID FOR ITSELF THREE TIMES, and each miss was a different lesson:

| what was missed | why | the general lesson |
|---|---|---|
| `--arm Q` and `--arms B,Q` | `choices=sorted(ARM_SERIALS)` and `ARM_SERIALS` lives in `src/yam/can.py`, so the choices would not resolve and the check was **skipped in silence** | ⛔ **a check that cannot resolve its data must not pass quietly** |
| nine FALSE positives on the real docs | the command pattern read `A && B --arm B --yes` as one command and blamed B's flags on A | ⚠️ noise gets a checker ignored, which is worse than not having one |
| `--arm Q`, a second time | the placeholder rule counted any ALL-CAPS word as a placeholder, and arm names are single capitals | ⛔ **a rule added to remove a false positive can silently create a false negative** |

⚠️ The last row is the one worth remembering: the placeholder rule was added to fix a
real false positive on `--arm <B|G>`, and it quietly disarmed a real check in the same
edit. **Only a falsification run makes that visible**, because both the before and the
after look green.

⛔ It writes only into a temporary directory and never touches `docs/`.
"""

from __future__ import annotations

import io
import sys
import tempfile
from contextlib import redirect_stdout
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "checks"))

import check_flags  # noqa: E402

#: Each entry: (label, the documented line, must it be reported?)
CASES = [
    ("a flag that does not exist",
     "`uv run apps/teleop_session.py --yes --arm B --turbo`", True),
    ("an arm name outside choices, resolved from src/yam/can.py",
     "`uv run apps/teleop_session.py --yes --arm Q`", True),
    ("a start mode outside choices",
     "`uv run apps/teleop_session.py --yes --arm B --start-mode flying`", True),
    ("a word where a float is required",
     "`uv run apps/teleop_session.py --yes --arm B --max-speed fast`", True),
    ("a mirror mode outside choices",
     "`uv run apps/teleop_session.py --yes --arms B,G --mirror sideways`", True),
    ("a bad arm inside a comma list, via the plural rule",
     "`uv run apps/teleop_session.py --yes --arms B,Q`", True),
    ("a script that does not exist",
     "`uv run scripts/does_not_exist.py --arm B`", True),
    # ---- these must NOT be reported ----
    ("a CORRECT command",
     "`uv run apps/teleop_session.py --yes --arms B,G --start-mode hold "
     "--max-speed 2 --mirror copy`", False),
    ("a chain, whose second command must not be blamed on the first",
     "`uv run checks/check_rig.py && uv run apps/ping_motors.py --arm B --yes`",
     False),
    ("a legitimate placeholder",
     "`uv run apps/ping_motors.py --arm <B|G> --yes`", False),
]


def report_for(line: str) -> list[str]:
    """Run the checker over a document holding exactly one command. Returns its ⛔ lines."""
    with tempfile.TemporaryDirectory() as tmp:
        doc = Path(tmp) / "BROKEN.md"
        doc.write_text(f"# throwaway\n\n{line}\n")
        original, check_flags.DOCS = check_flags.DOCS, Path(tmp)
        buf = io.StringIO()
        try:
            with redirect_stdout(buf):
                check_flags.main()
        finally:
            check_flags.DOCS = original
    return [ln for ln in buf.getvalue().splitlines()
            if ln.startswith("⛔") and "WOULD NOT RUN" not in ln]


def main() -> int:
    print("Falsifying checks/check_flags.py — every break must be reported, and every\n"
          "correct command must not be.\n")
    wrong = 0
    for label, line, should_report in CASES:
        found = report_for(line)
        ok = bool(found) == should_report
        wrong += not ok
        mark = "✓" if ok else "⛔"
        want = "reported" if should_report else "left alone"
        got = "reported" if found else "left alone"
        print(f"  {mark} {label}\n        wanted {want}, got {got}")
        if not ok and found:
            for f in found:
                print(f"        {f}")
    print()
    if wrong:
        print(f"⛔ {wrong} case(s) went the wrong way. The checker is not trustworthy "
              f"until this is clean.")
    else:
        print("✓ every deliberate break was caught and every correct command was left "
              "alone.")
    # ⭐ `CATCHES: n/m` is the one line `checks/run_falsifiers.py` reads. Every falsifier
    # ends with it so the catch counts can be TOTALLED, which is what rule 4 actually asks
    # for: a green run plus a stable catch count is evidence, and a green run alone is not.
    print(f"CATCHES: {len(CASES) - wrong}/{len(CASES)}")
    return 1 if wrong else 0


if __name__ == "__main__":
    sys.exit(main())
