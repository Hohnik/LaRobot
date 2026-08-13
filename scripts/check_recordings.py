#!/usr/bin/env python3
"""Report what is in `recordings/`, and whether any of it carries the §30.1 padding.

    uv run scripts/check_recordings.py
    uv run scripts/check_recordings.py --dir recordings --still 0.05

⛔ WHY THIS SCRIPT EXISTS, and it is a documentation failure rather than a code one.
[FINDINGS §30.1](../docs/FINDINGS.md) found that `w` did not stop a recording where it
should have, so each file carried the seconds the save prompt spent waiting. The handoff
then said, in prose, *"slots 1, 3, 4, 5, 6 are all padded, discard them"*. ⛔ **Three of
those five were recorded after the fix and are clean**, and the sentence had no way to
know: it was written once and never re-derived against the files. This is the same defect
this repo keeps finding in its own guards ([HANDOFF §4](../docs/HANDOFF.md) rule 7), and
the defence is to measure it instead of asserting it.

⭐ It reads the files only. No hardware, no motion, nothing is written.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "src"))

from recording import Trajectory  # noqa: E402

#: Above this, a tail is the §30.1 defect: it produced 1.8 to 4.4 s. Below it, a tail is
#: the arm coming to rest before the key was pressed, which is not a fault and not
#: something to re-record for. ⚠️ The gap between the two cases is wide in the measured
#: data, so this threshold is not delicate.
PADDING_S = 1.0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--dir", default="recordings", help="where the .json files are")
    ap.add_argument("--still", type=float, default=0.05,
                    help="rad/s below which a joint counts as not moving (default 0.05, "
                         "chosen above the 0.032-0.038 wobble floor of a held arm)")
    args = ap.parse_args()

    folder = REPO / args.dir if not Path(args.dir).is_absolute() else Path(args.dir)
    files = sorted(folder.glob("*.json"))
    if not files:
        print(f"no recordings in {folder}")
        return 0

    print(f"{'file':>9} {'commit':>9} {'recorded':>17} {'dur':>7} {'padding':>9} "
          f"{'share':>6} {'peak p99':>9}")
    padded = []
    for path in files:
        try:
            traj = Trajectory.load(path)
        except Exception as exc:  # noqa: BLE001
            print(f"{path.name:>9}  ⛔ unreadable: {type(exc).__name__}: {exc}")
            continue
        pad = traj.trailing_still_seconds(args.still)
        share = 100.0 * pad / traj.duration if traj.duration else 0.0
        flag = "  ⛔" if pad > PADDING_S else ""
        if pad > PADDING_S:
            padded.append(path.name)
        print(f"{path.name:>9} {str(traj.meta.get('commit', '?')):>9} "
              f"{str(traj.meta.get('recorded_at', '?'))[:16]:>17} "
              f"{traj.duration:6.2f}s {pad:8.2f}s {share:5.1f}% "
              f"{traj.joint_speed(99):8.2f}{flag}")

    print()
    if padded:
        print(f"⛔ {len(padded)} of {len(files)} carry more than {PADDING_S:.1f}s of "
              f"padding: {', '.join(padded)}")
        print("   That is the FINDINGS §30.1 defect. Re-record those; it takes seconds.")
    else:
        print(f"✓ none of the {len(files)} files carry more than {PADDING_S:.1f}s of "
              f"trailing still time.")
    print("\n⚠️ Padding is measured at the END only, so a pause in the middle is invisible,")
    print("   and a deliberate pause at the end reads the same as padding. Tenths of a")
    print("   second are the arm coming to rest, not the defect.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
