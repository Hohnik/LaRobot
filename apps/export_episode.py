#!/usr/bin/env python3
"""Export a saved recording as one MCAP episode in the team's contracted shape.

    uv run apps/export_episode.py --slot 7 --left B --right G
    uv run apps/export_episode.py --slot 7 --left B --right G --top c920 --left-wrist d405-255323071773 --right-wrist d405-260323072846

⭐ The contract is [Setup-Anleitung.md](../docs/Setup-Anleitung.md) C3 (topics, dimensions, the 33,333,333 ns tick, joint-space actions); the reasoning and the honesty limits are in `yam/episode.py`'s docstring — including why the ACTION is the next tick's state for a hand-taught demo, and why this output is "the contract as written" until the Anleitung's own C4 mini-sample gate has run against ABC's loader.

⛔ `--left` and `--right` are REQUIRED: the sides are physical bench positions nothing in a recording can derive, and a defaulted wrong side would mirror every episode silently — dataset poison that raises nothing. Moves nothing, so there is no `--yes`.

⛔ The camera roles work the same way (item 48): a recording that carries frames refuses to export until `--top`/`--left-wrist`/`--right-wrist` name which recorded camera stood where — the names are the ones the session printed and `check_recordings.py` shows. A frameless recording refuses the flags instead.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent

from yam.can import ARM_SERIALS  # noqa: E402 — the real arm names, so a rename cannot go stale here
from yam.episode import export_episode  # noqa: E402
from yam.recording import Trajectory  # noqa: E402


def find_slot(slot: str) -> Path:
    """The saved recording for `slot`, preferring a real one over a simulated one.

    ⛔ Simulated recordings live apart and are stamped `simulated: true` for exactly this
    moment (FINDINGS §60.2): a sim take must never become training data by accident. If
    only the sim copy exists it is used, and the episode metadata carries the stamp.
    """
    real = REPO / "recordings" / f"{slot}.json"
    sim = REPO / "recordings" / "sim" / f"{slot}.json"
    if real.is_file():
        if sim.is_file():
            print(f"  ⚠️ slot {slot} exists both real and simulated — exporting the REAL one.")
        return real
    if sim.is_file():
        print(f"  ⚠️ slot {slot} is a SIMULATED recording — fine for pipeline tests, "
              "never for training.")
        return sim
    raise SystemExit(f"⛔ nothing saved in slot {slot} (checked recordings/ and recordings/sim/).")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[1].strip())
    ap.add_argument("--slot", required=True, help="recording slot 0-9 to export")
    ap.add_argument("--left", required=True, choices=sorted(ARM_SERIALS),
                    help="which arm stands on the bench's LEFT — required, never guessed")
    ap.add_argument("--right", required=True, choices=sorted(ARM_SERIALS),
                    help="which arm stands on the bench's RIGHT")
    ap.add_argument("--out", default="",
                    help="output path (default: recordings/episodes/<slot>.mcap)")
    ap.add_argument("--top", default=None, metavar="CAMERA",
                    help="which recorded camera looks from the TOP (name as recorded)")
    ap.add_argument("--left-wrist", default=None, metavar="CAMERA",
                    help="which recorded camera rides the LEFT wrist")
    ap.add_argument("--right-wrist", default=None, metavar="CAMERA",
                    help="which recorded camera rides the RIGHT wrist")
    args = ap.parse_args()
    if args.left == args.right:
        raise SystemExit("⛔ left and right name the same arm — one of them is wrong.")
    cameras = {role: value for role, value in
               (("top", args.top), ("left-wrist", args.left_wrist),
                ("right-wrist", args.right_wrist)) if value}

    path = find_slot(args.slot)
    traj = Trajectory.load(path)
    out = Path(args.out) if args.out else REPO / "recordings" / "episodes" / f"{args.slot}.mcap"
    try:
        report = export_episode(traj, left=args.left, right=args.right,
                                out_path=out, source=str(path.relative_to(REPO)),
                                cameras=cameras or None, recording_path=path)
    except ValueError as e:
        raise SystemExit(f"⛔ {e}") from e

    print(f"\n⭐ EPISODE written: {report.path.relative_to(REPO)}")
    print(f"   {report.ticks} ticks at 30 Hz covering {report.duration_s:.1f}s · "
          f"left={report.mapping['left']} right={report.mapping['right']} · "
          f"{report.bad_spans} bad-labelled span(s) in the metadata")
    if report.cameras:
        print("   📷 " + " · ".join(f"{role}: {name}"
                                    for role, name in report.cameras.items()))
    for w in report.warnings:
        print(f"   ⚠️ {w}")
    print("   ⛔ The Anleitung's C4 gate still applies: verify a mini-sample against "
          "ABC's loader before collecting for real.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
