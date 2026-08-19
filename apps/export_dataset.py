#!/usr/bin/env python3
"""Export a saved recording as one training-episode DIRECTORY, in the C4 shape.

    uv run apps/export_dataset.py --slot 5 --left G --right B \
        --top c920 --left-wrist d405-260323072846 --right-wrist d405-255323071773

⭐ Two exports exist and they are different things. `apps/export_episode.py` writes the C3 LOG (one MCAP file, every stream on the 33,333,333 ns tick). **This writes the C4 TRAINING SET**: `episode_<id>/` with the flat `(num_steps, 28)` float64 table, the camera views stacked into one strictly-encoded video, and a metadata file. The reasoning, the encoding spec and the one named ambiguity are in `yam/dataset.py`'s docstring.

⛔ `--left`/`--right` and the camera roles are REQUIRED, for the same reason in both exports: bench positions are physical facts no recording can derive, and a silently wrong side mirrors an entire dataset without raising ([FINDINGS §70.13](../docs/FINDINGS.md)).

⚠️ Verify what comes out, every time: `uv run checks/check_dataset.py`. It re-reads the video frame by frame with ffprobe and checks the properties the training loader silently depends on. Moves nothing, so there is no `--yes`.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent

from yam.can import ARM_SERIALS  # noqa: E402
from yam.dataset import VIEW_SIZE, export_dataset  # noqa: E402
from yam.recording import Trajectory  # noqa: E402

sys.path.insert(0, str(REPO / "apps"))
from export_episode import find_slot  # noqa: E402 — one slot-finder, both exports


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[1].strip())
    ap.add_argument("--slot", required=True, help="recording slot 0-9 to export")
    ap.add_argument("--left", required=True, choices=sorted(ARM_SERIALS),
                    help="which arm stands on the bench's LEFT — required, never guessed")
    ap.add_argument("--right", required=True, choices=sorted(ARM_SERIALS),
                    help="which arm stands on the bench's RIGHT")
    ap.add_argument("--top", default=None, metavar="CAMERA",
                    help="which recorded camera looks from the TOP (name as recorded)")
    ap.add_argument("--left-wrist", default=None, metavar="CAMERA",
                    help="which recorded camera rides the LEFT wrist")
    ap.add_argument("--right-wrist", default=None, metavar="CAMERA",
                    help="which recorded camera rides the RIGHT wrist")
    ap.add_argument("--out", default="",
                    help="output root (default: recordings/datasets/)")
    ap.add_argument("--view-width", type=int, default=VIEW_SIZE,
                    help="per-view width in the stacked video (default %(default)s)")
    ap.add_argument("--view-height", type=int, default=VIEW_SIZE,
                    help="per-view height (default %(default)s — see the ambiguity note in "
                         "yam/dataset.py: the guide says 224 square, the team's sim renders 168)")
    ap.add_argument("--episode-id", default="",
                    help="name the episode instead of generating an id (for reproducible runs)")
    args = ap.parse_args()
    if args.left == args.right:
        raise SystemExit("⛔ left and right name the same arm — one of them is wrong.")
    cameras = {role: value for role, value in
               (("top", args.top), ("left-wrist", args.left_wrist),
                ("right-wrist", args.right_wrist)) if value}

    path = find_slot(args.slot)
    traj = Trajectory.load(path)
    out_root = Path(args.out) if args.out else REPO / "recordings" / "datasets"
    try:
        report = export_dataset(traj, left=args.left, right=args.right, cameras=cameras,
                                recording_path=path, out_root=out_root,
                                view_width=args.view_width, view_height=args.view_height,
                                episode_id=args.episode_id or None,
                                source=str(path.relative_to(REPO)))
    except ValueError as e:
        raise SystemExit(f"⛔ {e}") from e

    print(f"\n⭐ TRAINING EPISODE written: {report.path.relative_to(REPO)}")
    print(f"   {report.steps} steps at 30 Hz covering {report.duration_s:.1f}s · "
          f"left={report.mapping['left']} right={report.mapping['right']}")
    print(f"   📹 {len(report.roles)} view(s) stacked in order {list(report.roles)} · "
          f"each {report.view_size[0]}x{report.view_size[1]} · "
          f"frame {report.frame_size[0]}x{report.frame_size[1]}")
    for w in report.warnings:
        print(f"   ⚠️ {w}")
    print("\n⭐ VERIFY IT NOW — the properties the loader depends on are invisible to the eye:")
    print(f"   uv run checks/check_dataset.py --dir {report.path.relative_to(REPO)}")
    print("   ⛔ And the Anleitung's C4 gate still applies: only ABC's own loader can confirm "
          "the\n      published spec is complete.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
