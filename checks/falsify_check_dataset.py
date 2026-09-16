#!/usr/bin/env python3
"""Feed `check_dataset.py` deliberately broken episodes and count what it catches.

    uv run checks/falsify_check_dataset.py
    uv run checks/falsify_check_dataset.py --source <episode_dir>

By default, encode a temporary synthetic episode and destroy only its disposable
copies. A fresh clone needs ffmpeg/ffprobe, but no user recordings or export.
An explicit source is checked first and is never mutated.

⭐⭐ WHY EVERY CHECKER IN THIS REPO HAS ONE OF THESE ([FINDINGS §70.8](../docs/FINDINGS.md)): a green checker and a *blind* checker look identical from the outside, and this repo has caught three checkers that had silently stopped validating anything. A green run plus a stable catch-count is evidence. A green run alone is not.

⛔ THE BREAKS BELOW ARE THE REAL FAILURE MODES, not invented ones. Each is a plausible mistake in an encoding pipeline, and each produces a file that plays perfectly and trains wrongly: B-frames left on (the ffmpeg default), scene-cut keyframe placement (also the default), the wrong timescale, a truncated table, a metadata shape that no longer matches the bytes, and an action column shifted by one tick. If `check_dataset.py` ever stops catching one of these, the count drops and this script fails.
"""

from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "checks"))

from check_dataset import check_episode  # noqa: E402
from yam.dataset import META_FILE, STATES_FILE, VIDEO_FILE, ffmpeg_command  # noqa: E402
from yam.episode import ROW_COLUMNS  # noqa: E402


def reencode_badly(src: Path, dst: Path, extra: list[str]) -> None:
    """Re-encode the good video with a deliberately wrong option set."""
    subprocess.run(["ffmpeg", "-hide_banner", "-loglevel", "error", "-y", "-i", str(src),
                    *extra, str(dst)], check=True)


def synthetic_episode(root: Path) -> Path:
    """A nonconstant table and real encoded video, wholly owned by a temporary run.

    Use three GOPs so timing/keyframe mutations are observable. These are synthetic
    checker inputs, not a demonstration or evidence that a training loader accepts it.
    """
    import numpy as np

    source = root / "episode_synthetic"
    source.mkdir()
    steps, width, height = 90, 32, 32
    state = np.arange(steps * 14, dtype=np.float64).reshape(steps, 14) / 1000
    actions = np.vstack([state[1:], state[-1:]])
    (source / STATES_FILE).write_bytes(np.hstack([state, actions]).tobytes())
    frames = np.empty((steps, height, width, 3), dtype=np.uint8)
    for tick in range(steps):
        frames[tick] = (tick * 2, 80, 160)
        frames[tick, :, tick % width, :] = 255
    subprocess.run(ffmpeg_command(source / VIDEO_FILE, width, height),
                   input=frames.tobytes(), check=True)
    meta = {"fps": 30, "num_steps": steps, "simulated": True,
            "verified_against_abc_loader": False,
            "states_actions": {"shape": [steps, 28], "columns": list(ROW_COLUMNS)},
            "video": {"frame_width": width, "frame_height": height}}
    (source / META_FILE).write_text(json.dumps(meta))
    return source


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--source", type=Path,
                        help="falsify copies of this exported episode (default: temporary synthetic input)")
    args = parser.parse_args()
    if args.source is not None:
        return falsify(args.source)
    with tempfile.TemporaryDirectory(prefix="yam-dataset-falsifier-") as tmp:
        return falsify(synthetic_episode(Path(tmp)))


def falsify(source: Path) -> int:
    ok, bad = check_episode(source)
    if bad:
        print(f"⛔ the SOURCE episode {source.name} already fails {len(bad)} check(s); "
              "falsification needs a known-good starting point.")
        for line in bad:
            print(f"    ⛔ {line}")
        print("CATCHES: 0/5")
        return 1
    print(f"source: {source.name} — {len(ok)} checks pass on it\n")

    breaks: list[tuple[str, callable, list[str]]] = [
        # (what was broken, how, which failure texts must appear)
        ("B-frames re-enabled and GOP left to the encoder's default",
         lambda d: reencode_badly(source / VIDEO_FILE, d / VIDEO_FILE,
                                  ["-c:v", "libx264", "-preset", "ultrafast", "-bf", "2",
                                   "-g", "250", "-pix_fmt", "yuv420p",
                                   "-video_track_timescale", "15360"]),
         ["B-frame"]),
        ("the wrong timescale, so PTS is no longer 512·k",
         lambda d: reencode_badly(source / VIDEO_FILE, d / VIDEO_FILE,
                                  ["-c:v", "libx264", "-preset", "ultrafast", "-bf", "0",
                                   "-g", "30", "-pix_fmt", "yuv420p",
                                   "-video_track_timescale", "1000"]),
         ["timebase", "PTS"]),
        ("a truncated states_actions.bin",
         lambda d: (d / STATES_FILE).write_bytes(
             (source / STATES_FILE).read_bytes()[:-8 * 28]),
         ["bytes"]),
        ("metadata claiming a shape the bytes do not have",
         lambda d: _rewrite_meta(source, d, {"num_steps": 999}),
         ["bytes", "frames"]),
        ("the action columns shifted, so the action policy no longer holds",
         lambda d: _shift_actions(source, d),
         ["action row k"]),
    ]

    caught = 0
    for label, apply, must_mention in breaks:
        with tempfile.TemporaryDirectory() as tmp:
            broken = Path(tmp) / source.name
            shutil.copytree(source, broken)
            apply(broken)
            _, failures = check_episode(broken)
            text = " | ".join(failures)
            hit = bool(failures) and all(m.lower() in text.lower() for m in must_mention)
            caught += hit
            mark = "✓ caught" if hit else "⛔ MISSED"
            print(f"  {mark}: {label}")
            for line in failures[:3]:
                print(f"           → {line[:120]}")
            if not hit and failures:
                print(f"           ⚠️ expected the report to mention {must_mention}")

    print(f"\n{caught}/{len(breaks)} breaks caught")
    if caught != len(breaks):
        print("⛔ check_dataset.py has gone partly blind. A checker that cannot see a break "
              "is worse\n   than no checker, because it is believed.")
        print(f"CATCHES: {caught}/{len(breaks)}")
        return 1
    print("✓ the checker sees every break. Its green runs mean something.")
    print(f"CATCHES: {caught}/{len(breaks)}")
    return 0


def _rewrite_meta(source: Path, dest: Path, changes: dict) -> None:
    meta = json.loads((source / META_FILE).read_text())
    meta.update(changes)
    (dest / META_FILE).write_text(json.dumps(meta, indent=2))


def _shift_actions(source: Path, dest: Path) -> None:
    import numpy as np

    meta = json.loads((source / META_FILE).read_text())
    steps = int(meta["num_steps"])
    table = np.frombuffer((source / STATES_FILE).read_bytes(),
                          dtype=np.float64).reshape(steps, 28).copy()
    table[:, 14:] = np.roll(table[:, 14:], 3, axis=0)   # actions no longer the next state
    (dest / STATES_FILE).write_bytes(table.tobytes(order="C"))


if __name__ == "__main__":
    sys.exit(main())
