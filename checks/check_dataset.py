#!/usr/bin/env python3
"""Read an exported training episode BACK and check every contract property, numerically.

    uv run checks/check_dataset.py                       # every episode under recordings/datasets/
    uv run checks/check_dataset.py --dir <episode_dir>   # just this one

⭐⭐ WHY THIS IS THE INTERESTING HALF OF THE C4 WORK. The guide's advice is *"do not encode it yourself"*, because the training loader does not decode the video to find frame k — **it computes where frame k is**, from the timebase and the frame rate. Every property that makes that arithmetic true is invisible to the eye and fatal to the dataset: one B-frame, one scene-cut keyframe, one wrong timescale, and the loader reads the wrong image for the right row, or runs ~70× slower. A file like that opens fine, plays fine, and trains wrongly.

So this checker asserts the CONSEQUENCE rather than the intention ([FINDINGS §0](../docs/FINDINGS.md)): it asks `ffprobe` what actually landed in the file, frame by frame, and compares it against the spec — PTS exactly 512·k, keyframes exactly on k%30, no B-frames anywhere, `yuv420p`, timebase 1/15360, the frame count equal to the table's row count, `moov` before `mdat`, and the `.bin` exactly `num_steps × 28 × 8` bytes of finite float64.

⛔ What a green run means: **this directory matches the PUBLISHED spec.** What it does not mean: that ABC's loader accepts it. That is the C4 gate, and `checks/falsify_check_dataset.py` is what proves this checker can still see a break.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent

from yam.dataset import (  # noqa: E402
    EPISODE_FPS,
    GOP,
    META_FILE,
    PTS_STEP,
    ROW_WIDTH,
    STATES_FILE,
    TIMEBASE,
    VIDEO_FILE,
)

DEFAULT_ROOT = REPO / "recordings" / "datasets"


def ffprobe_json(args: list[str]) -> dict:
    """Run ffprobe and parse its JSON. Raises with the tool's own words on failure."""
    out = subprocess.run(["ffprobe", "-v", "error", "-of", "json", *args],
                         capture_output=True, text=True, check=False)
    if out.returncode != 0:
        raise RuntimeError(f"ffprobe failed: {out.stderr.strip()[:300]}")
    return json.loads(out.stdout or "{}")


def check_episode(path: Path) -> tuple[list[str], list[str]]:
    """`(passes, failures)` for one episode directory. Every string is human-readable."""
    ok: list[str] = []
    bad: list[str] = []

    def want(condition: bool, message: str) -> None:
        (ok if condition else bad).append(message)

    meta_path, video, states = path / META_FILE, path / VIDEO_FILE, path / STATES_FILE
    for f in (meta_path, video, states):
        if not f.is_file():
            bad.append(f"{f.name} is missing — the contract names all three files")
    if bad:
        return ok, bad

    meta = json.loads(meta_path.read_text())
    steps = int(meta["num_steps"])
    want(meta.get("fps") == EPISODE_FPS, f"metadata says {meta.get('fps')} fps, want {EPISODE_FPS}")

    # ---- the numeric table -------------------------------------------------------
    import numpy as np

    expect_bytes = steps * ROW_WIDTH * 8
    got_bytes = states.stat().st_size
    want(got_bytes == expect_bytes,
         f"{STATES_FILE} is {got_bytes} bytes; {steps} steps × {ROW_WIDTH} cols × 8 = "
         f"{expect_bytes}")
    if got_bytes == expect_bytes:
        table = np.frombuffer(states.read_bytes(), dtype=np.float64).reshape(steps, ROW_WIDTH)
        want(bool(np.isfinite(table).all()),
             "every value in the table is finite (no NaN or inf reached the dataset)")
        want(list(meta["states_actions"]["shape"]) == [steps, ROW_WIDTH],
             f"metadata shape {meta['states_actions']['shape']} matches the file")
        want(len(meta["states_actions"]["columns"]) == ROW_WIDTH,
             f"metadata names all {ROW_WIDTH} columns")
        # ⭐ The action policy is checkable, not just documented: action row k must equal
        # state row k+1. If that ever silently changed, every trained policy would be
        # learning a one-tick-shifted target and nothing would raise.
        if steps > 2:
            shifted = np.allclose(table[:-2, 14:], table[1:-1, :14])
            want(bool(shifted),
                 "action row k equals state row k+1 (the documented action policy holds "
                 "in the actual bytes)")

    # ---- the video stream --------------------------------------------------------
    stream = ffprobe_json([
        "-select_streams", "v:0", "-show_entries",
        "stream=codec_name,pix_fmt,width,height,time_base,r_frame_rate,has_b_frames,nb_frames",
        str(video)])["streams"][0]
    want(stream["codec_name"] == "h264", f"codec is {stream['codec_name']}, want h264")
    want(stream["pix_fmt"] == "yuv420p", f"pix_fmt is {stream['pix_fmt']}, want yuv420p")
    want(stream["time_base"] == f"1/{TIMEBASE}",
         f"timebase is {stream['time_base']}, want 1/{TIMEBASE}")
    want(stream["r_frame_rate"] == f"{EPISODE_FPS}/1",
         f"frame rate is {stream['r_frame_rate']}, want {EPISODE_FPS}/1")
    want(int(stream["has_b_frames"]) == 0,
         f"has_b_frames is {stream['has_b_frames']} — B-frames reorder PTS against decode "
         "order and break the analytic frame index")
    want(int(stream["width"]) == int(meta["video"]["frame_width"])
         and int(stream["height"]) == int(meta["video"]["frame_height"]),
         f"video is {stream['width']}x{stream['height']}, metadata says "
         f"{meta['video']['frame_width']}x{meta['video']['frame_height']}")

    # ---- frame by frame: the arithmetic the loader depends on ---------------------
    frames = ffprobe_json(["-select_streams", "v:0", "-show_frames",
                           "-show_entries", "frame=pts,key_frame,pict_type",
                           str(video)]).get("frames", [])
    want(len(frames) == steps,
         f"the video holds {len(frames)} frames and the table {steps} rows — one image per "
         "row is the whole point")
    bad_pts = [(i, f.get("pts")) for i, f in enumerate(frames)
               if int(f.get("pts", -1)) != i * PTS_STEP]
    want(not bad_pts,
         f"every frame's PTS is exactly {PTS_STEP}·k"
         + (f" (first wrong: frame {bad_pts[0][0]} has pts {bad_pts[0][1]}, "
            f"want {bad_pts[0][0] * PTS_STEP})" if bad_pts else ""))
    wrong_keys = [i for i, f in enumerate(frames)
                  if bool(int(f.get("key_frame", 0))) != (i % GOP == 0)]
    want(not wrong_keys,
         f"keyframes land exactly on k%{GOP}==0"
         + (f" (first wrong: frame {wrong_keys[0]})" if wrong_keys else ""))
    b_frames = [i for i, f in enumerate(frames) if f.get("pict_type") == "B"]
    want(not b_frames, "no frame is a B-frame"
         + (f" (found {len(b_frames)})" if b_frames else ""))

    # ---- faststart: moov must precede mdat, or the loader seeks twice -------------
    head = video.read_bytes()[:400_000]
    moov, mdat = head.find(b"moov"), head.find(b"mdat")
    want(moov != -1 and (mdat == -1 or moov < mdat),
         "the moov atom precedes mdat (+faststart), so the index is readable without a seek")
    return ok, bad


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--dir", default="", help="one episode directory (default: scan them all)")
    ap.add_argument("--quiet", action="store_true", help="only print failures and the total")
    args = ap.parse_args()

    if args.dir:
        episodes = [Path(args.dir)]
    else:
        episodes = sorted(p for p in DEFAULT_ROOT.glob("episode_*") if p.is_dir())
    if not episodes:
        print(f"no episode directories under {DEFAULT_ROOT.relative_to(REPO)}.")
        print("  Export one:  uv run apps/export_dataset.py --slot 5 --left G --right B \\")
        print("                 --top c920 --left-wrist d405-260323072846 \\")
        print("                 --right-wrist d405-255323071773")
        return 0

    total_ok = total_bad = 0
    for path in episodes:
        try:
            ok, bad = check_episode(path)
        except Exception as exc:  # noqa: BLE001
            print(f"⛔ {path.name}: {type(exc).__name__}: {exc}")
            total_bad += 1
            continue
        total_ok += len(ok)
        total_bad += len(bad)
        mark = "✓" if not bad else "⛔"
        print(f"\n{mark} {path.name}  ({len(ok)} checks passed, {len(bad)} failed)")
        if not args.quiet:
            for line in ok:
                print(f"    ✓ {line}")
        for line in bad:
            print(f"    ⛔ {line}")

    print(f"\n{total_ok} check(s) passed, {total_bad} failed across {len(episodes)} episode(s)")
    print("⚠️ A green run means this matches the PUBLISHED C4 spec. Only ABC's own loader can "
          "say\n   whether the published spec is complete — that gate is still open.")
    return 1 if total_bad else 0


if __name__ == "__main__":
    sys.exit(main())
