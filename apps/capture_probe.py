#!/usr/bin/env python3
"""Measure what the attached cameras actually deliver, sampled the way the control loop will.

    uv run apps/capture_probe.py --cameras d405 c920 --seconds 10
    uv run apps/capture_probe.py --indices 0 1 2 --seconds 10 --save

⚠️ `--indices 0 1 2` and `--indices 0,1,2` both parse. They did NOT always: the flag took one comma-joined string, the his-list said only "the three indices", and the 2026-08-19 bench pass lost its measurement to three refused spellings in a row (FINDINGS §71.1). Accepting every natural spelling is the fix.

⭐ WHY THIS EXISTS (ROADMAP §8.2 item 6): a dataset needs frames that line up with joint data, and the number that decides whether that is possible is not in any spec sheet — it is what each camera ACTUALLY delivers while the others are streaming on the same USB tree. Bandwidth exhaustion shows up as a low frame rate and long blind gaps, never as an error (FINDINGS §34.5), so it has to be measured, not assumed.

It opens the named cameras, reads each in its own thread (the same `FrameGrabber` the confirmed viewer uses), samples them together at the control loop's rate, and reports per camera: achieved capture fps · fresh-sample ratio · mean and worst gap between frames · the delivered pixel format and size.

⛔ Moves nothing and commands nothing — there is no `--yes` because there is nothing to consent to. ⛔ An agent can never run this (macOS grants camera access per app, FINDINGS §61.3); it is written headless-tested (`tests/test_capture.py`) and handed over.

⚠️ Identification: `--cameras d405` resolves by MEASUREMENT (the confirmed `camera_view` machinery). With TWO D405s attached the measurement cannot tell them apart (FINDINGS §67.12) — use `--indices` and establish which is which once by covering a lens, then keep the mapping.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent

from camera_view import CameraLookupError, open_camera, resolve_camera  # noqa: E402
from yam.cameras.capture import CaptureSet  # noqa: E402
from yam.cameras.grabber import FrameGrabber  # noqa: E402
from yam.cameras.specs import flatten_tokens, parse_indices  # noqa: E402
from yam.provenance import git_commit  # noqa: E402

SAVE_DIR = REPO / "recordings" / "cameras"


def open_named(args) -> dict[str, tuple[int, object]]:  # noqa: ANN001
    """name → (index, configured capture), refusing loudly rather than guessing."""
    out: dict[str, tuple[int, object]] = {}
    if args.indices:
        try:
            indices = parse_indices(args.indices)
        except ValueError as e:
            raise SystemExit(f"⛔ {e}") from e
        for idx in indices:
            cap = open_camera(idx, args.width, args.height, args.fps)
            if cap is None:
                raise SystemExit(f"⛔ index {idx} would not open. `uv run apps/camera_view.py --list` shows what is there.")
            out[f"cam{idx}"] = (idx, cap)
        return out
    for spec in flatten_tokens(args.cameras):
        try:
            idx, cam, found_cap = resolve_camera(spec)
        except CameraLookupError as e:
            raise SystemExit(f"⛔ {e}") from e
        # ⛔ The resolver may hand the device back ALREADY OPEN (find_camera_index keeps it open to save the caller a multi-second reopen). Discarding that handle without releasing it leaks the device, and the configured reopen below then finds it busy — release first, reopen with the asked-for mode.
        if found_cap is not None:
            found_cap.release()
        cap = open_camera(idx, args.width, args.height, args.fps)
        if cap is None:
            raise SystemExit(f"⛔ {spec} resolved to index {idx} and would not open.")
        out[spec] = (idx, cap)
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[1].strip())
    ap.add_argument("--cameras", nargs="*", default=["d405"],
                    help="camera names to resolve by measurement, space- or comma-separated "
                         "(default: d405)")
    ap.add_argument("--indices", nargs="*", default=[],
                    help="raw OpenCV indices instead, space- or comma-separated — the only "
                         "way to open two D405s knowingly")
    ap.add_argument("--seconds", type=float, default=10.0, help="how long to sample (default 10)")
    ap.add_argument("--hz", type=float, default=90.0,
                    help="sampling rate, default 90 — the control loop's own rate")
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--fps", type=int, default=30, help="the rate to ASK each camera for")
    ap.add_argument("--save", action="store_true",
                    help=f"write a JSON report and one PNG per camera under {SAVE_DIR.relative_to(REPO)}/")
    args = ap.parse_args()

    named = open_named(args)
    grabbers = {name: FrameGrabber(cap) for name, (idx, cap) in named.items()}
    print(f"sampling {', '.join(named)} at {args.hz:g} Hz for {args.seconds:g}s "
          f"(asked each for {args.width}x{args.height}@{args.fps}) …")

    capture = CaptureSet(grabbers)
    last: dict[str, object] = {}
    period = 1.0 / args.hz
    t_end = time.perf_counter() + args.seconds
    try:
        while time.perf_counter() < t_end:
            t_next = time.perf_counter() + period
            for name, frame in capture.sample().items():
                if frame is not None:
                    last[name] = frame
            wait = t_next - time.perf_counter()
            if wait > 0:
                time.sleep(wait)

        print()
        results = []
        for r in capture.report():
            grab = grabbers[r.name]
            shape = getattr(last.get(r.name), "rgb", None)
            size = f"{shape.shape[1]}x{shape.shape[0]}" if shape is not None and hasattr(shape, "shape") else "?"
            fresh_pct = 100.0 * r.fresh / r.samples if r.samples else 0.0
            print(f"  {r.name:>8}  {size:>9} {grab.pixel_format():>4} · "
                  f"{r.capture_fps:5.1f} fps captured · fresh {fresh_pct:4.1f}% of samples · "
                  f"gap mean {r.mean_gap_ms:5.1f} ms, worst {r.worst_gap_ms:6.1f} ms"
                  + (f" · ⚠️ never delivered a frame" if r.fresh == 0 else ""))
            results.append({"name": r.name, "size": size, "format": grab.pixel_format(),
                            "capture_fps": round(r.capture_fps, 2), "samples": r.samples,
                            "fresh": r.fresh, "duplicates": r.duplicates, "empty": r.empty,
                            "mean_gap_ms": round(r.mean_gap_ms, 2),
                            "worst_gap_ms": round(r.worst_gap_ms, 2)})
            if r.capture_fps < 0.8 * args.fps and r.fresh > 0:
                print(f"           ⚠️ well under the {args.fps} fps asked for — with several "
                      "cameras on one USB tree this is what bandwidth exhaustion looks "
                      "like: dropped frames, no error (FINDINGS §34.5).")

        if args.save:
            import cv2  # noqa: PLC0415

            SAVE_DIR.mkdir(parents=True, exist_ok=True)
            stamp = time.strftime("%Y-%m-%d_%H%M%S")
            for name, frame in last.items():
                cv2.imwrite(str(SAVE_DIR / f"{stamp}_{name}.png"), frame.rgb)
            report_path = SAVE_DIR / f"{stamp}_report.json"
            report_path.write_text(json.dumps({
                "when": stamp, "commit": git_commit(),
                "asked": {"width": args.width, "height": args.height, "fps": args.fps,
                          "sample_hz": args.hz, "seconds": args.seconds},
                "cameras": results}, indent=2))
            print(f"\n  saved frames + {report_path.relative_to(REPO)}")
    finally:
        capture.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
