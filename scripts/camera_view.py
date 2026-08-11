#!/usr/bin/env python3
"""⭐ Live wrist-camera view, for driving the arm from the camera's point of view.

    uv run scripts/camera_view.py --list          # which camera is which index
    uv run scripts/camera_view.py                 # live view, lowest latency
    uv run scripts/camera_view.py --index 1 --big # full-screen-ish

⛔ THIS TOUCHES NO ROBOT. It opens no CAN bus, imports nothing from `yam_can` or
`yam_robot`, and cannot move a motor. Run it alongside `teleop_session.py` in a
second terminal.

WHY A SEPARATE PROCESS, AND NOT A WINDOW INSIDE THE SESSION
------------------------------------------------------------
Three independent reasons, and they all point the same way:

1. **Latency.** Julien's requirement is *"real time, no latency type of setup."*
   Decoding 1080p costs milliseconds; doing it inside the 100 Hz control loop
   spends the arm's cycle budget on pixels. The measured CAN budget (~6.2 ms of a
   10 ms deadline) was taken with **nothing else competing for CPU**.
2. **macOS demands the main thread for GUI.** `cv2.imshow` from a worker thread is
   unreliable here — the same constraint that makes MuJoCo's viewer need
   `mjpython`. A process whose main thread is the window has no such problem.
3. **It survives the bimanual refactor untouched**, because it shares nothing with
   the session.

⚠️ **OpenCV on macOS selects cameras by INDEX, not by name**, and this repo has a
hard-won rule against selecting hardware by index (FINDINGS §0 #5: an adapter
chosen by index silently retargeted the wrong robot). AVFoundation gives no way
around it, so instead of pretending, `--list` makes the ambiguity visible: it opens
each index and reports what answered. Check it after any replug.

LATENCY, AND WHAT ACTUALLY CAUSES IT
-------------------------------------
Almost all webcam "lag" is **queued frames**, not decode time: the driver buffers,
and a naive `read()` hands you the oldest frame in the queue. Two fixes are applied:

- **MJPG** rather than the default YUY2. The C920 does 1080p30 compressed; uncompressed
  it falls to ~5 fps over USB2 bandwidth, which *looks* like latency and is not.
- **Drain-then-retrieve**: `grab()` is cheap (no decode), `retrieve()` is not. Grabbing
  until the queue is empty and decoding only the last frame shows the newest image
  rather than the oldest.

`--measure` reports the real frame interval so the claim is checked, not asserted.
"""

from __future__ import annotations

import argparse
import sys
import time

import cv2
import numpy as np

MAX_PROBE_INDEX = 6


def list_cameras() -> None:
    """Open each index in turn and report what answers.

    ⚠️ Indices are an OpenCV/AVFoundation artefact and **can change on replug**.
    This is deliberately a report rather than an auto-pick: the operator can tell
    the built-in FaceTime camera from a C920 by looking at the picture, and no
    property reliably does it for them.
    """
    print("probing camera indices — this opens each one briefly\n")
    found = 0
    for idx in range(MAX_PROBE_INDEX):
        cap = cv2.VideoCapture(idx)
        if not cap.isOpened():
            cap.release()
            continue
        ok, frame = cap.read()
        w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = cap.get(cv2.CAP_PROP_FPS)
        mean = float(np.mean(frame)) if ok and frame is not None else float("nan")
        print(f"  index {idx}: {w}x{h} @ {fps:.0f} fps   "
              f"{'frame OK' if ok else 'NO FRAME'}   mean brightness {mean:.0f}")
        cap.release()
        found += 1
    if not found:
        print("  none opened. On macOS the FIRST run must be granted camera access —")
        print("  look for the permission dialog, then run this again.")
        return
    print("\n  Which is which: cover the wrist camera with your hand and re-run —")
    print("  the index whose mean brightness collapses is the one on the arm.")


def probe_modes(index: int, secs: float = 2.5) -> None:
    """Measure the REAL frame rate at each resolution, with and without MJPG.

    ⚠️ This exists because the agent cannot run it. macOS grants camera access
    **per application**, and the permission Julien granted covers his terminal, not
    the process the agent's shell runs under — so every agent-side attempt returns
    `not authorized to capture video` no matter what the code does. The measurement
    therefore has to be a command he runs, which is what this is.
    """
    print(f"sweeping camera {index} — real fps, {secs:.0f}s per mode\n")
    print("%-24s %-12s %-7s %s" % ("requested", "actual", "codec", "measured fps"))
    for w, h in ((1920, 1080), (1280, 720), (960, 540), (640, 480), (424, 240)):
        for cc in ("MJPG", None):
            cap = cv2.VideoCapture(index)
            if not cap.isOpened():
                print("  could not open — is camera access granted to this terminal?")
                return
            if cc:
                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*cc))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
            cap.set(cv2.CAP_PROP_FPS, 30)
            got = int(cap.get(cv2.CAP_PROP_FOURCC))
            got_s = "".join(chr((got >> (8 * i)) & 0xFF) for i in range(4)).strip() or "?"
            aw = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            ah = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            for _ in range(5):
                cap.read()
            t0 = time.perf_counter()
            n = 0
            while time.perf_counter() - t0 < secs:
                if cap.read()[0]:
                    n += 1
            dur = time.perf_counter() - t0
            cap.release()
            print("%-24s %-12s %-7s %.1f" % (f"{w}x{h} {cc or 'as-is'}", f"{aw}x{ah}", got_s, n / dur))
    print("\n  Pick the largest size that still gives ~30 fps and pass it as")
    print("  --width/--height. If MJPG changes nothing, macOS is ignoring the codec")
    print("  request and resolution is your only lever — which is why the default is 640x480.")


def open_camera(index: int, width: int, height: int, fps: int):  # noqa: ANN201
    cap = cv2.VideoCapture(index)
    if not cap.isOpened():
        return None
    # ⭐ MJPG first, THEN the resolution. Setting size before the codec leaves the
    # C920 in uncompressed YUY2, where 1080p does not fit in the USB bandwidth and
    # drops to a few fps — which reads as "lag" but is a bandwidth problem.
    # ⚠️ UNVERIFIED ON THIS MACHINE: OpenCV's macOS (AVFoundation) backend is widely
    # reported to IGNORE CAP_PROP_FOURCC, unlike Linux V4L2 and Windows DSHOW. If it
    # does, this line is a no-op here and resolution is the only lever that works.
    # `--probe` settles it: if the MJPG and as-is rows report the same fps, it is
    # being ignored. Either way the 640x480 default is safe, because lowering the
    # resolution cuts bandwidth under both explanations.
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)
    try:
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)   # ignored by some backends; harmless
    except Exception:  # noqa: BLE001, S110
        pass
    return cap


def newest_frame(cap):  # noqa: ANN001, ANN201
    """Drain the queue and decode only the most recent frame.

    `grab()` pulls a frame off the driver queue without decoding it; `retrieve()`
    decodes. Grabbing repeatedly until the queue runs dry and decoding once is what
    turns "several frames behind" into "current".
    """
    ok = cap.grab()
    if not ok:
        return None
    for _ in range(4):                     # bounded: never spin on a fast producer
        if not cap.grab():
            break
    ok, frame = cap.retrieve()
    return frame if ok else None


def main() -> int:
    ap = argparse.ArgumentParser(description="Live camera view. Touches no robot.")
    ap.add_argument("--list", action="store_true", help="probe every index and report")
    ap.add_argument("--probe", action="store_true",
                    help="sweep resolutions and codecs on --index and report the REAL fps "
                         "for each. Run this once to find the best setting for your link")
    ap.add_argument("--index", type=int, default=0, help="camera index (see --list)")
    # ⛔ 640x480 BY DEFAULT, and the reason is arithmetic rather than taste.
    # USB 2.0 sustains roughly 24 MB/s for video. One UNCOMPRESSED 1920x1080 frame
    # is 1920*1080*2 = 4.15 MB, so the link allows 24/4.15 = 5.8 fps -- and Julien
    # measured exactly 5 fps on 2026-08-11. At 640x480 a frame is 0.61 MB, allowing
    # ~39 fps, so the sensor's own 30 fps cap becomes the limit instead.
    # This fix holds whether or not MJPG compression is being applied, which matters
    # because macOS may be ignoring the codec request entirely (see open_camera).
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--big", action="store_true", help="open the window large")
    ap.add_argument("--flip", action="store_true",
                    help="mirror the image left-right — try this if steering feels inverted")
    ap.add_argument("--rotate", type=int, default=0, choices=[0, 90, 180, 270],
                    help="rotate the view, for a camera mounted sideways on the arm")
    ap.add_argument("--measure", type=float, default=0.0,
                    help="capture headlessly for N seconds and report the real frame "
                         "interval, then exit. No window — safe over SSH")
    args = ap.parse_args()

    if args.list:
        list_cameras()
        return 0

    if args.probe:
        probe_modes(args.index)
        return 0

    cap = open_camera(args.index, args.width, args.height, args.fps)
    if cap is None:
        print(f"⛔ could not open camera index {args.index}. Try:  --list")
        return 1

    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"camera {args.index}: {w}x{h}, requested {args.fps} fps, MJPG")

    if args.measure:
        # Verify the latency claim instead of asserting it.
        t0 = time.perf_counter()
        gaps = []
        last = t0
        frames = 0
        while time.perf_counter() - t0 < args.measure:
            f = newest_frame(cap)
            if f is None:
                continue
            now = time.perf_counter()
            gaps.append(now - last)
            last = now
            frames += 1
        cap.release()
        if not gaps:
            print("⛔ no frames captured at all.")
            return 1
        g = np.array(gaps[1:]) * 1000.0
        print(f"\n  frames        : {frames} in {args.measure:.0f}s "
              f"= {frames / args.measure:.1f} fps")
        print(f"  frame interval: mean {g.mean():.1f} ms  p50 {np.percentile(g, 50):.1f}  "
              f"p95 {np.percentile(g, 95):.1f}  max {g.max():.1f}")
        print("\n  ⚠️ This is the CAPTURE interval, not glass-to-glass latency. It bounds")
        print("     it from below: display and USB transport add more. The honest way to")
        print("     measure the real thing is to point the camera at a running stopwatch")
        print("     on the screen and photograph both.")
        return 0

    win = "wrist camera — q or ESC to quit"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    if args.big:
        cv2.resizeWindow(win, 1600, 900)

    print("\n  q or ESC quits.  f mirrors.  r rotates 90°.")
    print("  ⭐ Drive with `teleop_session.py` in another terminal, and press v there")
    print("     to put the controls in the TOOL frame — then 'forward' on the puck means")
    print("     forward in THIS image, which is the whole point.\n")

    flip, rotate = args.flip, args.rotate
    try:
        while True:
            frame = newest_frame(cap)
            if frame is None:
                continue
            if flip:
                frame = cv2.flip(frame, 1)
            r = {90: cv2.ROTATE_90_CLOCKWISE, 180: cv2.ROTATE_180,
                 270: cv2.ROTATE_90_COUNTERCLOCKWISE}.get(rotate)
            if r is not None:
                frame = cv2.rotate(frame, r)
            cv2.imshow(win, frame)
            k = cv2.waitKey(1) & 0xFF
            if k in (ord("q"), 27):
                break
            if k == ord("f"):
                flip = not flip
                print(f"  mirror {'ON' if flip else 'OFF'}")
            if k == ord("r"):
                rotate = (rotate + 90) % 360
                print(f"  rotate {rotate}°")
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    sys.exit(main())
