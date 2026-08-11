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

LATENCY AND FRAME RATE — WHAT ACTUALLY MATTERED HERE
-----------------------------------------------------
⛔ The first two explanations were both wrong, and the way they were wrong is worth
keeping. The camera was thought to be bandwidth-limited (uncompressed 1080p over
USB 2.0), and separately, stale frames were thought to be queued by the driver. The
fix written for the second — grab repeatedly to drain the queue, decode only the
last — is right on Linux and **backwards on macOS, where `grab()` blocks until the
next frame arrives.** Five grabs per displayed frame at 30 fps is 167 ms, i.e. 6 fps.
That, not bandwidth, is why Julien saw 5 fps.

⚠️ It survived because `--probe` measured with `cap.read()` while the viewer used
the drain loop. The probe reported a healthy 30 fps for code the viewer never ran.
**A measurement that does not exercise the real path measures nothing** — so
`--measure` now runs through `FrameGrabber`, exactly as the viewer does.

The working approach on a blocking backend is to put the blocking where it cannot
hurt: a background thread reads continuously at the camera's own rate and each frame
overwrites the last, so the display loop never waits and always shows the newest
frame. Old frames are dropped by being overwritten rather than by being read.
"""

from __future__ import annotations

import argparse
import shutil
import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np

MAX_PROBE_INDEX = 6

# Live-switchable capture sizes, bound to keys 1..5 in the viewer.
SIZES = [(424, 240), (640, 480), (960, 540), (1280, 720), (1920, 1080)]


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
    # ⚠️ MEASURED 2026-08-11: on this Mac the codec request makes NO difference —
    # --probe reported ~30 fps with and without it at every resolution, and reading
    # CAP_PROP_FOURCC back returns -1 (prints as "ÿÿÿÿ"), i.e. the property is not
    # readable. So AVFoundation is choosing the format itself, and choosing well:
    # 1920x1080 at 30 fps cannot fit down USB 2.0 uncompressed, so it must already
    # be compressing. This line is kept because it is correct and load-bearing on
    # Linux, where this rig is ultimately headed.
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)
    try:
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)   # ignored by some backends; harmless
    except Exception:  # noqa: BLE001, S110
        pass
    return cap


class FrameGrabber:
    """Reads the camera in a background thread and keeps only the newest frame.

    ⛔⭐ THE BUG THIS REPLACES, because it is a good lesson and it was mine.

    The previous version tried to avoid stale frames by "draining the queue":

        cap.grab()                    # cheap, no decode
        for _ in range(4):            # drain whatever else is waiting
            if not cap.grab(): break
        ok, frame = cap.retrieve()    # decode only the newest

    That is correct on Linux/V4L2, where `grab()` returns immediately when no frame
    is waiting. **On macOS/AVFoundation `grab()` BLOCKS until the next frame
    arrives.** So the loop did not drain a queue — it *waited for five more frames*.
    At 30 fps that is 5 x 33 ms = 167 ms per displayed frame, i.e. **6 fps**.
    Julien measured 5.

    ⚠️ And the reason it survived: `--probe` measured with `cap.read()` while the
    viewer used the drain loop, so the probe reported a healthy 30 fps for code the
    viewer never ran. **A measurement that does not exercise the real path measures
    nothing.** `--measure` now runs through this exact class.

    The right way to get "newest frame, no waiting" on a blocking backend is to move
    the blocking somewhere it does not matter: a thread reads continuously at the
    camera's own rate, and each new frame simply overwrites the last. The display
    loop then takes whatever is currently there and never blocks, so it shows the
    most recent frame captured and old frames are dropped by being overwritten
    rather than by being read and thrown away.
    """

    def __init__(self, cap):  # noqa: ANN001
        self._cap = cap
        self._lock = threading.Lock()
        self._frame = None
        self._seq = 0
        self._running = True
        self._captured = 0
        self._t0 = time.perf_counter()
        self._thread = threading.Thread(target=self._run, name="camera", daemon=True)
        self._thread.start()

    def _run(self) -> None:
        while self._running:
            ok, frame = self._cap.read()
            if not ok:
                time.sleep(0.005)
                continue
            with self._lock:
                self._frame = frame
                self._seq += 1
                self._captured += 1

    def newest(self):  # noqa: ANN201
        """The most recent frame and its sequence number. Never blocks."""
        with self._lock:
            return self._frame, self._seq

    def capture_fps(self) -> float:
        dt = time.perf_counter() - self._t0
        return self._captured / dt if dt > 0 else 0.0

    def stop(self) -> None:
        """⚠️ Always call this. A daemon thread holding the camera open keeps the
        device busy for the next process, and Julien asked specifically that every
        test be quittable."""
        self._running = False
        self._thread.join(timeout=1.0)
        self._cap.release()


UPPER_HALF = "\u2580"   # ▀ — foreground paints the top pixel, background the bottom


def render_ansi(frame, cols: int, rows: int) -> str:
    """Turn a camera frame into coloured text that fills a terminal.

    ⭐ WHY THIS EXISTS. Julien, 2026-08-11: *"the order of my open programs on my mac
    constantly gets moved around whenever the cam opens to the desktop as a small
    window, and I have to re-sort the windows manually."* A native window makes the
    Python process a GUI application, so macOS brings it to the front and reshuffles
    his layout every single time. Nothing in OpenCV prevents that.

    Drawing into a terminal sidesteps it completely: he already keeps terminals open
    for the teleop session, so the camera becomes one more pane in a layout he
    already has, and no window is ever created.

    **The half-block trick.** A terminal cell is roughly twice as tall as it is wide,
    which would squash the picture. Printing `▀` with a foreground colour and a
    background colour puts **two** vertically-stacked pixels in one cell — the
    foreground paints the top half, the background the bottom. That doubles the
    vertical resolution and makes each rendered pixel close to square.

    ⚠️ **Colour codes are only emitted when the colour changes.** A full-colour cell
    costs ~40 bytes; a 100x50 render is 5000 cells, so a naive version writes 200 KB
    per frame and 6 MB/s at 30 fps, which the terminal cannot keep up with. Emitting
    a code only on change collapses that dramatically on real images, where
    neighbouring pixels are usually similar.
    """
    small = cv2.resize(frame, (cols, rows * 2), interpolation=cv2.INTER_AREA)
    rgb = cv2.cvtColor(small, cv2.COLOR_BGR2RGB)
    out: list[str] = []
    for y in range(rows):
        top, bot = rgb[2 * y], rgb[2 * y + 1]
        last_fg = last_bg = None
        for x in range(cols):
            fg = (int(top[x][0]), int(top[x][1]), int(top[x][2]))
            bg = (int(bot[x][0]), int(bot[x][1]), int(bot[x][2]))
            if fg != last_fg:
                out.append(f"\x1b[38;2;{fg[0]};{fg[1]};{fg[2]}m")
                last_fg = fg
            if bg != last_bg:
                out.append(f"\x1b[48;2;{bg[0]};{bg[1]};{bg[2]}m")
                last_bg = bg
            out.append(UPPER_HALF)
        out.append("\x1b[0m\n")
    return "".join(out)


def terminal_grid(margin_rows: int = 3) -> tuple[int, int]:
    """Character columns and rows available for the picture."""
    size = shutil.get_terminal_size(fallback=(100, 30))
    return max(20, size.columns), max(10, size.lines - margin_rows)


def run_terminal(cap, args) -> int:  # noqa: ANN001
    """Draw the camera into this terminal. Creates no window at all.

    ⛔ Quitting is guaranteed: `q`/ESC, Ctrl-C, and the `finally` block all restore
    the cursor and colours and stop the grabber thread. Julien asked specifically
    that every test be quittable, and a program that leaves a terminal without a
    cursor is exactly the kind of mess that makes people avoid a tool.
    """
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))
    from keyboard import KeyReader  # noqa: PLC0415

    grab = FrameGrabber(cap)
    flip, rotate = args.flip, args.rotate
    shown, t_fps, disp_fps, prev_seq = 0, time.perf_counter(), 0.0, -1
    sys.stdout.write("\x1b[?25l\x1b[2J")          # hide cursor, clear once
    try:
        with KeyReader() as keys:
            while True:
                frame, seq = grab.newest()
                if frame is None:
                    time.sleep(0.01)
                    continue
                if seq == prev_seq:
                    time.sleep(0.002)
                else:
                    prev_seq = seq
                    shown += 1
                if flip:
                    frame = cv2.flip(frame, 1)
                r = {90: cv2.ROTATE_90_CLOCKWISE, 180: cv2.ROTATE_180,
                     270: cv2.ROTATE_90_COUNTERCLOCKWISE}.get(rotate)
                if r is not None:
                    frame = cv2.rotate(frame, r)

                cols, rows = terminal_grid()
                now = time.perf_counter()
                if now - t_fps >= 0.5:
                    disp_fps = shown / (now - t_fps)
                    shown, t_fps = 0, now
                h, w = frame.shape[:2]
                # \x1b[H homes the cursor instead of clearing, so the picture is
                # overwritten in place. Clearing every frame flickers badly.
                sys.stdout.write("\x1b[H" + render_ansi(frame, cols, rows))
                sys.stdout.write(
                    f"\x1b[0m{w}x{h}  {disp_fps:4.1f} fps shown / {grab.capture_fps():4.1f} captured"
                    f"   q quit · f mirror · r rotate · 1-5 resolution\x1b[K"
                )
                sys.stdout.flush()

                for k in keys.drain():
                    if k in ("q", "\x1b"):
                        return 0
                    if k == "f":
                        flip = not flip
                    elif k == "r":
                        rotate = (rotate + 90) % 360
                    elif k in "12345":
                        w2, h2 = SIZES[int(k) - 1]
                        grab.stop()
                        cap = open_camera(args.index, w2, h2, args.fps)
                        if cap is None:
                            return 1
                        grab = FrameGrabber(cap)
                        shown, t_fps, prev_seq = 0, time.perf_counter(), -1
    except KeyboardInterrupt:
        pass
    finally:
        grab.stop()
        sys.stdout.write("\x1b[0m\x1b[?25h\n")   # colours off, cursor back
        sys.stdout.flush()
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description="Live camera view. Touches no robot.")
    ap.add_argument("--list", action="store_true", help="probe every index and report")
    ap.add_argument("--probe", action="store_true",
                    help="sweep resolutions and codecs on --index and report the REAL fps "
                         "for each. Run this once to find the best setting for your link")
    ap.add_argument("--index", type=int, default=0, help="camera index (see --list)")
    # ⭐ 1280x720 by default. An earlier version defaulted to 640x480 on the theory
    # that USB 2.0 bandwidth capped an uncompressed 1080p stream at ~5.8 fps, which
    # matched the 5 fps Julien saw. **That theory was REFUTED by --probe on
    # 2026-08-11**: the camera delivers ~30 fps at every size up to 1920x1080, so it
    # is compressing and bandwidth was never the constraint. The 5 fps came from the
    # viewer's own frame-draining loop (see FrameGrabber). Resolution is now a free
    # choice, and 720p is a good default; keys 1..5 change it live.
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--term", action="store_true",
                    help="⭐ draw the video IN THIS TERMINAL instead of opening a window. "
                         "No window is created, so your macOS window order is never disturbed")
    ap.add_argument("--big", action="store_true",
                    help="make the WINDOW larger on screen. ⚠️ It does not change capture "
                         "quality — a 640x480 stream just gets upscaled. Use --width/--height "
                         "for actual resolution")
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
        # ⚠️ Runs through FrameGrabber, the same class the viewer uses. The previous
        # version measured a different code path and therefore measured nothing.
        grab = FrameGrabber(cap)
        t0 = time.perf_counter()
        gaps, last, seen, prev_seq = [], t0, 0, -1
        try:
            while time.perf_counter() - t0 < args.measure:
                frame, seq = grab.newest()
                if frame is None or seq == prev_seq:
                    time.sleep(0.001)
                    continue
                prev_seq = seq
                now = time.perf_counter()
                gaps.append(now - last)
                last = now
                seen += 1
        finally:
            cap_fps = grab.capture_fps()
            grab.stop()
        if not gaps:
            print("no frames captured at all.")
            return 1
        g = np.array(gaps[1:]) * 1000.0
        print(f"\n  captured      : {cap_fps:.1f} fps  (what the camera delivers)")
        print(f"  delivered     : {seen / args.measure:.1f} fps  (what a viewer would see)")
        print(f"  frame interval: mean {g.mean():.1f} ms  p50 {np.percentile(g, 50):.1f}  "
              f"p95 {np.percentile(g, 95):.1f}  max {g.max():.1f}")
        print("\n  ⚠️ This is the CAPTURE interval, not glass-to-glass latency. It bounds it")
        print("     from below; the sensor, USB transport and display add more. To measure the")
        print("     real thing, point the camera at a running stopwatch and photograph both.")
        return 0

    if args.term:
        return run_terminal(cap, args)

    win = "wrist camera — q or ESC to quit"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    if args.big:
        cv2.resizeWindow(win, 1600, 900)

    print("\n  q or ESC quits.   f mirrors.   r rotates 90°.")
    print("  1..5 switch resolution live: 424x240 / 640x480 / 960x540 / 1280x720 / 1920x1080")
    print("  ⭐ Drive with `teleop_session.py` in another terminal and press v there to put")
    print("     the controls in the TOOL frame — then 'forward' on the puck means forward in")
    print("     THIS picture, which is the point of having the camera.\n")

    flip, rotate = args.flip, args.rotate
    grab = FrameGrabber(cap)
    shown, t_fps, disp_fps, prev_seq = 0, time.perf_counter(), 0.0, -1
    try:
        while True:
            frame, seq = grab.newest()
            if frame is None:
                if cv2.waitKey(5) & 0xFF in (ord("q"), 27):
                    break
                continue
            if seq != prev_seq:
                prev_seq = seq
                shown += 1
            if flip:
                frame = cv2.flip(frame, 1)
            r = {90: cv2.ROTATE_90_CLOCKWISE, 180: cv2.ROTATE_180,
                 270: cv2.ROTATE_90_COUNTERCLOCKWISE}.get(rotate)
            if r is not None:
                frame = cv2.rotate(frame, r)

            now = time.perf_counter()
            if now - t_fps >= 0.5:
                disp_fps = shown / (now - t_fps)
                shown, t_fps = 0, now
            h, w = frame.shape[:2]
            # ⭐ The rate is drawn ON the picture. Julien could not tell 5 fps from 30
            # by eye until it was measured; a number in the corner makes a regression
            # obvious the instant it happens instead of after a session of confusion.
            cv2.putText(frame, f"{w}x{h}  {disp_fps:4.1f} fps shown / {grab.capture_fps():4.1f} captured",
                        (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(frame, f"{w}x{h}  {disp_fps:4.1f} fps shown / {grab.capture_fps():4.1f} captured",
                        (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1, cv2.LINE_AA)
            cv2.imshow(win, frame)

            k = cv2.waitKey(1) & 0xFF
            if k in (ord("q"), 27):
                break
            if k == ord("f"):
                flip = not flip
                print(f"  mirror {'ON' if flip else 'OFF'}")
            elif k == ord("r"):
                rotate = (rotate + 90) % 360
                print(f"  rotate {rotate}°")
            elif k in [ord(c) for c in "12345"]:
                w2, h2 = SIZES[int(chr(k)) - 1]
                grab.stop()
                cap = open_camera(args.index, w2, h2, args.fps)
                if cap is None:
                    print(f"  could not reopen at {w2}x{h2}")
                    return 1
                grab = FrameGrabber(cap)
                shown, t_fps, prev_seq = 0, time.perf_counter(), -1
                print(f"  resolution -> {w2}x{h2}")
    except KeyboardInterrupt:
        pass
    finally:
        grab.stop()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    sys.exit(main())
