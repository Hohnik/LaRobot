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
import base64
import os
import shutil
import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np

MAX_PROBE_INDEX = 6

# Live-switchable capture sizes, bound to keys 1..6.
#
# ⚠️ A UVC webcam only supports a FIXED LIST of modes, and asking for anything else
# silently gets you the nearest one it does have. Julien noticed `--probe` reporting
# "424x240 requested -> 640x360 actual": 424x240 is not a C920 mode, so the driver
# substituted. These are all real C920 modes, so what you ask for is what you get.
#
# ⭐ For the TERMINAL view, capture resolution barely affects picture quality — the
# renderer downsamples to the character grid anyway. What it does affect is
# **latency and CPU**: fewer pixels to transfer, decode and scale. So 320x180 is the
# right choice for the lowest-latency terminal view, and the large sizes are for the
# windowed view or for recording.
SIZES = [(320, 180), (320, 240), (640, 360), (640, 480), (1280, 720), (1920, 1080)]


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


def terminal_grid(frame_aspect: float, scale: float = 1.0, margin_rows: int = 2) -> tuple[int, int]:
    """Character columns and rows to use, **preserving the picture's aspect ratio**.

    ⛔ THE BUG THIS FIXES. The first version returned the whole terminal and let
    `render_ansi` squash the frame into it, so a 16:9 camera came out stretched to
    whatever shape the pane happened to be. Julien's screenshot showed it clearly.

    The geometry: a monospace cell is about twice as tall as it is wide, and the
    half-block trick puts **two** stacked pixels in each cell — so one rendered pixel
    is roughly square, and a grid of `C` columns by `R` rows displays with an aspect
    ratio of `C / (2R)`. To match a source of aspect `A` we therefore need
    `C = 2 · R · A`, and we take the largest such grid that fits inside the terminal.

    `scale` shrinks it below the maximum, for a small corner view rather than a
    full-pane one.
    """
    size = shutil.get_terminal_size(fallback=(100, 30))
    max_cols = max(20, int(size.columns * scale))
    max_rows = max(6, int((size.lines - margin_rows) * scale))
    rows = min(max_rows, int(max_cols / (2 * frame_aspect)))
    rows = max(6, rows)
    cols = min(max_cols, int(2 * rows * frame_aspect))
    return max(20, cols), rows


# ⭐ Terminals that can draw a real image, and how. Keys are what they set in the
# environment; values are the protocol they speak.
#
# ⚠️ This list is the reason `b` appeared broken. It toggled between "blocks" and
# `detect_term_mode()`, so in a terminal this list did not recognise, BOTH sides of
# the toggle were "blocks" and pressing it changed nothing visibly. A toggle whose
# two states can be identical is not a toggle — and worse, kitty was *detected* and
# then silently discarded because its protocol was unimplemented. Both are fixed:
# kitty is implemented, and the mode is always reported so a downgrade is visible.
IMAGE_TERMINALS = {
    "iTerm.app": "iterm",
    "WezTerm": "iterm",
    "vscode": "iterm",        # VS Code implements iTerm2's inline-image escape
    "Hyper": "iterm",
    "Tabby": "iterm",
    "ghostty": "kitty",
    "kitty": "kitty",
    "rio": "kitty",
    "konsole": "kitty",
    "warp": "iterm",
    "WarpTerminal": "iterm",
}


def detect_term_mode() -> tuple[str, str]:
    """Best available drawing method, and a human-readable reason.

    Returns e.g. `("iterm", "iTerm.app supports inline images")` or
    `("blocks", "Apple_Terminal has no image protocol — coloured text only")`.

    ⛔ The reason is returned, not just the mode. A silent fallback to blocks is
    indistinguishable from a broken feature, which is exactly how `b` wasted
    Julien's time.
    """
    prog = os.environ.get("TERM_PROGRAM", "")
    term = os.environ.get("TERM", "")
    if os.environ.get("KITTY_WINDOW_ID") or term == "xterm-kitty":
        return "kitty", "kitty graphics protocol detected"
    if term.startswith("xterm-ghostty") or prog == "ghostty":
        return "kitty", "Ghostty detected (speaks the kitty graphics protocol)"
    for key, mode in IMAGE_TERMINALS.items():
        if prog and key.lower() in prog.lower():
            return mode, f"{prog} supports inline images ({mode} protocol)"
    if prog:
        return "blocks", f"{prog} reports no image protocol — coloured text only"
    return "blocks", ("no TERM_PROGRAM set, so image support cannot be detected — "
                      "coloured text only. Force with --term-mode iterm or kitty to try anyway")


def term_diagnosis() -> str:
    """Everything relevant about this terminal, for pasting into a conversation."""
    keys = ("TERM_PROGRAM", "TERM_PROGRAM_VERSION", "TERM", "COLORTERM",
            "KITTY_WINDOW_ID", "WEZTERM_PANE", "LC_TERMINAL")
    lines = ["terminal environment:"]
    for k in keys:
        lines.append(f"    {k:22s} {os.environ.get(k, '(unset)')}")
    mode, why = detect_term_mode()
    lines += ["", f"  -> best mode: {mode}", f"     because   : {why}", "",
              "  If your terminal DOES support images and was not detected, force it:",
              "      uv run scripts/camera_view.py --term --term-mode iterm",
              "      uv run scripts/camera_view.py --term --term-mode kitty",
              "  and tell the agent which one worked so the detection list can be fixed."]
    return "\n".join(lines)


def _downscale(frame, max_width: int):  # noqa: ANN001, ANN201
    """Shrink to at most `max_width`, preserving shape. Payload is latency.

    The terminal scales whatever it receives into the cell box, so sending more
    pixels than the box can display is pure cost. Measured PNG payloads for a
    photo-like frame, and why the default is not 720p:

        1280x720   998 KB   31 ms encode   ->  40 MB/s at 30 fps. Impossible.
         640x360   283 KB    6.6 ms
         480x270  ~180 KB   ~3 ms          ->  the default
         320x180    78 KB    1.6 ms        ->  still 22x the detail of blocks
    """
    h, w = frame.shape[:2]
    if w <= max_width:
        return frame
    return cv2.resize(frame, (max_width, max(1, round(h * max_width / w))),
                      interpolation=cv2.INTER_AREA)


def render_kitty(frame, cols: int, rows: int, max_width: int = 480, quiet: bool = True) -> str:
    """A real image, drawn by the kitty graphics protocol (kitty, Ghostty, Konsole).

    ⛔⭐ THE BUG THIS FIXES, and it is a good lesson in reading a spec properly.

    The first version encoded **JPEG** and labelled it `f=100`. But in the kitty
    protocol `f` takes only three values — `f=24` (raw RGB), `f=32` (raw RGBA) and
    `f=100` (**PNG**). **There is no JPEG.** So the terminal was handed JPEG bytes,
    told they were PNG, failed to decode them, and said nothing — because `q=2` had
    suppressed exactly the error message that would have explained it. Julien saw a
    blank screen in kitty mode while blocks mode worked.

    ⚠️ Two lessons worth keeping. **A format code is not a MIME type**: `f=100`
    named the container, and assuming it meant "some compressed image" is the same
    class of error as assuming an SDK flag means what its name suggests. And
    **suppressing errors cost more than the noise it saved** — `q=2` is right for a
    30 fps redraw, but it turned a one-line diagnosis into a session of guessing,
    which is why `--term-test` now exists to send one image with errors ENABLED.

    Three further protocol facts, all load-bearing:

    - **PNG is the only compressed format**, and PNG of a photo is large — hence the
      downscale. `IMWRITE_PNG_COMPRESSION=1` is deliberate: level 1 costs ~1.6 ms at
      320x180 where the default costs several times that, for a few percent of size.
    - **Images persist until deleted.** One per frame at 30 fps accumulates
      placements without bound, so `a=d,d=A` clears the previous frame first.
    - **The terminal replies on stdin**, which this viewer reads for keypresses, so
      `q=2` is needed in the live loop or every frame injects junk input.
    """
    small = _downscale(frame, max_width)
    ok, buf = cv2.imencode(".png", small, [int(cv2.IMWRITE_PNG_COMPRESSION), 1])
    if not ok:
        return ""
    data = base64.b64encode(buf.tobytes()).decode("ascii")
    chunks = [data[i:i + 4096] for i in range(0, len(data), 4096)]
    q = "q=2," if quiet else ""
    out = [f"\x1b_Ga=d,d=A,{q}".rstrip(",") + "\x1b\\"]      # clear the previous placement
    for i, chunk in enumerate(chunks):
        first, last = i == 0, i == len(chunks) - 1
        ctrl = f"a=T,f=100,c={cols},r={rows},{q}" if first else ""
        out.append(f"\x1b_G{ctrl}m={0 if last else 1};{chunk}\x1b\\")
    return "".join(out)


def term_test(cols: int = 40, rows: int = 12) -> int:
    """Send ONE small image with errors ENABLED, and report what the terminal says.

    ⭐ This exists because `q=2` — correct for a 30 fps loop — silently swallowed the
    error that would have identified the JPEG-labelled-as-PNG bug immediately. One
    command now produces ground truth instead of a guess.

    kitty replies `ESC _G i=<id>;OK ESC \\` on success, or `ESC _G i=<id>;<ERROR>
    ESC \\` on failure. Anything else — including silence — means the terminal does
    not implement the protocol at all.
    """
    import select
    import termios
    import tty

    img = np.zeros((180, 320, 3), np.uint8)
    img[:60] = (60, 60, 220); img[60:120] = (60, 220, 60); img[120:] = (220, 120, 60)
    cv2.putText(img, "KITTY TEST", (40, 100), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)

    print("Sending one test image with error reporting ON.")
    print("You should see three colour bars and the words KITTY TEST.\n")
    payload = render_kitty(img, cols, rows, quiet=False)

    fd = sys.stdin.fileno()
    try:
        saved = termios.tcgetattr(fd)
    except Exception:  # noqa: BLE001
        print("⚠️  not a terminal — run this directly in your shell.")
        return 1
    try:
        tty.setcbreak(fd)
        sys.stdout.write(payload)
        sys.stdout.flush()
        reply = ""
        deadline = time.time() + 2.0
        while time.time() < deadline:
            if select.select([sys.stdin], [], [], 0.05)[0]:
                reply += sys.stdin.read(1)
                if reply.endswith("\x1b\\"):
                    break
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, saved)

    print("\n\n--- what the terminal replied ---")
    if not reply:
        print("  (nothing)  -> this terminal does not implement the kitty graphics protocol")
        return 1
    printable = reply.replace("\x1b", "<ESC>")
    print(f"  {printable!r}")
    if ";OK" in reply:
        print("\n  ✅ OK — the protocol works and the image above should be visible.")
        return 0
    print("\n  ⛔ the terminal reported an ERROR. The text after ';' is the reason.")
    return 1



def render_iterm(frame, cols: int, rows: int, max_width: int = 480, quality: int = 60) -> str:
    """A real image, drawn inline by iTerm2/WezTerm at full resolution.

    The frame is JPEG-encoded and base64'd into iTerm2's inline-image escape. JPEG
    rather than PNG deliberately: a 320x180 PNG is several times larger and the whole
    payload is written to the terminal every frame, so size is latency.
    """
    ok, buf = cv2.imencode(".jpg", _downscale(frame, max_width),
                           [int(cv2.IMWRITE_JPEG_QUALITY), quality])
    if not ok:
        return ""
    b64 = base64.b64encode(buf.tobytes()).decode("ascii")
    return (f"\x1b]1337;File=inline=1;width={cols};height={rows};"
            f"preserveAspectRatio=1;doNotMoveCursor=0:{b64}\x07")


def run_terminal(cap, args) -> int:  # noqa: ANN001
    """Draw the camera into this terminal. Creates no window at all.

    ⛔ Quitting is guaranteed: `q`/ESC, Ctrl-C and the `finally` block all restore the
    cursor and colours and stop the grabber thread. Julien asked specifically that
    every test be quittable, and a tool that leaves a terminal without a cursor is one
    people stop reaching for.
    """
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))
    from keyboard import KeyReader  # noqa: PLC0415

    best, why = detect_term_mode()
    mode = best if args.term_mode == "auto" else args.term_mode
    print(f"  terminal: {why}")
    print(f"  drawing with: {mode}"
          f"{'  (forced)' if args.term_mode != 'auto' else ''}")
    if mode == "blocks":
        print("  ⚠️ Block mode is one character cell per pixel — that is the medium's")
        print("     limit, not a bug. If your terminal does support images, force it:")
        print("     --term-mode iterm   or   --term-mode kitty     (b cycles them live)")
    time.sleep(1.2)
    scale = min(1.0, max(0.1, args.scale))

    grab = FrameGrabber(cap)
    flip, rotate = args.flip, args.rotate
    shown, t_fps, disp_fps, prev_seq = 0, time.perf_counter(), 0.0, -1
    draw_ms = 0.0
    sys.stdout.write("\x1b[?25l\x1b[2J")
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

                h, w = frame.shape[:2]
                cols, rows = terminal_grid(w / h, scale)

                now = time.perf_counter()
                if now - t_fps >= 0.5:
                    disp_fps = shown / (now - t_fps)
                    shown, t_fps = 0, now

                t_draw = time.perf_counter()
                if mode == "iterm":
                    body = render_iterm(frame, cols, rows, args.image_width)
                elif mode == "kitty":
                    body = render_kitty(frame, cols, rows, args.image_width)
                else:
                    body = render_ansi(frame, cols, rows)
                sys.stdout.write("\x1b[H" + body)
                sys.stdout.write(
                    f"\x1b[0m\n{w}x{h} {mode}  {cols}x{rows} cells  "
                    f"{disp_fps:4.1f} shown / {grab.capture_fps():4.1f} captured  "
                    f"draw {draw_ms:4.1f} ms\x1b[K\n"
                    f"q quit · f mirror · r rotate · 1-6 resolution · +/- size · b cycle draw mode\x1b[K"
                )
                sys.stdout.flush()
                # ⭐ The draw cost is displayed because it is the latency the SOFTWARE
                # controls. If it approaches the frame interval the terminal cannot
                # keep up, output backs up in the pipe, and lag grows without any
                # single component looking wrong. Seeing it makes that diagnosable.
                draw_ms = (time.perf_counter() - t_draw) * 1000.0

                for k in keys.drain():
                    if k in ("q", "\x1b"):
                        return 0
                    if k == "f":
                        flip = not flip
                    elif k == "r":
                        rotate = (rotate + 90) % 360
                    elif k == "b":
                        # ⛔ CYCLES through all three, rather than toggling against a
                        # detection that may return the mode you are already in. The
                        # old two-way toggle was a no-op in any terminal the detector
                        # did not recognise, which is indistinguishable from broken.
                        order = ["blocks", "iterm", "kitty"]
                        mode = order[(order.index(mode) + 1) % len(order)]
                        sys.stdout.write("\x1b[2J")
                        note = "" if mode == best else "  (not what was detected)"
                        sys.stdout.write(f"\r  drawing with: {mode}{note}\n")
                    elif k in "+=":
                        scale = min(1.0, scale + 0.1)
                        sys.stdout.write("\x1b[2J")
                    elif k == "-":
                        scale = max(0.1, scale - 0.1)
                        sys.stdout.write("\x1b[2J")
                    elif k in "123456":
                        w2, h2 = SIZES[int(k) - 1]
                        grab.stop()
                        cap = open_camera(args.index, w2, h2, args.fps)
                        if cap is None:
                            return 1
                        grab = FrameGrabber(cap)
                        shown, t_fps, prev_seq = 0, time.perf_counter(), -1
                        sys.stdout.write("\x1b[2J")
    except KeyboardInterrupt:
        pass
    finally:
        grab.stop()
        sys.stdout.write("\x1b[0m\x1b[?25h\n")
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
    ap.add_argument("--term-test", action="store_true",
                    help="⭐ send ONE image with error reporting ON and print what the terminal "
                         "says. Use this when image mode shows nothing — it turns a blank "
                         "screen into the terminal's actual error message")
    ap.add_argument("--image-width", type=int, default=480,
                    help="longest side of the image sent in iterm/kitty mode. Payload is "
                         "latency: 480 is ~180 KB/frame, 320 is ~104 KB, 720p is 1.3 MB and "
                         "will not keep up. Watch the draw-ms readout and tune")
    ap.add_argument("--term-info", action="store_true",
                    help="print what this terminal is and whether it can draw images, then exit")
    ap.add_argument("--term-mode", default="auto", choices=["auto", "blocks", "iterm", "kitty"],
                    help="how to draw in the terminal. auto detects iTerm2/WezTerm and uses "
                         "their inline-image protocol (full resolution, no pixelation); "
                         "blocks forces coloured text, which works in any terminal")
    ap.add_argument("--scale", type=float, default=1.0,
                    help="fraction of the terminal to fill, 0.1-1.0. Use ~0.4 for a small "
                         "corner view that keeps the picture's real proportions")
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

    if args.term_info:
        print(term_diagnosis())
        return 0

    if args.term_test:
        return term_test()

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
    print("  1..6 switch resolution: 320x180 / 320x240 / 640x360 / 640x480 / 1280x720 / 1920x1080")
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
            elif k in [ord(c) for c in "123456"]:
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
