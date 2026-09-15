#!/usr/bin/env python3
"""⭐ Live wrist-camera view, for driving the arm from the camera's point of view.

    uv run apps/camera_view.py --list          # which camera is which index
    uv run apps/camera_view.py                 # live view, lowest latency
    uv run apps/camera_view.py --index 1 --big # full-screen-ish

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

⭐ **CAMERAS HAVE NAMES, AND THE NAMES ARE MEASURED.** OpenCV opens cameras by INDEX
and offers no name API at all, while this repo has a hard-won rule against selecting
hardware by index (FINDINGS §0 #5: an adapter chosen by index silently retargeted the
wrong robot). macOS *will* name them — but ⛔ **macOS's enumeration order is not
OpenCV's**, which was learned the expensive way on 2026-08-11 after shipping a
positional guess that was wrong about two of four cameras.

So identity is established by **asking each index for a resolution only one camera
supports** and seeing who answers exactly (`identify_indices`). Nothing here uses list
order for anything. `--camera d405` then means what it says.

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
import os
import shutil
import sys
import termios
import time
from pathlib import Path

import cv2

# ⭐ Moved to the library on 2026-08-19 (ROADMAP §8.2 item 6): one copy, hardware-
# confirmed HERE, now shared with the capture layer. The API this file calls is
# unchanged — see yam/cameras/grabber.py for the move note.
from yam.cameras.grabber import FrameGrabber, fourcc_name  # noqa: E402
from yam.cameras.open import FIRST_FRAME_S, await_first_frame  # noqa: E402 — one copy of "wait for a frame"
import numpy as np

from yam.cameras.discovery import (  # noqa: E402
    MacCamera,
    ProbeResult,
    CameraLookupError,
    mac_cameras,
    probe_indices,
    frame_is_mono,
    discriminating_mode,
    model_discriminating_mode,
    identify_indices,
    hinted_index,
    find_camera_index,
    resolve_camera,
    MAX_PROBE_INDEX,
)
from yam.cameras.open import configure as configure_camera, open_camera  # noqa: E402


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

# A capture mode slower than this is a stills mode, not a view. See `key_sizes`.
MIN_LIVE_FPS = 15.0


def key_sizes(cam: "MacCamera | None") -> list[tuple[int, int]]:
    """The six capture sizes bound to keys 1-6 — **this camera's own modes** if known.

    ⛔⭐ THE BUG THIS FIXES, reported by Julien on 2026-08-11: *"the numbers when I
    press them don't allow for all the quality options. They cycle between about
    three, and not in the correct order either."*

    `SIZES` above is a list of **C920** modes. Point it at a different camera and a
    UVC device silently substitutes the nearest mode it does have, so several keys
    collapse onto the same result. On the MacBook Air camera — whose seven modes are
    640x480, 1280x720, 1552x1552, 1760x1328, 1328x1760, 1920x1080, 1080x1920 — keys
    1 to 4 all land on 640x480 and only 5 and 6 differ. **Exactly three distinct
    sizes, which is exactly what he saw**, and its portrait modes are why the order
    looked wrong too.

    ⭐ That symptom is also evidence: it says he was driving the built-in camera while
    the tool told him it was the C920 — the naming bug, showing up in a second place.

    Asking the device for its own modes removes the guesswork. Six spread evenly
    across the range, always including the smallest and largest, always ascending, so
    every key does something distinct and `6` is always "as good as this camera gets".
    """
    if not (cam and cam.modes):
        return SIZES
    # ⛔⭐ SLOW MODES ARE NOT "BETTER". The C920 advertises 2560x1472 at a **measured
    # 2 fps** — a stills mode. The first version of this function offered it on key 6
    # as "the best this camera can do", and Julien found the obvious consequence:
    # *"the frame rate drops to like two frames per second"*. It also bought nothing,
    # because what reaches the terminal is capped far below that anyway.
    #
    # A live view wants the sharpest mode that still MOVES. AVFoundation reports a
    # max frame rate per format, so this is a measurement rather than a judgement.
    usable = {(w, h) for w, h, fps in cam.mode_rates if fps >= MIN_LIVE_FPS}
    pool = (cam.modes & usable) if usable else cam.modes
    modes = sorted({(int(w), int(h)) for w, h in pool}, key=lambda wh: wh[0] * wh[1])
    # ⚠️ Landscape only, when there is a real choice. Apple's camera advertises
    # 1080x1920 and 1552x1552 — Center Stage crop modes — which by pixel count sort
    # ABOVE 1920x1080 and would put a portrait or square frame on key 6, where the
    # operator expects "the best this camera can do" for a video view.
    landscape = [m for m in modes if m[0] > m[1]]
    if len(landscape) >= 2:
        modes = landscape
    if len(modes) <= 6:
        return modes
    step = (len(modes) - 1) / 5
    return [modes[round(i * step)] for i in range(6)]


# ============================================================================
#  CAMERA IDENTITY — which index is which camera
# ============================================================================
#
# ⭐ WHY THIS SECTION EXISTS. Four cameras are visible on this Mac — the built-in
# one, the D405 on arm B, the C920, and Julien's iPhone over Continuity — their
# indices move on replug, and OpenCV reports **no name at all** (verified on OpenCV
# 5.0: `cv2.videoio_registry` exposes backends, never devices). Driving the arm from
# the wrong camera's point of view is exactly the class of mistake FINDINGS §0 #5
# was written about, where an adapter chosen by index silently retargeted the other
# robot.
#
# The way out is that **macOS will tell us the names** even though OpenCV will not.


def list_cameras() -> None:
    """Everything known about every camera: what macOS lists, which index each one is
    **measured** to be, and what each index actually shows. Run after any replug."""
    from yam.platform import IS_LINUX, read_v4l_cameras  # noqa: PLC0415

    if IS_LINUX:
        # ⭐ On Linux this report is short because the kernel already answers it: the by-id
        # name carries model and serial, and its symlink target IS OpenCV's index. There is
        # nothing to measure, so measuring would only be theatre (ROADMAP §8.2 item 49).
        cams_linux = read_v4l_cameras()
        if not cams_linux:
            print("⛔ nothing in /dev/v4l/by-id — no camera is attached, or this user is not")
            print("   in the `video` group. `uv run checks/check_platform.py --raw` prints")
            print("   the raw listing and says which it is.")
            return
        print(f"Linux lists {len(cams_linux)} camera(s) from /dev/v4l/by-id.")
        print("⭐ These indices ARE OpenCV's — the by-id symlink points at /dev/videoN, so")
        print("   nothing here is inferred from a list order.\n")
        print(f"  {'idx':>3s}  {'device':<13s} {'serial':<16s} model")
        for cam in cams_linux:
            print(f"  {cam.index:>3d}  {cam.device:<13s} "
                  f"{cam.serial or '(none reported)':<16s} {cam.model}")
        print("\n  Select by name or serial — both work, and the name is what a recording"
              " stores:")
        print("      uv run apps/camera_view.py --camera c920 --term")
        print("      uv run apps/camera_view.py --camera d405:2603 --term")
        return

    cams = mac_cameras()
    if cams:
        print(f"macOS lists {len(cams)} camera(s). ⚠️ THIS ORDER IS NOT OpenCV's ORDER —")
        print("   assuming it was is the bug this tool was rewritten to fix.\n")
        for cam in cams:
            usb = f"   USB {cam.usb}" if cam.usb else ""
            modes = f"   {len(cam.modes)} modes" if cam.modes else "   (no mode list)"
            print(f"    {cam.short:<28s}{modes}{usb}".rstrip())
            # ⭐ The uniqueID is printed for ROADMAP §8.2 item 5's open hypothesis: if a
            # D405's AVFoundation uniqueID embeds its USB locationID (0x01220000 and
            # 0x01210000 for the two D405s measured in FINDINGS §70.6), then
            # serial→locationID→uniqueID→index closes identical-camera identification
            # with no root and no lens-covering. One glance at this line with both
            # D405s attached answers it.
            if cam.unique_id:
                print(f"        uniqueID {cam.unique_id}")
        print()
    else:
        print("⚠️  macOS reported no cameras at all.\n")

    print("identifying indices by measurement — asking each one for a mode only one")
    print("camera has. This opens every camera briefly.\n")
    identified, id_notes = identify_indices(cams)
    for note in id_notes:
        print(f"  {note}")

    print("\nreading a real frame from each index (waiting out the sensor warm-up)\n")
    probes = probe_indices(read_frames=True,
                           limit=len(cams) + 1 if cams else MAX_PROBE_INDEX)
    if not probes:
        print("  none opened. On macOS the FIRST run must be granted camera access —")
        print("  look for the permission dialog, then run this again.")
        return

    named = {idx: cam for idx, cam in identified}
    print(f"  {'idx':>3s}  {'measured identity':<28s} {'resolution':>11s}  "
          f"{'bright':>6s} {'settle':>7s}  picture")
    for probe in probes:
        cam = named.get(probe.index)
        name = cam.short if cam else "⛔ UNIDENTIFIED"
        picture = ("cannot say — no variation in the frame" if probe.mono is None
                   else "MONO — depth/IR, not a picture" if probe.mono else "colour")
        bright = "   n/a" if probe.mean != probe.mean else f"{probe.mean:6.0f}"  # NaN
        print(f"  {probe.index:>3d}  {name[:28]:<28s} {probe.width:>5d}x{probe.height:<5d} "
              f"{bright} {probe.settle:6.1f}s  {picture}")

    print("\n  Select by name — the index moves on replug, the name does not:")
    print("      uv run apps/camera_view.py --camera c920 --term")
    print("      uv run apps/camera_view.py --camera d405 --term")
    print("\n  ⭐ These names are MEASURED, not inferred from any list order. To check")
    print("     one anyway: cover a camera and re-run — the index that goes dark is it.")


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
            for _ in range(5):
                cap.read()
            # ⛔⭐ THE "actual" COLUMN USED TO COME FROM `cap.get`, WHICH ANSWERS THE REQUEST.
            # In this table of all places: its whole purpose is telling the operator which
            # sizes a camera really delivers, and the line below is the one they read to pick
            # `--width`/`--height`. A camera that accepts a size and streams nothing at it
            # showed up here as working (FINDINGS §76.0, §76.2). The shape of a real frame is
            # the only honest answer, and this loop was already reading frames.
            t0 = time.perf_counter()
            n = 0
            shape = None
            while time.perf_counter() - t0 < secs:
                ok_read, got_frame = cap.read()
                if ok_read and got_frame is not None:
                    n += 1
                    shape = got_frame.shape[:2]
            dur = time.perf_counter() - t0
            cap.release()
            actual = f"{shape[1]}x{shape[0]}" if shape else "NO FRAMES"
            print("%-24s %-12s %-7s %.1f" % (f"{w}x{h} {cc or 'as-is'}", actual, got_s, n / dur))
    print("\n  Pick the largest size that still gives ~30 fps and pass it as")
    print("  --width/--height. If MJPG changes nothing, macOS is ignoring the codec")
    print("  request and resolution is your only lever — which is why the default is 640x480.")


from yam.ui.camera_render import (  # noqa: E402
    UPPER_HALF,
    render_ansi,
    CellSize,
    ASSUMED_CELL,
    cell_size,
    IMAGE_WIDTH_CAP,
    auto_image_width,
    useful_image_width,
    SHRINK_AFTER,
    tune_image_width,
    terminal_grid,
    IMAGE_TERMINALS,
    detect_term_mode,
    term_diagnosis,
    _downscale,
    render_kitty,
    render_iterm,
)

def term_test(cols: int = 40, rows: int = 12) -> int:
    """Send a test image in **each** protocol, errors ENABLED, and report the replies.

    ⭐ This exists because `q=2` — correct for a 30 fps loop — silently swallowed the
    error that would have identified the JPEG-labelled-as-PNG bug immediately. One
    command now produces ground truth instead of a guess.

    kitty replies `ESC _G i=<id>;OK ESC \\` on success, or `ESC _G i=<id>;<ERROR>
    ESC \\` on failure. Anything else — including silence — means the terminal does
    not implement the protocol at all.

    ⭐⭐ It now tests **iTerm2's protocol too**, and the reason is worth stating: that
    one carries JPEG, and JPEG is ~25x cheaper than the PNG the kitty protocol
    forces. If a terminal happens to speak both — Ghostty is the open question on
    this rig — then the sharpness ceiling on the terminal view moves by a factor of
    two. That is worth ten seconds of looking at the screen. iTerm2's protocol
    defines no reply, so this half is a question to a human, not a measurement.
    """
    import select
    import termios
    import tty

    def bars(label: str):  # noqa: ANN202
        img = np.zeros((180, 320, 3), np.uint8)
        img[:60] = (60, 60, 220); img[60:120] = (60, 220, 60); img[120:] = (220, 120, 60)
        cv2.putText(img, label, (30, 100), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
        return img

    cellinfo = cell_size()
    origin = ("measured from the terminal" if cellinfo.measured else
              "ASSUMED — this terminal does not report ws_xpixel, so the image size "
              "is a guess")
    print(term_diagnosis())
    print(f"\n  cell size: {cellinfo.width:.1f}x{cellinfo.height:.1f} px ({origin})\n")

    print("Sending TWO test images: one in each protocol, errors ON.\n")
    fd = sys.stdin.fileno()
    try:
        saved = termios.tcgetattr(fd)
    except Exception:  # noqa: BLE001
        print("⚠️  not a terminal — run this directly in your shell.")
        return 1
    try:
        tty.setcbreak(fd)
        # ⛔⭐ `image_id` IS WHAT MAKES A REPLY POSSIBLE, and leaving it out is why
        # this test reported a false negative on 2026-08-12. The kitty protocol keys
        # its response to an image **id** (`i=`) or number (`I=`); with neither, the
        # terminal has nothing to answer *about* and correctly says nothing. Ghostty
        # 1.3.1 was accordingly declared "does not implement this protocol" while it
        # was, at that very moment, drawing the camera view in kitty mode.
        #
        # A test that says "unsupported" when it means "I asked a question that has
        # no addressee" is the same confident-wrong-answer this file keeps being
        # rewritten to avoid — this time in the diagnostic itself.
        sys.stdout.write(render_kitty(bars("KITTY"), cols, rows, quiet=False, image_id=771))
        sys.stdout.flush()
        # ⛔⭐ READ THE DESCRIPTOR, NOT `sys.stdin`. This had the same defect as
        # KeyReader: `select()` asks the file descriptor whether bytes are waiting
        # while `sys.stdin.read(1)` pulls them into Python's own buffer, where the
        # descriptor cannot see them. The visible result on 2026-08-12 was a reply of
        # exactly `'\x1b'` — one byte of a longer answer — which this then reported as
        # **"⛔ ERROR"** about a terminal that had answered perfectly well.
        reply = ""
        deadline = time.time() + 2.0
        while time.time() < deadline:
            if select.select([fd], [], [], 0.05)[0]:
                chunk = os.read(fd, 256)
                if not chunk:
                    break
                reply += chunk.decode("utf-8", errors="replace")
                if reply.endswith("\x1b\\"):
                    break
        # ⭐ And now the OTHER protocol. It is worth knowing whether this terminal
        # takes iTerm2's escape as well, because that one carries **JPEG**: measured
        # 0.3 ms and 26 KB per 640px frame against PNG's 6.7 ms and 391 KB. A
        # terminal that speaks both should be driven with iterm mode, not kitty.
        # ⚠️ iTerm2's protocol defines no reply, so only a human can answer this.
        sys.stdout.write("\n")
        sys.stdout.write(render_iterm(bars("ITERM2"), cols, rows))
        sys.stdout.flush()
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, saved)

    print("\n\n--- 1. kitty graphics protocol (PNG) ---")
    kitty_ok = ";OK" in reply
    if not reply:
        print("  the terminal said nothing.")
        print("  ⚠️ That does NOT prove the protocol is missing — some terminals draw")
        print("     images perfectly well and never answer. **Look at the screen**: if")
        print("     bars labelled KITTY appeared, kitty mode works regardless of silence.")
    else:
        print(f"  it replied: {reply.replace(chr(27), '<ESC>')!r}")
        print("  ✅ OK — the protocol works." if kitty_ok else
              "  ⛔ ERROR — the text after ';' is the reason.")

    print("\n--- 2. iTerm2 inline-image protocol (JPEG) ---")
    print("  This protocol never replies, so look at the screen: did a SECOND set of")
    print("  bars appear, labelled ITERM2?")
    print("     yes -> use --term-mode iterm. It is ~25x cheaper than PNG, so the")
    print("            picture can be much sharper at the same frame rate.")
    print("     no  -> stay on kitty mode; detail is capped by PNG encode cost.")
    return 0 if (kitty_ok or reply) else 1


def run_terminal(cap, args, label: str = "", cam: "MacCamera | None" = None) -> int:  # noqa: ANN001
    """Draw the camera into this terminal. Creates no window at all.

    `label` is the camera's name, shown in the status line so a two-terminal setup
    can never be confused about which arm's view is which. `cam` supplies that
    camera's real capture modes for keys 1-6 (see `key_sizes`).

    ⛔ Quitting is guaranteed: `q`/ESC, Ctrl-C and the `finally` block all restore the
    cursor and colours and stop the grabber thread. Julien asked specifically that
    every test be quittable, and a tool that leaves a terminal without a cursor is one
    people stop reaching for.
    """
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))
    from yam.inputs.keyboard import KeyReader  # noqa: PLC0415

    best, why = detect_term_mode()
    mode = best if args.term_mode == "auto" else args.term_mode
    print(f"  terminal: {why}")
    print(f"  drawing with: {mode}"
          f"{'  (forced)' if args.term_mode != 'auto' else ''}")
    if mode == "blocks":
        print("  ⚠️ Block mode is one character cell per pixel — that is the medium's")
        print("     limit, not a bug. If your terminal does support images, force it:")
        print("     --term-mode iterm   or   --term-mode kitty     (b cycles them live)")
    # Long enough to read the two lines above, short enough not to be a third of the
    # startup delay. Opening the camera already costs seconds on macOS.
    time.sleep(0.6)
    scale = min(1.0, max(0.1, args.scale))

    grab = FrameGrabber(cap)
    flip, rotate = args.flip, args.rotate
    sizes = key_sizes(cam)
    # None = size the image automatically. A number pins it, from --image-width or
    # the [ and ] keys.
    manual_width = args.image_width or None
    # ⭐ The adaptive detail level, in pixels of image width. It starts at the
    # measured-safe cap and then climbs or backs off against the real draw cost —
    # see useful_image_width() for why a constant could not do this job.
    auto_w = float(IMAGE_WIDTH_CAP.get(mode, 720))
    next_tune = 0.0
    # How many consecutive over-budget frames have been seen. Reset by any frame inside
    # budget, so one hiccup cannot shrink the picture. See tune_image_width().
    over_budget = 0
    # ⭐ Set whenever a keypress changes what should be on screen, so the frame is
    # redrawn immediately instead of at the next capture.
    dirty = True
    # Two kitty image ids, used alternately. See render_kitty: placing the new image
    # before deleting the old one is what stops the picture blinking.
    kitty_id, kitty_prev = 991, 992
    shown, t_fps, disp_fps, prev_seq = 0, time.perf_counter(), 0.0, -1
    draw_ms = 0.0
    sys.stdout.write("\x1b[?25l\x1b[2J")
    try:
        with KeyReader() as keys:
            while True:
                frame, seq = grab.newest()
                if frame is None:
                    # ⛔ Keys must still be read here. Before the first frame arrives
                    # this branch skipped the key handler entirely, so a camera that
                    # never delivered one left `q` dead and Ctrl-C as the only way
                    # out — against Julien's standing requirement that every test be
                    # quittable.
                    if any(k in ("q", "\x1b") for k in keys.drain()):
                        return 0
                    time.sleep(0.01)
                    continue
                fresh = seq != prev_seq
                if fresh:
                    prev_seq = seq
                    shown += 1

                # ⛔⭐ REDRAWING A FRAME THE TERMINAL ALREADY HAS IS PURE HARM, and it
                # was happening constantly. The display loop runs faster than the
                # camera delivers — at 30 fps capture and a ~18 ms draw it goes round
                # about 55 times a second — so nearly half of every second was spent
                # re-encoding and re-transmitting an identical picture. In kitty mode
                # each of those redraws also deleted and replaced the image, which is
                # **the flicker Julien reported**: "some frames seem to not be drawn".
                # Skipping them halves the terminal's load and removes half the
                # flashes. `dirty` covers keypresses, which must redraw at once
                # rather than waiting for the next frame.
                if not (fresh or dirty):
                    time.sleep(0.002)
                else:
                    dirty = False
                    if flip:
                        frame = cv2.flip(frame, 1)
                    r = {90: cv2.ROTATE_90_CLOCKWISE, 180: cv2.ROTATE_180,
                         270: cv2.ROTATE_90_COUNTERCLOCKWISE}.get(rotate)
                    if r is not None:
                        frame = cv2.rotate(frame, r)

                    h, w = frame.shape[:2]
                    # Re-measured every frame so resizing the window or changing the
                    # font size is picked up live rather than at the next launch.
                    cell = cell_size()
                    cols, rows = terminal_grid(w / h, scale, cell_aspect=cell.aspect)
                    ceiling = useful_image_width(cols, w, cell)
                    sent_w = int(min(manual_width or auto_w, ceiling))
                    sent_h = max(1, round(h * sent_w / w))

                    now = time.perf_counter()
                    if now - t_fps >= 0.5:
                        disp_fps = shown / (now - t_fps)
                        shown, t_fps = 0, now

                    t_draw = time.perf_counter()
                    if mode == "iterm":
                        body = render_iterm(frame, cols, rows, sent_w)
                    elif mode == "kitty":
                        body = render_kitty(frame, cols, rows, sent_w,
                                            image_id=kitty_id, previous_id=kitty_prev)
                        kitty_id, kitty_prev = kitty_prev, kitty_id
                    else:
                        body = render_ansi(frame, cols, rows)
                    sys.stdout.write("\x1b[H" + body)

                    # ⭐ The status line answers "did that keypress do anything?" —
                    # which is the question that made keys 1-6 look broken. Every
                    # number a key can change is on screen: the capture size, the size
                    # actually sent to the terminal, the cell grid, the draw cost.
                    detail = ("blocks — 2 px per cell" if mode == "blocks"
                              else f"sent {sent_w}x{sent_h} "
                                   f"{'fixed' if manual_width else 'auto'}/{ceiling} max")
                    cell_note = (f"cell {cell.width:.0f}x{cell.height:.0f}px"
                                 if cell.measured else
                                 f"cell {cell.width:.0f}x{cell.height:.0f}px ASSUMED")
                    # A draw cost past half the frame interval means the terminal, not
                    # the camera, is the bottleneck — and it is fixable from here.
                    budget = 1000.0 / max(1.0, grab.capture_fps())
                    if manual_width and draw_ms > 0.5 * budget:
                        warn = "   ⚠️ draw is over half the frame — press 0 for auto detail"
                    elif w > 1.7 * sent_w:
                        # ⭐ Decoding pixels that are then thrown away costs USB
                        # bandwidth, CPU and latency for nothing visible.
                        warn = "   ⚠️ capturing far more than is sent — a smaller capture looks the same"
                    else:
                        warn = ""
                    sys.stdout.write(
                        f"\x1b[0m\n{label}capture {w}x{h} {grab.pixel_format()} · {detail} · {mode} · "
                        f"{cols}x{rows} cells · {cell_note}\x1b[K\n"
                        f"{disp_fps:4.1f} shown / {grab.capture_fps():4.1f} captured fps · "
                        f"draw {draw_ms:4.1f} ms{warn}\x1b[K\n"
                        f"q quit · f mirror · r rotate · b draw mode · +/- pane · "
                        f"1-6 capture size · [ ] detail · 0 auto\x1b[K"
                    )
                    sys.stdout.flush()
                    # ⭐ The draw cost is displayed because it is the latency the
                    # SOFTWARE controls. If it approaches the frame interval the
                    # terminal cannot keep up, output backs up in the pipe, and lag
                    # grows without any single component looking wrong.
                    draw_ms = (time.perf_counter() - t_draw) * 1000.0

                    # ⭐⭐ CLIMB TO WHAT THIS TERMINAL ACTUALLY SUSTAINS. Julien:
                    # *"the max resolution that can be sent is 720x405 … it doesn't
                    # really make any sort of difference"* — because 720 was a
                    # constant picked from PNG encode cost on one machine, and on his
                    # the cost is dominated by writing the bytes, not encoding them.
                    #
                    # Spend at most half the frame interval drawing, and use the
                    # measured draw to decide. Backs off fast (×0.85) and climbs
                    # slowly (×1.15) with a dead band between, because overshooting
                    # costs frame rate the operator notices and undershooting only
                    # costs detail they can ask for with `]`.
                    if manual_width is None and now >= next_tune:
                        next_tune = now + 0.4
                        target = 0.5 * (1000.0 / max(1.0, grab.capture_fps()))
                        auto_w, over_budget = tune_image_width(
                            auto_w, draw_ms, target, over_budget, ceiling)

                for k in keys.drain():
                    if k in ("q", "\x1b"):
                        return 0
                    dirty = True      # any recognised key changes what should be shown
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
                    elif k in "123456" and int(k) <= len(sizes):
                        # ⚠️ Bounds-checked: a camera with fewer than six distinct
                        # modes gets a shorter list, and an unbound key must do
                        # nothing rather than crash the viewer mid-session.
                        w2, h2 = sizes[int(k) - 1]
                        grab.stop()
                        # ⚠️ A camera released a moment ago is not always instantly
                        # re-openable. One retry, rather than killing the viewer and
                        # the operator's terminal layout with it.
                        cap = open_camera(args.index, w2, h2, args.fps)
                        if cap is None:
                            time.sleep(0.3)
                            cap = open_camera(args.index, w2, h2, args.fps)
                        if cap is None:
                            sys.stdout.write("\x1b[0m\x1b[?25h\n  could not reopen "
                                             f"camera {args.index} at {w2}x{h2}\n")
                            return 1
                        grab = FrameGrabber(cap)
                        shown, t_fps, prev_seq = 0, time.perf_counter(), -1
                        sys.stdout.write("\x1b[2J")
                    # ⭐ Detail, separate from capture size. Capture is what the
                    # camera sends the Mac; this is what the Mac sends the terminal,
                    # and in kitty/Ghostty it is the expensive one — PNG only.
                    elif k == "]":
                        manual_width = min(1920, (manual_width or sent_w) + 160)
                    elif k == "[":
                        manual_width = max(160, (manual_width or sent_w) - 160)
                    elif k == "0":
                        manual_width = None
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
    ap.add_argument("--camera", default="",
                    help="⭐ select by NAME instead of index — 'c920', 'd405', 'iphone', "
                         "'builtin', or any part of the name --list prints. The index "
                         "moves when something is replugged; the name does not. Refuses "
                         "rather than guessing if the name cannot be pinned to one index")
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
                    help="⭐ send one image in EACH protocol with error reporting ON and print "
                         "what the terminal says. Use this when image mode shows nothing — it "
                         "turns a blank screen into the terminal's actual error message — and "
                         "to find out whether this terminal also takes iTerm2's JPEG escape, "
                         "which is ~25x cheaper than the PNG kitty mode forces")
    ap.add_argument("--image-width", type=int, default=0,
                    help="pixels wide to send in iterm/kitty mode. ⭐ Default 0 = AUTO: "
                         "fit the pane, never exceed what was captured, and stay inside "
                         "the protocol's budget (kitty is PNG-only and costs ~25x what "
                         "iTerm2's JPEG does). A number pins it; [ and ] change it live "
                         "against the on-screen draw-ms readout")
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

    # ⭐ Name → index BEFORE anything opens a device, so --probe and the viewer both
    # act on the camera that was asked for. Resolution refuses rather than guessing,
    # and it prints what it chose: FINDINGS §0 #5 is about an adapter picked by index
    # that silently drove the other robot.
    label, chosen, ready = "", None, None
    if args.camera:
        try:
            args.index, chosen, ready = resolve_camera(args.camera)
        except CameraLookupError as exc:
            print(f"⛔ {exc}")
            return 1
        if chosen is not None:
            cam = chosen
            label = f"{cam.short} · "
            print(f"  {args.camera!r} → index {args.index}: {cam.short}  (measured)")
            if "depth" in cam.name.lower():
                print("  ⚠️ macOS exposes only this camera's DEPTH stream over plain UVC, "
                      "so expect\n     a depth/infrared picture rather than colour. "
                      "`--list` shows which it is.")

    if args.probe:
        probe_modes(args.index)
        return 0

    # ⭐ Reuse the handle the identification already opened — see find_camera_index.
    t_open = time.perf_counter()
    if ready is not None:
        cap = configure_camera(ready, args.width, args.height, args.fps)
    else:
        cap = open_camera(args.index, args.width, args.height, args.fps)
    if cap is None:
        print(f"⛔ could not open camera index {args.index}. Try:  --list")
        return 1

    # ⛔⭐ ONE REAL FRAME BEFORE THE SIZE IS PRINTED. This line used to read the size back
    # from the handle, which returns what was ASKED for. The fps already said "requested"
    # and the size did not, so the same line carried a request and a claim side by side
    # (FINDINGS §76.0). `await_first_frame` is the same helper the session's camera open uses, so
    # there is one copy of "wait for a frame, with a deadline".
    first = await_first_frame(cap, FIRST_FRAME_S)
    if first is None:
        print(f"⛔ camera {args.index} opened and delivered NO FRAMES within "
              f"{FIRST_FRAME_S:.1f}s. It accepted the settings and produced nothing.\n"
              "  Try a smaller --width/--height: a camera can accept a size it cannot "
              "stream (FINDINGS §76.2).")
        cap.release()
        return 1
    h, w = first.shape[:2]
    print(f"camera {args.index}: measured {w}x{h}, requested {args.fps} fps, MJPG"
          f"  ({'reused the probe handle' if ready is not None else 'opened'} in "
          f"{time.perf_counter() - t_open:.1f}s)")

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
        return run_terminal(cap, args, label, chosen)

    win = "wrist camera — q or ESC to quit"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    if args.big:
        cv2.resizeWindow(win, 1600, 900)

    sizes = key_sizes(chosen)
    print("\n  q or ESC quits.   f mirrors.   r rotates 90°.")
    print("  1..%d switch resolution: %s" % (len(sizes),
                                             " / ".join(f"{w}x{h}" for w, h in sizes)))
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
            elif k in [ord(c) for c in "123456"] and int(chr(k)) <= len(sizes):
                w2, h2 = sizes[int(chr(k)) - 1]
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
