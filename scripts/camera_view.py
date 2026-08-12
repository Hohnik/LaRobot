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
import base64
import fcntl
import json
import os
import shutil
import struct
import subprocess
import sys
import termios
import threading
import time
from dataclasses import dataclass
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
    modes = sorted({(int(w), int(h)) for w, h in cam.modes}, key=lambda wh: wh[0] * wh[1])
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


@dataclass(frozen=True)
class MacCamera:
    """One camera as macOS reports it.

    `modes` is the set of `(width, height)` this device can actually deliver, read
    from AVFoundation. ⭐ It is the load-bearing field: it is what lets an OpenCV
    index be **identified by measurement** instead of guessed at by position.
    Empty when only `system_profiler` was available.
    """

    name: str
    model_id: str
    unique_id: str
    modes: frozenset = frozenset()

    @property
    def usb(self) -> str | None:
        """`vid:pid` in hex, or None for a camera that is not USB.

        ⚠️ macOS writes the IDs in **decimal** inside the model string:
        `UVC Camera VendorID_32902 ProductID_2907` is `8086:0b5b`. Every datasheet,
        `ioreg` dump and USB tool speaks hex, so convert once here rather than
        leaving two number bases loose in the codebase.
        """
        vid = pid = None
        for token in self.model_id.split():
            if token.startswith("VendorID_"):
                vid = token.removeprefix("VendorID_")
            elif token.startswith("ProductID_"):
                pid = token.removeprefix("ProductID_")
        if not (vid and pid and vid.isdigit() and pid.isdigit()):
            return None
        return f"{int(vid):04x}:{int(pid):04x}"

    @property
    def short(self) -> str:
        """A name that fits on a status line without pushing the numbers off it."""
        tidy = " ".join(self.name.replace("(R)", "").replace("(TM)", "").split())
        return SHORT_NAMES.get(self.usb or "", tidy)


# Friendly names for the hardware actually on this rig. Keyed by USB id because
# that is stable — the marketing string is not (`HD Pro Webcam C920` is one of
# several names Logitech ships for the same camera).
SHORT_NAMES = {
    "8086:0b5b": "RealSense D405 (depth)",
    "046d:08e5": "C920 webcam",
}

_MAC_CAMERA_CACHE: list[MacCamera] | None = None


def _av_cameras() -> list[MacCamera]:
    """Cameras straight from AVFoundation, **with their supported modes**.

    Needs no camera permission — enumerating is not capturing — so the agent can run
    this even though it can never open a stream (FINDINGS §21.1).

    ⚠️ Returns `[]` when the binding is missing (a non-macOS machine, or a checkout
    that skipped the optional dependency). The caller then falls back to
    `system_profiler`, which gives names but **no modes**, and without modes there is
    no way to identify an index — so the tool refuses to name rather than guess.
    """
    try:
        import AVFoundation as AV  # noqa: PLC0415
        import CoreMedia as CM  # noqa: PLC0415
    except ImportError:
        return []
    cams = []
    for dev in AV.AVCaptureDevice.devicesWithMediaType_(AV.AVMediaTypeVideo) or []:
        modes = set()
        for fmt in dev.formats() or []:
            dims = CM.CMVideoFormatDescriptionGetDimensions(fmt.formatDescription())
            modes.add((int(dims.width), int(dims.height)))
        cams.append(MacCamera(str(dev.localizedName()), str(dev.modelID() or ""),
                              str(dev.uniqueID() or ""), frozenset(modes)))
    return cams


def _profiler_cameras() -> list[MacCamera]:
    """Names only, from `system_profiler`. The fallback when AVFoundation is absent."""
    try:
        out = subprocess.run(["system_profiler", "-json", "SPCameraDataType"],
                             capture_output=True, text=True, timeout=30, check=False)
        return [MacCamera(e.get("_name", "?"), e.get("spcamera_model-id", ""),
                          e.get("spcamera_unique-id", ""))
                for e in json.loads(out.stdout).get("SPCameraDataType", [])]
    except (OSError, ValueError, subprocess.SubprocessError):
        return []          # not macOS, or the tool changed its output shape


def mac_cameras(refresh: bool = False) -> list[MacCamera]:
    """Every camera macOS knows about. **The ORDER MEANS NOTHING — see below.**

    ⛔⭐ THE MISTAKE THIS DOCSTRING EXISTS TO PREVENT, because it was made here on
    2026-08-11 and shipped.

    The first version of this file assumed macOS's n-th camera was OpenCV's n-th
    index. It said so out loud, cross-checked it, and published a falsification
    procedure. **Julien ran that procedure and it failed:** covering each camera in
    turn showed the C920 answering on index 0, where macOS lists the built-in camera,
    and the built-in one answering on index 2, where macOS lists the C920.

    It is worse than a coincidence of one API. **Three separate macOS enumerations —
    `system_profiler`, `devicesWithMediaType:`, and an `AVCaptureDeviceDiscoverySession`
    asked in two different device-type orders — all return the SAME order, and it is
    not OpenCV's.** (A pattern, offered only as a lead if the measurement below ever
    fails: OpenCV's order looks like USB cameras sorted by location ID first, then
    built-in, then Continuity. `0x1120000` (C920) < `0x1210000` (D405), which is the
    order it used. Do not build on this; it is one observation.)

    ⭐ **So the order is never used.** Identity comes from `identify_indices()`, which
    asks each index for a resolution only one device supports and sees who answers.
    """
    global _MAC_CAMERA_CACHE  # noqa: PLW0603
    if _MAC_CAMERA_CACHE is None or refresh:
        _MAC_CAMERA_CACHE = _av_cameras() or _profiler_cameras()
    return _MAC_CAMERA_CACHE


@dataclass
class ProbeResult:
    """What one camera index answered when opened.

    ⚠️ `fps` is **not trustworthy** and is shown only for completeness: the same
    device reported 1 fps on one run and 30 on the next, so OpenCV is deriving it
    from frame timing rather than reading the format. `width`/`height` were stable
    across every run and are.
    """

    index: int
    ok: bool
    width: int
    height: int
    fps: float
    mean: float
    mono: bool | None
    settle: float = 0.0     # seconds until the camera produced a usable frame


def frame_is_mono(frame) -> bool | None:  # noqa: ANN001
    """True if the colour channels are identical, False if not, **None if the frame
    cannot say.** That third case is the whole point.

    ⛔⭐ THE BUG THIS FIXES, found by Julien's own `--list` output on 2026-08-11.
    A **black** frame has three identical channels, so the first version declared it
    `MONO — depth/IR, not a picture` and pointed that verdict at his **iPhone**,
    which is neither a depth camera nor an infrared one. The frame was black because
    the probe read it the instant the camera opened, before the sensor had exposed.

    **A frame with no variation carries no colour information at all.** The honest
    answer is "unknown", and dressing it up as a measurement is exactly the confident,
    plausible, wrong answer of FINDINGS §0 — produced, this time, by the code written
    to prevent it.

    What it is genuinely for: the D405's UVC entry is a depth stream widened into
    three equal channels, while a colour camera disagrees between channels almost
    everywhere — white balance alone guarantees that. On a frame that actually has
    content, this separates the two.
    """
    if frame is None or frame.ndim != 3 or frame.shape[2] != 3:
        return None
    if float(frame.std()) < 1.0:
        return None      # uniform: black, blown out, or a lens cap. It cannot say.
    return bool(np.array_equal(frame[..., 0], frame[..., 1])
                and np.array_equal(frame[..., 1], frame[..., 2]))


def probe_indices(read_frames: bool = True, limit: int | None = None,
                  settle: float = 1.5) -> list[ProbeResult]:
    """Open each index in turn and record what answered.

    ⛔⭐ **THE FIRST FRAME IS NOT THE PICTURE**, and reading it as though it were sent
    this investigation down a blind alley on 2026-08-11. The original probe took
    exactly one frame the instant the camera opened. Apple's built-in camera takes
    roughly half a second to expose, and the iPhone over Continuity takes longer
    still — so the built-in camera reported brightness 5 while it was looking at a
    bright room, and the iPhone reported `NO FRAME`. **Both readings were about
    sensor warm-up and nothing else**, and the brightness column is precisely what
    the operator is told to use to tell cameras apart.

    So this now reads frames until one actually has variation, or `settle` seconds
    pass, and reports how long that took — a slow camera is worth knowing about
    rather than silently averaging away.

    `read_frames=False` skips capture entirely: enough to count devices, not enough
    to say what they show.

    ⚠️ A closed index does **not** end the loop. Assuming indices are contiguous is
    the kind of tidy assumption this rig punishes; an unopenable index in the middle
    would silently shift everything after it.
    """
    results: list[ProbeResult] = []
    for idx in range(limit if limit is not None else MAX_PROBE_INDEX):
        cap = cv2.VideoCapture(idx)
        if not cap.isOpened():
            cap.release()
            continue
        ok, frame, waited = False, None, 0.0
        if read_frames:
            t0 = time.perf_counter()
            while time.perf_counter() - t0 < settle:
                got, candidate = cap.read()
                if got and candidate is not None:
                    ok, frame = True, candidate
                    if float(candidate.std()) >= 1.0:
                        break        # real content, not a warm-up frame
            waited = time.perf_counter() - t0
        results.append(ProbeResult(
            index=idx,
            ok=bool(ok),
            width=int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
            height=int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),
            fps=float(cap.get(cv2.CAP_PROP_FPS)),
            mean=float(np.mean(frame)) if ok and frame is not None else float("nan"),
            mono=frame_is_mono(frame) if ok else None,
            settle=waited,
        ))
        cap.release()
    return results


def discriminating_mode(cam: MacCamera, others: list[MacCamera]) -> tuple[int, int] | None:
    """A resolution **only this camera can deliver** — the question that identifies it.

    Every camera advertises a fixed list of modes, and the lists differ sharply on
    this rig: the C920 offers 34 including `160x90` and `176x144`, the MacBook Air
    camera offers 7 including the square `1552x1552`, the D405 offers `848x480`, the
    iPhone offers `1920x1440`. Ask a camera for a mode it does not have and **it
    substitutes the nearest one it does** — measured here in session 7, where
    `424x240` came back as `640x360`. So an *exact* match to a mode only one device
    owns is that device answering, whatever index it happens to be sitting on.

    Returns None when the camera shares every one of its modes with another — which
    ⚠️ **is exactly what will happen when the second D405 is plugged in.** Two
    identical cameras cannot be told apart this way, and the tool must say so rather
    than pick one.

    The smallest qualifying mode is chosen: least bandwidth, quickest to negotiate.
    """
    shared: set = set()
    for other in others:
        shared |= set(other.modes)
    unique = set(cam.modes) - shared
    return min(unique, key=lambda wh: wh[0] * wh[1]) if unique else None


def identify_indices(cams: list[MacCamera], limit: int | None = None,
                     ) -> tuple[list[tuple[int, MacCamera | None]], list[str]]:
    """Work out which OpenCV index is which camera **by measurement**.

    ⛔⭐ THIS REPLACES A POSITIONAL GUESS THAT WAS WRONG. See `mac_cameras()` for the
    full account: macOS's enumeration order is not OpenCV's, three separate macOS
    APIs agree with each other and disagree with reality, and Julien's covering test
    caught it. Position is now never used for anything.

    The method: for each index, ask for every camera's discriminating mode in turn
    and see which one comes back **exactly**. A camera cannot deliver a mode it does
    not have, so an exact match is an identification rather than an inference.

    ⚠️ This rests on one measured property of the backend: `cap.get(FRAME_WIDTH)`
    after a `set` returns what the camera ACTUALLY did, not what was asked for. That
    is how the 424x240 → 640x360 substitution was discovered in the first place. If a
    future OpenCV echoes the request instead, every camera would match every mode —
    which shows up here as *every* index being ambiguous, not as a wrong name.

    Costs one open per index plus a few format changes: a few seconds, paid once at
    startup. It is deliberately **not cached**: a replug can reorder indices without
    changing anything a cache could key on, and a stale camera map is the failure
    this whole section exists to prevent.
    """
    notes: list[str] = []
    if not cams:
        notes.append("⚠️  macOS reported no cameras at all — nothing to identify against.")
        return [], notes

    questions: dict[str, tuple[int, int]] = {}
    for cam in cams:
        mode = discriminating_mode(cam, [c for c in cams if c.unique_id != cam.unique_id])
        if mode is None:
            notes.append(f"⚠️  {cam.short} shares every mode with another camera, so it "
                         "cannot be identified by measurement. Tell it apart by covering "
                         "it and watching which index goes dark.")
        else:
            questions[cam.unique_id] = mode
    if not questions:
        notes.append("⛔ no camera has a mode of its own — identification is impossible "
                     "here. Use --index and confirm by looking at the picture.")
        return [], notes

    by_uid = {c.unique_id: c for c in cams}
    found: list[tuple[int, MacCamera | None]] = []
    claimed: dict[str, int] = {}
    for idx in range(limit if limit is not None else len(cams) + 1):
        cap = cv2.VideoCapture(idx)
        if not cap.isOpened():
            cap.release()
            continue
        hits = []
        for uid, (want_w, want_h) in questions.items():
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, want_w)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, want_h)
            got = (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
                   int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
            if got == (want_w, want_h):
                hits.append(uid)
        cap.release()

        if len(hits) == 1 and hits[0] not in claimed:
            cam = by_uid[hits[0]]
            w, h = questions[hits[0]]
            claimed[hits[0]] = idx
            found.append((idx, cam))
            notes.append(f"✅ index {idx} delivered {w}x{h}, which only {cam.short} offers.")
        elif len(hits) > 1:
            found.append((idx, None))
            notes.append(f"⛔ index {idx} matched {len(hits)} cameras at once "
                         f"({', '.join(by_uid[u].short for u in hits)}) — ambiguous, so it "
                         "is left unnamed.")
        elif hits:
            found.append((idx, None))
            notes.append(f"⛔ index {idx} claims to be {by_uid[hits[0]].short}, which index "
                         f"{claimed[hits[0]]} already answered for. Two indices cannot be "
                         "one camera — both are left unnamed.")
        else:
            found.append((idx, None))
            notes.append(f"⚠️  index {idx} matched no camera's own mode — unidentified.")

    missing = [c.short for c in cams if c.unique_id not in claimed]
    if missing:
        notes.append(f"⚠️  never found on any index: {', '.join(missing)}. A camera macOS "
                     "lists but OpenCV cannot open is normal for Continuity when the phone "
                     "is asleep.")
    return found, notes


class CameraLookupError(Exception):
    """Raised when `--camera` cannot be resolved to exactly one index."""


# Words worth accepting that do not appear in the macOS name string. `d405` is the
# obvious one: the device calls itself "Depth Camera 405".
ALIASES = {"d405": "405", "realsense": "realsense", "intel": "realsense",
           "c920": "c920", "logitech": "c920", "webcam": "webcam",
           "iphone": "iphone", "builtin": "macbook", "internal": "macbook"}


def _normalise(text: str) -> str:
    return "".join(ch for ch in text.lower() if ch.isalnum())


def resolve_camera(spec: str, cams: list[MacCamera] | None = None,
                   identified: list[tuple[int, MacCamera | None]] | None = None,
                   ) -> tuple[int, MacCamera | None]:
    """Turn `--camera d405` (or `--camera 2`) into an index, or refuse loudly.

    ⛔ Never falls back to index 0, and — since 2026-08-11 — never falls back to
    position either. The index comes from `identify_indices()`, which measures it.

    `cams` and `identified` exist to be injected by the tests. ⚠️ They must stay
    injectable: without them every test run would open all four cameras — waking
    Julien's iPhone over Continuity — and a test suite with side effects on hardware
    is one people stop running.
    """
    cams = mac_cameras() if cams is None else cams
    if not cams:
        raise CameraLookupError(
            "macOS reported no cameras, so --camera cannot be resolved.\n"
            "  Use --index N instead, and --list to see what is there.")

    if spec.isdigit():
        idx = int(spec)
        if identified is None:
            identified, _ = identify_indices(cams)
        for got_idx, cam in identified:
            if got_idx == idx:
                return idx, cam
        return idx, None

    want = _normalise(spec)
    want = _normalise(ALIASES.get(want, want)) if want in ALIASES else want
    matches = [c for c in cams
               if want in _normalise(c.name) or want in _normalise(c.usb or "")]
    if not matches:
        listing = "\n".join(f"    {c.short}" for c in cams)
        raise CameraLookupError(f"no camera matches {spec!r}. macOS lists:\n{listing}")
    if len(matches) > 1:
        listing = "\n".join(f"    {c.short}" for c in matches)
        raise CameraLookupError(f"{spec!r} matches more than one camera:\n{listing}\n"
                                "  Be more specific, or use --index.")

    cam = matches[0]
    if identified is None:
        print(f"  finding which index is the {cam.short} — this opens each camera briefly")
        identified, notes = identify_indices(cams)
    else:
        notes = []
    for idx, got in identified:
        if got is not None and got.unique_id == cam.unique_id:
            return idx, cam
    detail = "\n".join(f"    {n}" for n in notes)
    raise CameraLookupError(
        f"macOS lists {cam.short}, but it could not be identified on any OpenCV index.\n"
        f"{detail}\n"
        "  Run --list to see what each index actually shows, then use --index N.")


def list_cameras() -> None:
    """Everything known about every camera: what macOS lists, which index each one is
    **measured** to be, and what each index actually shows. Run after any replug."""
    cams = mac_cameras()
    if cams:
        print(f"macOS lists {len(cams)} camera(s). ⚠️ THIS ORDER IS NOT OpenCV's ORDER —")
        print("   assuming it was is the bug this tool was rewritten to fix.\n")
        for cam in cams:
            usb = f"   USB {cam.usb}" if cam.usb else ""
            modes = f"   {len(cam.modes)} modes" if cam.modes else "   (no mode list)"
            print(f"    {cam.short:<28s}{modes}{usb}".rstrip())
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
    print("      uv run scripts/camera_view.py --camera c920 --term")
    print("      uv run scripts/camera_view.py --camera d405 --term")
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


@dataclass(frozen=True)
class CellSize:
    """How many screen pixels one character cell occupies."""

    width: float
    height: float
    measured: bool

    @property
    def aspect(self) -> float:
        """Height divided by width. ~2 for most fonts, but not exactly, and the
        difference is a visible stretch."""
        return self.height / self.width


# Used when the terminal will not say. A 2:1 cell is the usual shape and the value
# the geometry always silently assumed before it was measurable.
ASSUMED_CELL = CellSize(8.0, 16.0, measured=False)


def cell_size() -> CellSize:
    """Measure the character cell in pixels, or fall back and **say so**.

    ⭐ WHY THIS EXISTS. Two things were being guessed at once, and both were visible
    on screen. The grid geometry assumed a cell is exactly twice as tall as it is
    wide, which stretches the picture whenever the font disagrees; and the image sent
    in kitty/iTerm2 mode was a fixed 480 px regardless of how large the pane was, so
    on a big terminal it was upscaled into a soft mess — **which is exactly what
    Julien reported on 2026-08-11: "the resolution is not great … pressing the
    numbers doesn't do anything."**

    Both are answered by one number the terminal already knows. `TIOCGWINSZ` returns
    `ws_xpixel`/`ws_ypixel` alongside the row and column count, so the cell is simply
    pixels ÷ cells. kitty and Ghostty fill those fields in; Apple Terminal reports
    zeros, and a piped or captured run has no terminal at all.

    ⚠️ `measured` is returned, never hidden. An assumed cell size is a fine default
    and a terrible silent one — the status line prints which it used, because a
    fallback you cannot see is indistinguishable from a bug (the same lesson as `b`
    silently doing nothing).
    """
    for stream in (sys.stdout, sys.stdin, sys.stderr):
        try:
            buf = fcntl.ioctl(stream.fileno(), termios.TIOCGWINSZ, b"\0" * 8)
        except (OSError, ValueError, AttributeError):
            continue
        rows, cols, xpix, ypix = struct.unpack("HHHH", buf)
        if rows and cols and xpix and ypix:
            return CellSize(xpix / cols, ypix / rows, measured=True)
    return ASSUMED_CELL


# ⭐ How many pixels wide the image sent to the terminal may get, per protocol.
# MEASURED 2026-08-11 on this Mac, encoding a detailed 16:9 frame (gradients, hard
# edges, text and grain — a realistic camera picture), best of five:
#
#     width   kitty: PNG level 1      iTerm2: JPEG q60
#      480     3.8 ms   277 KB/frame    0.1 ms    16 KB/frame
#      640     6.7 ms   521 KB          0.3 ms    26 KB
#      720    ~8   ms  ~650 KB          ~0.4 ms   ~32 KB
#      960    16.1 ms  1266 KB          0.6 ms    46 KB
#     1280    28.8 ms  2259 KB          1.1 ms    70 KB
#
# ⛔ **The kitty protocol has exactly one compressed format, and it is PNG** — `f`
# takes 24 (raw RGB), 32 (raw RGBA) or 100 (PNG). There is no JPEG. That single
# protocol fact is why the terminal view is soft: PNG of a photo is ~20x the bytes
# of a JPEG and ~25x the encode time, so detail costs frame rate in Ghostty/kitty in
# a way it simply does not in iTerm2. 720 px keeps encoding to ~25% of a 33 ms frame
# budget; 1280 px would spend 87% of it before a single byte is written.
#
# The numbers above are the *floor*: the terminal must also read and draw them, and
# writing ~650 KB into a pty every frame is not free. The live `draw ms` readout is
# the real measurement — these values only pick a sane starting point.
IMAGE_WIDTH_CAP = {"kitty": 720, "iterm": 1280}


def auto_image_width(cols: int, capture_width: int, mode: str, cell: CellSize) -> int:
    """Pixels wide to send, when the operator has not fixed it by hand.

    Three ceilings, and the smallest wins:

    1. **The box on screen** — `cols x cell.width` pixels. Sending more than the
       terminal can display is pure cost; the extra pixels are scaled straight back
       out again.
    2. **What the camera captured** — upscaling before transmission invents nothing
       and costs bytes.
    3. **The protocol's budget** — see `IMAGE_WIDTH_CAP`.

    ⭐ This is what makes keys 1-6 do something visible. They change the *capture*
    size, and previously the sent image was pinned at 480 px, so a sharper capture
    changed nothing on screen and the keys looked dead. With ceiling 2 in place, a
    bigger capture genuinely produces a bigger image — up to what the pane can show.
    """
    box = int(cols * cell.width)
    return max(64, min(box, capture_width, IMAGE_WIDTH_CAP.get(mode, 720)))


def terminal_grid(frame_aspect: float, scale: float = 1.0, margin_rows: int = 4,
                  cell_aspect: float | None = None) -> tuple[int, int]:
    """Character columns and rows to use, **preserving the picture's aspect ratio**.

    ⛔ THE BUG THIS FIXES. The first version returned the whole terminal and let
    `render_ansi` squash the frame into it, so a 16:9 camera came out stretched to
    whatever shape the pane happened to be. Julien's screenshot showed it clearly.

    The geometry: a cell is `k` times taller than it is wide, so a grid of `C`
    columns by `R` rows occupies a box whose aspect is `C / (R · k)`. To display a
    source of aspect `A` undistorted we therefore need `C = A · R · k`, and we take
    the largest such grid that fits inside the terminal.

    ⭐ `k` used to be hard-coded to 2 — right for a typical font, wrong for any
    other, and a silent stretch when wrong. It is now **measured** from the
    terminal's own pixel geometry (`cell_size()`) and passed in, falling back to 2
    only when the terminal will not say. This matters in both drawing modes: blocks
    mode paints two stacked pixels per cell, and image mode hands the terminal a
    `C x R` cell box that it scales the picture into.

    `scale` shrinks it below the maximum, for a small corner view rather than a
    full-pane one.

    ⚠️ `margin_rows` reserves space for the status lines underneath. Reserve too
    little and the picture pushes them off the bottom, or worse, scrolls the whole
    view every frame.
    """
    k = cell_aspect if cell_aspect and cell_aspect > 0 else ASSUMED_CELL.aspect
    size = shutil.get_terminal_size(fallback=(100, 30))
    max_cols = max(20, int(size.columns * scale))
    max_rows = max(6, int((size.lines - margin_rows) * scale))
    rows = min(max_rows, int(max_cols / (k * frame_aspect)))
    rows = max(6, rows)
    cols = min(max_cols, int(k * rows * frame_aspect))
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


def render_kitty(frame, cols: int, rows: int, max_width: int = 480, quiet: bool = True,
                 image_id: int | None = None, previous_id: int | None = None) -> str:
    """A real image, drawn by the kitty graphics protocol (kitty, Ghostty, Konsole).

    ⛔⭐ THE FLICKER, reported by Julien 2026-08-11: *"the image in the terminal is
    flickering because some frames seem to not be drawn."*

    Every frame used to begin `a=d,d=A` — **delete all images** — and only then
    transmit the new one. Between the delete and the new image being decoded there is
    nothing on screen, so at 30 fps the picture is blanked 30 times a second. Whether
    that reads as flicker depends on how fast the terminal decodes, which is why it
    got worse as the image got bigger.

    The fix is double buffering, the same idea as any graphics pipeline: pass
    `image_id` and `previous_id`, and the new image is **placed first, over the old
    one**, then the old id is deleted underneath it. There is no moment with nothing
    on screen. The caller alternates two ids each frame.

    ⚠️ Deleting still has to happen: `d=I` frees the image data as well as the
    placement, and without it a 30 fps stream leaks an image per frame into the
    terminal's memory. Omitting `previous_id` (as `--term-test` does) keeps the old
    delete-then-draw path, which is correct for a single still image.

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
    out: list[str] = []
    if image_id is None:
        # One-shot use (--term-test): nothing follows, so clear first and be simple.
        out.append(f"\x1b_Ga=d,d=A,{q}".rstrip(",") + "\x1b\\")
    ident = f"i={image_id}," if image_id is not None else ""
    for i, chunk in enumerate(chunks):
        first, last = i == 0, i == len(chunks) - 1
        ctrl = f"a=T,f=100,{ident}c={cols},r={rows},{q}" if first else ""
        out.append(f"\x1b_G{ctrl}m={0 if last else 1};{chunk}\x1b\\")
    if image_id is not None and previous_id is not None:
        # ⭐ Delete the OLD image only after the new one is on screen. See the
        # docstring: deleting first is what makes the picture blink.
        out.append(f"\x1b_Ga=d,d=I,i={previous_id},{q}".rstrip(",") + "\x1b\\")
    return "".join(out)


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
        sys.stdout.write(render_kitty(bars("KITTY"), cols, rows, quiet=False))
        sys.stdout.flush()
        reply = ""
        deadline = time.time() + 2.0
        while time.time() < deadline:
            if select.select([sys.stdin], [], [], 0.05)[0]:
                reply += sys.stdin.read(1)
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
        print("  the terminal said NOTHING -> it does not implement this protocol")
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
    sizes = key_sizes(cam)
    # None = size the image to the pane automatically. A number pins it, either from
    # --image-width or from the [ and ] keys.
    manual_width = args.image_width or None
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
                    sent_w = min(manual_width or auto_image_width(cols, w, mode, cell), w)
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
                              else f"sent {sent_w}x{sent_h}"
                                   f"{' (fixed)' if manual_width else ' (auto)'}")
                    cell_note = (f"cell {cell.width:.0f}x{cell.height:.0f}px"
                                 if cell.measured else
                                 f"cell {cell.width:.0f}x{cell.height:.0f}px ASSUMED")
                    # A draw cost past half the frame interval means the terminal, not
                    # the camera, is the bottleneck — and it is fixable from here.
                    budget = 1000.0 / max(1.0, grab.capture_fps())
                    warn = "   ⚠️ draw is over half the frame — press [ for less detail" \
                        if draw_ms > 0.5 * budget else ""
                    sys.stdout.write(
                        f"\x1b[0m\n{label}capture {w}x{h} · {detail} · {mode} · "
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
    label, chosen = "", None
    if args.camera:
        try:
            args.index, chosen = resolve_camera(args.camera)
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
