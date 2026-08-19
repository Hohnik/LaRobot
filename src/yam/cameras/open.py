"""Open a camera and PROVE it delivers, instead of asking it politely and believing the answer.

⛔⭐⭐ WHY THIS EXISTS. On 2026-08-19 the first camera-carrying session on the Linux station printed this, and every word of it was wrong in a different way ([FINDINGS §76](../../../docs/FINDINGS.md)):

    📷 c920 open on /dev/video0 (index 0), delivering 1280x720.
    📷 d405-260323072846 open on /dev/video6 (index 6), delivering 1280x720.

The D405 delivered **zero frames** for the whole session. The C920 delivered **10 fps, not 30**. Nothing raised anything, and the word on screen was "delivering".

⭐ THE TWO CAUSES, both measured on the station rather than guessed.

**1. The Linux path never asked for MJPG.** `apps/camera_view.py::configure_camera` sets `CAP_PROP_FOURCC` to MJPG before it sets the size, and its own comment says the line is "load-bearing on Linux, where this rig is ultimately headed". The session's Linux camera path was then written with three bare `set()` calls and no codec request. So OpenCV took the device's first advertised format, which on a C920 is `YUYV`, and **a C920 at 1280x720 in YUYV offers a maximum of 10 fps** while MJPG offers 30. The knowledge was already written down, in a comment, in a file this path did not import. Two copies of a configuration step is how that happens, so there is one copy now and both platforms call it.

**2. A camera can accept a size and then deliver nothing at it, INTERMITTENTLY.** The D405's colour node advertises 1280x720 at 30 fps in `v4l2-ctl --list-formats-ext`, `cap.set` accepts it, and `cap.get` reads 1280x720 back. That mode has now been measured **dead twice and alive twice on the same camera in one afternoon**: `v4l2-ctl --stream-mmap` first blocked for four minutes with zero frames, then a timed retry got zero again, and later the same command reported a steady 30.01 fps. 848x480 and below have never once failed. The USB link is 5 Gbit/s and the format is advertised, so bandwidth and format are both ruled out by measurement.

⚠️ **What changed in between is not established.** The single new variable was librealsense opening and configuring the camera, and the mode has worked in every plain-V4L2 process since. That is an inference from one before-and-after pair rather than proof. The alternative nobody can rule out from here: the first attempt was killed with `pkill` while blocked, which may have left that one mode wedged until something re-initialised it.

⭐ **An intermittent mode is a BETTER argument for this module than a dead one.** A size that works on Monday and not on Tuesday is exactly what a screen message built from `cap.get` hides perfectly, and exactly what reading a real frame catches every time.

⭐ SO THE RULE HERE IS: a camera is open when a real frame has arrived, and the size printed on screen comes out of that frame's own shape. Never from `cap.get`, which reports what was requested.

⚠️ ZERO FRAMES REFUSES, A SLOW RATE WARNS, and the split is deliberate. Zero frames means the session would record nothing at all, which is a broken session and no operator wants it. A low rate is degraded data rather than a broken session: the exporter fills 30 ticks a second and a 10 fps camera repeats frames across them. That is the operator's call to accept or fix, so it is a loud warning with the consequence named ([HANDOFF §4](../../../docs/HANDOFF.md) rule 4 is about hazards, and a slow camera is not one).
"""

from __future__ import annotations

import time
from dataclasses import dataclass

__all__ = ["SIZE_LADDER", "TARGET_FPS", "SLOW_FPS", "FIRST_FRAME_S", "WARMUP_S",
           "MEASURE_S", "CameraOpen", "await_first_frame", "configure", "open_measured"]

#: Sizes tried in order, largest first, until one actually delivers a frame.
#:
#: ⭐ 1280x720 stays first, so nothing that works today changes. 848x480 is next because it is the largest size the D405 colour node was MEASURED to stream on Linux. 640x480 and 424x240 are the fallbacks below that; both are offered by every camera on this rig.
#:
#: ⚠️ The ladder only ever goes DOWN within one open handle. Raising a size after lowering it confuses some V4L2 drivers, and there is never a reason to.
SIZE_LADDER: list[tuple[int, int]] = [(1280, 720), (848, 480), (640, 480), (424, 240)]

#: What the episode exporter assumes: 30 ticks a second, 33,333,333 ns apart.
TARGET_FPS = 30.0

#: Below this, the warning fires. Two thirds of the target: a camera measured at 20 fps is within sampling noise of 30, and one at 10 is not.
SLOW_FPS = 20.0

#: How long to wait for the camera's FIRST frame before giving up on this size.
#:
#: ⭐ MEASURED, not chosen: the D405 at 1280x720 on the station takes **0.56 s** to produce its first frame after the format is set. This is four times that, because the cost of being wrong is stepping a working camera down to a smaller size for no reason.
FIRST_FRAME_S = 2.5

#: Frames read and DISCARDED before the rate is measured.
#:
#: ⛔⭐ WHY THIS EXISTS, and it is a defect the unit tests could not see. The first version measured the rate over one 0.7 s window starting at the format change. On the real cameras that reported **8.6 fps for a C920 that runs at 30**, because the window was mostly the camera starting up. So the "this camera is too slow" warning would have fired on every healthy camera in every session, which is the cry-wolf failure. The fake handle in `tests/test_camera_open.py` returns instantly and could never show it; only running it on the station did.
WARMUP_S = 0.4

#: The window the reported rate is measured over, after the warm-up.
#:
#: ⚠️ At 30 fps this is about 30 frames, which is enough to tell 30 from 10 and nowhere near enough to tell 29 from 30. That is the right precision for the question being asked.
MEASURE_S = 1.0


@dataclass(frozen=True)
class CameraOpen:
    """What actually happened when a camera was opened. Every field is measured, none is claimed.

    `width`/`height` come from a real frame's shape. `fps` is frames counted over `window_s`. `fourcc` is the format read back from the driver, or `"?"` when it will not say (macOS/AVFoundation does not). `stepped_down` is True when the first size in the ladder delivered nothing.
    """

    width: int
    height: int
    fps: float
    fourcc: str
    stepped_down: bool
    asked: tuple[int, int]
    frames: int
    window_s: float

    @property
    def slow(self) -> bool:
        return self.fps < SLOW_FPS

    def line(self) -> str:
        """One line for the session's startup screen, saying only what was measured."""
        return (f"{self.width}x{self.height} at {self.fps:.1f} fps"
                f"{'' if self.fourcc == '?' else f' in {self.fourcc}'}")


def configure(cap, width: int, height: int, fps: int = 30):  # noqa: ANN001, ANN201
    """Apply capture settings to an already-open handle. THE only copy of this step.

    ⭐ MJPG FIRST, THEN THE SIZE. Setting the size before the codec leaves a C920 in uncompressed YUYV, where 1280x720 is capped at 10 fps by the camera itself and 1080p does not fit down USB 2 at all. It reads on screen as lag and it is a format problem.

    ⚠️ MEASURED ON macOS 2026-08-11: the codec request makes no measurable difference there and `CAP_PROP_FOURCC` reads back as -1, so AVFoundation is choosing the format itself and choosing well. ⭐ MEASURED ON LINUX 2026-08-19: it makes all the difference, 10 fps against 30.
    """
    import cv2  # noqa: PLC0415 — lazy, so fakes and tests never touch OpenCV

    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)
    try:
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)   # ignored by some backends; harmless
    except Exception:  # noqa: BLE001, S110
        pass
    return cap


def _fourcc(cap) -> str:  # noqa: ANN001
    """The four letters the driver says it is delivering, or `"?"` when it will not say."""
    try:
        import cv2  # noqa: PLC0415

        code = int(cap.get(cv2.CAP_PROP_FOURCC))
    except Exception:  # noqa: BLE001
        return "?"
    if code <= 0:
        return "?"
    name = "".join(chr((code >> (8 * i)) & 0xFF) for i in range(4)).strip()
    return name if name.isprintable() and name else "?"


def await_first_frame(cap, deadline_s: float):  # noqa: ANN001, ANN201
    """Read until ONE real frame arrives, or `deadline_s` passes. The frame, or None.

    ⭐ This is the delivery proof, and it is separate from the rate measurement on purpose. Whether a camera works at all and how fast it runs are two different questions, and answering them in one window got the second one wrong.

    ⚠️ A wall-clock deadline rather than a frame count. A frame count would wait for ever on the camera that started all of this, because it delivered none.
    """
    end = time.perf_counter() + deadline_s
    while time.perf_counter() < end:
        ok, got = cap.read()
        if ok and got is not None:
            return got
        # ⚠️ A failed read on V4L2 returns after its own select() timeout, so this is not a
        # busy loop. The small sleep only matters on a backend that fails fast.
        time.sleep(0.002)
    return None


def _measure_rate(cap, warmup_s: float, measure_s: float) -> tuple[object, int, float]:  # noqa: ANN001
    """Discard `warmup_s` of frames, then count for `measure_s`. `(last frame, count, seconds)`."""
    end = time.perf_counter() + warmup_s
    while time.perf_counter() < end:
        cap.read()
    frame, count = None, 0
    t0 = time.perf_counter()
    end = t0 + measure_s
    while time.perf_counter() < end:
        ok, got = cap.read()
        if ok and got is not None:
            frame, count = got, count + 1
    return frame, count, max(1e-6, time.perf_counter() - t0)


def open_measured(cap, sizes: list[tuple[int, int]] | None = None,  # noqa: ANN001
                  fps: int = 30, first_frame_s: float = FIRST_FRAME_S,
                  warmup_s: float = WARMUP_S,
                  measure_s: float = MEASURE_S) -> CameraOpen | None:
    """Configure `cap`, then prove it delivers and measure how fast. `None` when no size does.

    Walks `sizes` largest first. For each one it configures the handle and waits up to `first_frame_s` for one real frame. The first size that produces a frame wins. It then discards `warmup_s` of frames and counts for `measure_s`, and everything reported comes out of that.

    ⭐ THREE PHASES, because the first version had two and got the rate wrong. Waiting for a first frame answers "does this work". Discarding a warm-up removes the camera's start-up latency, which otherwise dominates. Only then is the rate worth reporting.

    ⛔ Returns None rather than raising, so the caller owns the refusal message. The caller knows which `--cameras` spec this was and what the operator should do about it. This function only knows that nothing arrived.
    """
    ladder = list(sizes if sizes is not None else SIZE_LADDER)
    for i, (w, h) in enumerate(ladder):
        configure(cap, w, h, fps)
        first = await_first_frame(cap, first_frame_s)
        if first is None:
            continue
        frame, count, dt = _measure_rate(cap, warmup_s, measure_s)
        # ⚠️ Fall back to the proof frame for the SIZE. A camera can deliver one frame and
        # then stall, and the size it delivered is still the truth about the size.
        shape_from = frame if frame is not None else first
        height, width = shape_from.shape[:2]
        return CameraOpen(width=int(width), height=int(height),
                          fps=count / dt, fourcc=_fourcc(cap),
                          stepped_down=i > 0, asked=(w, h),
                          frames=count, window_s=dt)
    return None
