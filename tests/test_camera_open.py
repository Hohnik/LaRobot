#!/usr/bin/env python3
"""Tests for `yam/cameras/open.py` — proving a camera delivers instead of believing it.

    uv run tests/test_camera_open.py

⛔ No camera is opened (FINDINGS §61.3 on macOS, and a test must not depend on hardware anyway). The fake handle below reproduces the two real failures of 2026-08-19 exactly: a device that ACCEPTS 1280x720 and delivers nothing at it, and a device that delivers at a size other than the one it was asked for. Both were silent on screen, and both are what the assertions here are aimed at ([FINDINGS §76](../docs/FINDINGS.md)).
"""

from __future__ import annotations

import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent

import numpy as np  # noqa: E402

from yam.cameras.open import (  # noqa: E402
    FIRST_FRAME_S,
    MEASURE_S,
    SIZE_LADDER,
    SLOW_FPS,
    TARGET_FPS,
    WARMUP_S,
    CameraOpen,
    open_measured,
)

#: Tiny windows, so the suite does not spend 1.4 s per fake camera. The real defaults are
#: asserted separately below, because a test that overrides every default proves nothing
#: about the defaults.
FAST = {"first_frame_s": 0.3, "warmup_s": 0.01, "measure_s": 0.05}


class FakeCap:
    """An OpenCV-shaped handle that delivers frames only at sizes it actually supports.

    `delivers` maps `(width, height)` to a frames-per-second figure. A size not in it accepts the `set()` calls and returns `(False, None)` from every `read()`, which is what the D405's colour node did at 1280x720 on Linux.

    `frame_size` overrides the shape of the returned array, so a handle can claim one size through `get()` and hand back another. That is the case the old code could not see, because it printed `get()` and never looked at a frame.
    """

    def __init__(self, delivers: dict[tuple[int, int], float],
                 frame_size: tuple[int, int] | None = None, fourcc: str = "MJPG",
                 dead_reads: int = 0) -> None:
        self.delivers = delivers
        self.frame_size = frame_size
        self._fourcc = fourcc
        # ⭐ `dead_reads` reproduces the real defect of 2026-08-19: a camera's first frames do
        # not arrive the instant the format is set. The D405 took 0.56 s. A handle that
        # answers instantly cannot show that, and that is exactly why the first version of
        # `open_measured` reported 8.6 fps for a 30 fps camera.
        self.dead_reads = dead_reads
        self.asked = (0, 0)
        self.set_order: list[str] = []
        self.reads = 0

    # --- the OpenCV surface this module uses ------------------------------------------
    def set(self, prop: int, value: float) -> bool:  # noqa: ANN001
        import cv2

        names = {cv2.CAP_PROP_FOURCC: "fourcc", cv2.CAP_PROP_FRAME_WIDTH: "width",
                 cv2.CAP_PROP_FRAME_HEIGHT: "height", cv2.CAP_PROP_FPS: "fps",
                 cv2.CAP_PROP_BUFFERSIZE: "buffer"}
        self.set_order.append(names.get(prop, str(prop)))
        if prop == cv2.CAP_PROP_FRAME_WIDTH:
            self.asked = (int(value), self.asked[1])
        elif prop == cv2.CAP_PROP_FRAME_HEIGHT:
            self.asked = (self.asked[0], int(value))
        return True

    def get(self, prop: int) -> float:  # noqa: ANN001
        import cv2

        if prop == cv2.CAP_PROP_FOURCC:
            code = 0
            for i, ch in enumerate(self._fourcc[:4]):
                code |= ord(ch) << (8 * i)
            return float(code)
        if prop == cv2.CAP_PROP_FRAME_WIDTH:
            return float(self.asked[0])
        if prop == cv2.CAP_PROP_FRAME_HEIGHT:
            return float(self.asked[1])
        return 0.0

    def read(self):  # noqa: ANN201
        self.reads += 1
        if self.asked not in self.delivers:
            return False, None
        if self.reads <= self.dead_reads:
            return False, None
        w, h = self.frame_size or self.asked
        return True, np.zeros((h, w, 3), dtype=np.uint8)


# --- the happy path ------------------------------------------------------------------

def test_a_camera_that_works_at_the_first_size_does_not_step_down():
    cap = FakeCap({(1280, 720): 30.0})
    got = open_measured(cap, **FAST)
    assert got is not None, "a working camera must open"
    assert (got.width, got.height) == (1280, 720), got
    assert got.stepped_down is False, "no step-down happened, so it must not say one did"
    assert got.asked == (1280, 720), got.asked


def test_the_size_comes_from_the_FRAME_not_from_what_the_handle_claims():
    # ⛔ The real defect: `cap.get` answered 1280x720 while nothing arrived at all. Here the
    # handle claims 1280x720 and hands back 640x480 frames. The measured answer must be 640x480.
    cap = FakeCap({(1280, 720): 30.0}, frame_size=(640, 480))
    got = open_measured(cap, **FAST)
    assert (got.width, got.height) == (640, 480), (
        f"reported {got.width}x{got.height} — that is `cap.get`, not the frame")


def test_the_format_is_read_back_when_the_driver_says():
    cap = FakeCap({(1280, 720): 30.0}, fourcc="MJPG")
    assert open_measured(cap, **FAST).fourcc == "MJPG"


def test_a_driver_that_will_not_name_its_format_reports_a_question_mark():
    # macOS/AVFoundation returns -1 for CAP_PROP_FOURCC. That is unknown, never a fault.
    cap = FakeCap({(1280, 720): 30.0}, fourcc="")
    assert open_measured(cap, **FAST).fourcc == "?"


# --- the D405 case: accepts the size, delivers nothing at it -------------------------

def test_a_camera_that_accepts_1280x720_and_delivers_nothing_STEPS_DOWN():
    cap = FakeCap({(848, 480): 30.0})
    got = open_measured(cap, **FAST)
    assert got is not None, "848x480 works on this handle, so the open must succeed"
    assert (got.width, got.height) == (848, 480), got
    assert got.stepped_down is True, "it stepped down, and the operator must be told"
    assert got.asked == (848, 480), "asked names the size that finally worked"


def test_the_ladder_is_tried_largest_first():
    cap = FakeCap({(1280, 720): 30.0, (640, 480): 30.0})
    got = open_measured(cap, **FAST)
    assert (got.width, got.height) == (1280, 720), "the largest working size must win"


def test_a_camera_that_delivers_at_NO_size_returns_None_so_the_caller_can_refuse():
    cap = FakeCap({})
    assert open_measured(cap, **FAST) is None, (
        "zero frames at every size is a broken session and must not open")
    assert cap.reads > 0, "it must actually have tried to read, not just called set()"


def test_every_size_in_the_ladder_is_tried_before_giving_up():
    cap = FakeCap({})
    open_measured(cap, **FAST)
    widths = [n for n in cap.set_order if n == "width"]
    assert len(widths) == len(SIZE_LADDER), (
        f"set width {len(widths)} time(s) for a ladder of {len(SIZE_LADDER)}")


# --- the C920 case: works, but far too slowly ----------------------------------------

def test_a_slow_camera_opens_and_is_flagged_slow():
    # 10 fps is what a C920 gives at 1280x720 in YUYV. It works; the data is thin.
    got = CameraOpen(width=1280, height=720, fps=10.0, fourcc="YUYV", stepped_down=False,
                     asked=(1280, 720), frames=7, window_s=0.7)
    assert got.slow is True, f"{got.fps} fps must count as slow against {SLOW_FPS}"


def test_a_camera_at_the_target_rate_is_not_flagged_slow():
    got = CameraOpen(width=1280, height=720, fps=TARGET_FPS, fourcc="MJPG",
                     stepped_down=False, asked=(1280, 720), frames=21, window_s=0.7)
    assert got.slow is False


def test_the_startup_line_names_only_measured_things():
    got = CameraOpen(width=848, height=480, fps=29.6, fourcc="YUYV", stepped_down=True,
                     asked=(848, 480), frames=21, window_s=0.7)
    line = got.line()
    assert "848x480" in line and "29.6 fps" in line and "YUYV" in line, line
    assert "1280" not in line, "the line must never carry a size that was only requested"


# --- the configuration order, which is the whole C920 defect -------------------------

def test_MJPG_is_requested_BEFORE_the_size():
    # ⛔ This ordering is the fix for the 10 fps recording. Setting the size first leaves a
    # C920 in YUYV, where its own firmware caps 1280x720 at 10 fps (measured on the station,
    # FINDINGS §76). The order is the property worth locking down in a test.
    cap = FakeCap({(1280, 720): 30.0})
    open_measured(cap, **FAST)
    assert cap.set_order[0] == "fourcc", (
        f"first set() was {cap.set_order[0]!r}; MJPG must come before the size")
    assert cap.set_order.index("fourcc") < cap.set_order.index("width")


# --- the rate window, which is where the first version was wrong ---------------------

def test_a_camera_whose_first_frames_come_LATE_is_still_measured_at_its_real_rate():
    # ⛔ THE REAL DEFECT, reproduced. The first version measured the rate from the instant the
    # format was set, so a camera that takes a moment to start reported a third of its true
    # rate. On the station a 30 fps C920 came out as 8.6 fps, and the "too slow" warning
    # would then have fired in every healthy session (FINDINGS §76.3).
    cap = FakeCap({(1280, 720): 30.0}, dead_reads=25)
    got = open_measured(cap, first_frame_s=1.0, warmup_s=0.02, measure_s=0.05)
    assert got is not None, "25 dead reads then frames must still count as working"
    assert got.slow is False, (
        f"reported {got.fps:.1f} fps for a handle that delivers every read after warm-up — "
        "the start-up latency is leaking into the rate again")


def test_the_warm_up_frames_are_not_counted_in_the_rate():
    cap = FakeCap({(1280, 720): 30.0})
    got = open_measured(cap, first_frame_s=0.3, warmup_s=0.05, measure_s=0.05)
    # window_s carries the MEASURED window, so it must reflect measure_s and never the total.
    assert got.window_s < 0.2, f"window_s {got.window_s:.3f} looks like it includes warm-up"


def test_the_defaults_are_the_measured_ones_and_not_placeholders():
    # ⭐ FIRST_FRAME_S is 4x the 0.56 s the D405 actually took. If someone shrinks it below
    # that measurement, a working camera silently steps down a size.
    assert FIRST_FRAME_S >= 2.0, f"{FIRST_FRAME_S}s is under the measured 0.56 s plus headroom"
    assert WARMUP_S > 0, "a zero warm-up is the defect this constant exists to prevent"
    assert MEASURE_S >= 0.5, f"{MEASURE_S}s cannot tell 30 fps from 20"


def main() -> int:
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failed = 0
    for fn in tests:
        try:
            fn()
        except AssertionError as e:  # noqa: PERF203
            failed += 1
            print(f"✗ {fn.__name__}: {e}")
        else:
            print(f"✓ {fn.__name__}")
    print(f"\n{len(tests) - failed}/{len(tests)} passed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
