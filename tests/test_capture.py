#!/usr/bin/env python3
"""Tests for the camera capture layer — frames, the reader thread, the sampling set.

    uv run tests/test_capture.py

⛔ No camera is ever opened here, by construction: an agent can never run a camera (FINDINGS §61.3), so everything in `yam.cameras` that CAN be proven headless must be — the fakes below drive the real `CaptureSet` and the real `FrameGrabber` (with a scripted capture object), and only the one-line OpenCV glue in `apps/capture_probe.py` waits for Julien's bench command.
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent

from yam.cameras.capture import CameraReport, CaptureSet, gap_stats_ms  # noqa: E402
from yam.cameras.frame import Frame  # noqa: E402
from yam.cameras.grabber import FrameGrabber, fourcc_name  # noqa: E402

MS = 1_000_000  # ns per millisecond


class FakeGrabber:
    """A scripted reader: the test controls exactly what `newest_stamped` returns."""

    def __init__(self):  # noqa: ANN204
        self.frame, self.seq, self.stamp = None, 0, 0
        self.stopped = False

    def store(self, frame, stamp_ns) -> None:  # noqa: ANN001
        self.frame, self.seq, self.stamp = frame, self.seq + 1, stamp_ns

    def newest_stamped(self):  # noqa: ANN201
        return self.frame, self.seq, self.stamp

    def capture_fps(self) -> float:
        return 30.0

    def stop(self) -> None:
        self.stopped = True


def test_a_frame_is_honest_about_what_it_cannot_know() -> None:
    g = FakeGrabber()
    g.store("img-1", stamp_ns=5 * MS)
    s = CaptureSet({"wrist": g})
    frame = s.sample()["wrist"]
    assert isinstance(frame, Frame)
    assert frame.camera_timestamp_ns is None, \
        "this stack cannot read a device timestamp, and the record must say so"
    assert frame.depth is None, "no depth over UVC on macOS (FINDINGS §63.0)"
    assert frame.host_timestamp_ns == 5 * MS, "the stamp is the STORE time, not sample time"
    assert frame.camera_name == "wrist" and frame.sequence == 1 and frame.rgb == "img-1"


def test_sampling_before_the_first_frame_says_none() -> None:
    s = CaptureSet({"scene": FakeGrabber()})
    assert s.sample()["scene"] is None
    r = s.report()[0]
    assert r.empty == 1 and r.fresh == 0 and r.samples == 1


def test_a_repeat_keeps_its_sequence_so_the_consumer_can_tell() -> None:
    g = FakeGrabber()
    g.store("img-1", stamp_ns=1 * MS)
    s = CaptureSet({"wrist": g})
    first, second = s.sample()["wrist"], s.sample()["wrist"]
    assert first.sequence == second.sequence == 1, \
        "two samples of one frame must carry ONE sequence number"
    g.store("img-2", stamp_ns=34 * MS)
    third = s.sample()["wrist"]
    assert third.sequence == 2
    r = s.report()[0]
    assert (r.samples, r.fresh, r.duplicates, r.empty) == (3, 2, 1, 0)


def test_a_slow_camera_never_holds_up_a_fast_one() -> None:
    fast, slow = FakeGrabber(), FakeGrabber()
    s = CaptureSet({"fast": fast, "slow": slow})
    slow.store("s-1", stamp_ns=1 * MS)
    for i in range(6):                       # the fast camera produces 6 frames
        fast.store(f"f-{i}", stamp_ns=(1 + i) * MS)
        out = s.sample()
        assert out["fast"].rgb == f"f-{i}", "the fast camera is always current"
        assert out["slow"].rgb == "s-1", "the slow camera repeats, never blocks"
    reports = {r.name: r for r in s.report()}
    assert reports["fast"].fresh == 6 and reports["fast"].duplicates == 0
    assert reports["slow"].fresh == 1 and reports["slow"].duplicates == 5


def test_gap_stats_measure_the_blind_stretches() -> None:
    mean, worst = gap_stats_ms([0, 33 * MS, 66 * MS, 166 * MS])
    assert abs(worst - 100.0) < 1e-9, "the 100 ms stall is the worst gap"
    assert abs(mean - (33 + 33 + 100) / 3) < 1e-9
    assert gap_stats_ms([]) == (0.0, 0.0) and gap_stats_ms([5]) == (0.0, 0.0), \
        "fewer than two frames means no gap was ever observable"


def test_the_report_reaches_every_camera_and_stop_stops_them_all() -> None:
    a, b = FakeGrabber(), FakeGrabber()
    s = CaptureSet({"a": a, "b": b})
    assert [r.name for r in s.report()] == ["a", "b"]
    assert all(isinstance(r, CameraReport) for r in s.report())
    s.stop()
    assert a.stopped and b.stopped


class ScriptedCap:
    """A `cv2.VideoCapture` stand-in: `read()` blocks briefly like AVFoundation does."""

    def __init__(self, frames: int, period_s: float = 0.005):  # noqa: ANN204
        self.n, self.period, self.given = frames, period_s, 0
        self.released = False

    def read(self):  # noqa: ANN201
        time.sleep(self.period)              # the blocking read the thread exists for
        if self.given >= self.n:
            return False, None
        self.given += 1
        return True, f"frame-{self.given}"

    def release(self) -> None:
        self.released = True


def test_the_real_grabber_thread_stores_stamps_and_stops_cleanly() -> None:
    grab = FrameGrabber(ScriptedCap(frames=8))
    deadline = time.perf_counter() + 2.0
    while time.perf_counter() < deadline:
        frame, seq, stamp = grab.newest_stamped()
        if seq >= 8:
            break
        time.sleep(0.005)
    frame, seq, stamp = grab.newest_stamped()
    assert seq == 8 and frame == "frame-8", f"expected all 8 frames, saw seq {seq}"
    assert stamp > 0, "every stored frame carries a store-time stamp"
    f2, s2 = grab.newest()
    assert (f2, s2) == (frame, seq), "the confirmed 2-tuple API is unchanged"
    grab.stop()
    assert grab._cap.released, "stop() must release the device"  # noqa: SLF001


def test_fourcc_name_decodes_and_refuses() -> None:
    code = ord("M") | (ord("J") << 8) | (ord("P") << 16) | (ord("G") << 24)
    assert fourcc_name(code) == "MJPG"
    assert fourcc_name(-1) == "?" and fourcc_name(0) == "?"


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
