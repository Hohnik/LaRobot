#!/usr/bin/env python3
"""Tests for the frame writer — the disk half of camera-frames-into-recordings (item 48).

    uv run tests/test_frame_writer.py

⛔ No camera is ever opened (an agent cannot — FINDINGS §61.3); frames are synthetic arrays and the REAL JPEG path is still exercised, because `cv2.imencode` needs no device and asserting on a fake encoder alone would measure the wrong instrument (FINDINGS §36.3). The backpressure test drives the real queue with a deliberately blocked encoder, so the drop-oldest arithmetic is proven rather than believed.
"""

from __future__ import annotations

import json
import sys
import tempfile
import threading
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent

import numpy as np  # noqa: E402

from yam.cameras.frame import Frame  # noqa: E402
from yam.cameras.specs import (  # noqa: E402
    camera_dir_name,
    flatten_tokens,
    parse_indices,
    sim_camera_error,
)
from yam.cameras.writer import (  # noqa: E402
    FrameSink,
    FrameWriter,
    attach_frames_to_slot,
    clear_slot_frames,
    discard_frames,
    pending_frames_dir,
)

MS = 1_000_000  # ns per millisecond


def a_frame(seq: int, stamp_ms: int = 0, name: str = "cam") -> Frame:
    rgb = np.full((8, 8, 3), seq % 251, dtype=np.uint8)
    return Frame(camera_name=name, sequence=seq, camera_timestamp_ns=None,
                 host_timestamp_ns=stamp_ms * MS, rgb=rgb)


def test_every_natural_spelling_of_a_list_parses() -> None:
    # The three spellings the 2026-08-19 bench pass actually produced, verbatim (FINDINGS §71.1).
    assert flatten_tokens(["0,", "1,", "2"]) == ["0", "1", "2"], "the dictated '0, 1, 2'"
    assert flatten_tokens(["0", "1", "2"]) == ["0", "1", "2"], "plain spaces"
    assert flatten_tokens(["0,1,2"]) == ["0", "1", "2"], "one comma-joined token"
    assert parse_indices(["0,", "1", "2,"]) == [0, 1, 2]
    try:
        parse_indices(["0", "c920"])
    except ValueError as e:
        assert "c920" in str(e), "the refusal must name the bad token"
    else:
        raise AssertionError("a non-numeric index must refuse, never guess")


def test_camera_names_are_filesystem_safe_and_bare_numbers_say_what_they_are() -> None:
    assert camera_dir_name("d405:255323071773") == "d405-255323071773"
    assert camera_dir_name("c920") == "c920"
    assert camera_dir_name("2") == "cam2"
    assert camera_dir_name(camera_dir_name("d405:255")) == "d405-255", \
        "the sanitiser must be idempotent — session names pass through it twice"


def test_sim_refuses_cameras_and_every_other_combination_passes() -> None:
    assert sim_camera_error(True, "c920") is not None, \
        "a simulated take with real photographs is a mislabelling engine (item 48 trap ②)"
    assert sim_camera_error(True, None) is None
    assert sim_camera_error(False, "c920") is None
    assert sim_camera_error(False, None) is None


def test_the_writer_writes_real_jpegs_and_an_index_that_reads_back() -> None:
    import cv2

    d = Path(tempfile.mkdtemp())
    w = FrameWriter("wrist", d / "wrist")
    for i in range(1, 4):
        w.offer(a_frame(i, stamp_ms=33 * i))
    index = w.stop()
    assert index["written"] == 3 and index["dropped"] == 0 and index["flushed"]
    assert index["entries"] == [[1, 33 * MS], [2, 66 * MS], [3, 99 * MS]]
    on_disk = json.loads((d / "wrist" / "index.json").read_text())
    assert on_disk == index, "stop()'s return and the file must be one thing"
    img = cv2.imread(str(d / "wrist" / "000002.jpg"))
    assert img is not None and img.shape == (8, 8, 3), "the JPEG must decode as an image"


class BlockedEncoder:
    """An encoder the test can hold shut, so backpressure is deterministic."""

    def __init__(self) -> None:
        self.started = threading.Event()
        self.release = threading.Event()

    def __call__(self, rgb) -> bytes:  # noqa: ANN001
        self.started.set()
        assert self.release.wait(timeout=5.0), "the test forgot to release the encoder"
        return b"jpeg-bytes"


def test_a_full_queue_drops_the_oldest_and_counts_it_and_stop_flushes() -> None:
    d = Path(tempfile.mkdtemp())
    enc = BlockedEncoder()
    w = FrameWriter("cam", d / "cam", encode=enc, queue_frames=2)
    w.offer(a_frame(1))
    assert enc.started.wait(timeout=5.0), "the thread must take frame 1 into the encoder"
    w.offer(a_frame(2))
    w.offer(a_frame(3))          # queue now holds [2, 3]
    w.offer(a_frame(4))          # full → 2 (the OLDEST) is dropped, queue holds [3, 4]
    assert w.dropped == 1, "exactly one frame was dropped, and it was counted"
    enc.release.set()
    index = w.stop()
    assert index["written"] == 3 and index["dropped"] == 1 and index["flushed"]
    assert [seq for seq, _ in index["entries"]] == [1, 3, 4], \
        "the dropped frame is the oldest QUEUED one, so the newest data survives a stall"


def test_an_encode_error_costs_one_image_and_is_counted_never_raised() -> None:
    d = Path(tempfile.mkdtemp())
    calls = {"n": 0}

    def flaky(rgb) -> bytes:  # noqa: ANN001
        calls["n"] += 1
        if calls["n"] == 2:
            raise ValueError("scripted failure")
        return b"ok"

    w = FrameWriter("cam", d / "cam", encode=flaky)
    for i in range(1, 4):
        w.offer(a_frame(i))
    index = w.stop()
    assert index["written"] == 2 and index["write_errors"] == 1, \
        "the failed frame is a counted error, not a crash and not a silent gap"
    assert [seq for seq, _ in index["entries"]] == [1, 3]


def test_the_sink_forwards_only_fresh_frames() -> None:
    d = Path(tempfile.mkdtemp())
    seen: list[int] = []

    def spy(rgb) -> bytes:  # noqa: ANN001
        seen.append(int(rgb[0, 0, 0]))
        return b"ok"

    sink = FrameSink(d, ["cam"], encode=spy)
    f1, f2 = a_frame(1), a_frame(2)
    sink.offer({"cam": f1})
    sink.offer({"cam": f1})      # CaptureSet repeats a slow camera — the sink must not store it twice
    sink.offer({"cam": None})    # before-first-frame samples pass through as None
    sink.offer({"cam": f2})
    reports = sink.stop()
    assert reports["cam"]["written"] == 2, f"one write per DISTINCT frame, saw {seen}"


def test_take_directories_move_replace_and_clear_honestly() -> None:
    root = Path(tempfile.mkdtemp())
    pending = pending_frames_dir(root, "20260819_120000")
    (pending / "cam").mkdir(parents=True)
    (pending / "cam" / "000001.jpg").write_bytes(b"new")
    stale = root / "frames" / "4" / "cam"
    stale.mkdir(parents=True)
    (stale / "000009.jpg").write_bytes(b"stale")

    dest = attach_frames_to_slot(root, pending, "4")
    assert dest == root / "frames" / "4"
    assert (dest / "cam" / "000001.jpg").read_bytes() == b"new"
    assert not (dest / "cam" / "000009.jpg").exists(), \
        "slot 4's STALE frames must vanish — old images beside a new recording lie"
    assert not pending.exists(), "attach is a move, not a copy"

    assert clear_slot_frames(root, "4") is True, "a frameless save clears the slot's frames"
    assert not dest.exists()
    assert clear_slot_frames(root, "4") is False, "nothing there → nothing cleared, honestly"

    p2 = pending_frames_dir(root, "x")
    p2.mkdir(parents=True)
    discard_frames(p2)
    assert not p2.exists()
    discard_frames(p2)           # already gone — must be harmless
    discard_frames(None)         # no pending take — must be harmless


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
