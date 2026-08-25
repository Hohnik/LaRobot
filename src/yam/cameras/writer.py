"""Write recorded camera frames to disk without ever blocking the control loop.

⭐⭐ THE DESIGN IS ROADMAP §8.2 ITEM 48, settled 2026-08-19 before any of this was built: while a take is being recorded the session samples `CaptureSet` and hands each FRESH frame to one writer thread per camera, which JPEG-encodes and writes `recordings/frames/<slot>/<camera>/<seq>.jpg` plus an `index.json` of `(seq, host_stamp_ns)`. Frames live on disk beside the recording; the recording JSON never carries a pixel. The recorder stamps the directory and the per-camera counts into its own `meta`, and `yam/episode.py` later joins the frames to the 30 Hz ticks by nearest stamp.

⛔ THE TRAP THIS FILE EXISTS TO NOT FALL INTO (item 48 trap ①): the writer must never block the 90 Hz loop. `offer()` is a lock-free hand-off by reference (OpenCV's `read()` returns a fresh array per frame, so the reference is safe to keep), and a full queue drops the OLDEST queued frame WITH A COUNT. A silent drop would be the FINDINGS §0 pattern — a dataset with blind gaps and a straight face — so `dropped` is carried in the index, in the recording meta, and printed at the stop line.

⚠️ WHY DROP-OLDEST rather than drop-newest: when encoding falls behind, the freshest frame is the one nearest the joint data being recorded right now. Dropping the newest would make every stall stretch the dataset's past instead of trimming it, and the episode join would then pair current joints with stale pixels — a lie with a timestamp on it.
"""

from __future__ import annotations

import json
import queue
import shutil
import threading
from pathlib import Path
from typing import Any, Callable

from yam.cameras.frame import Frame
from yam.cameras.specs import camera_dir_name

__all__ = ["FrameWriter", "FrameSink", "encode_jpeg",
           "pending_frames_dir", "attach_frames_to_slot", "discard_frames",
           "clear_slot_frames", "FRAMES_DIRNAME"]

#: Where frames live, under the recordings directory: `recordings/frames/<slot>/<camera>/`.
FRAMES_DIRNAME = "frames"

#: Frames waiting to be encoded, per camera. At ~30 fps and a few ms per JPEG the queue idles near empty; it fills only when the disk or CPU stalls, and then the oldest waiting frame is dropped and counted (see the module docstring for why oldest).
QUEUE_FRAMES = 8

#: JPEG quality. 90 keeps a 1280x720 frame around 100-200 kB and visually clean; the rebuild can tune this against its own storage budget, and the value is recorded in every index so a dataset knows what it holds.
JPEG_QUALITY = 90


def encode_jpeg(rgb: Any, quality: int = JPEG_QUALITY) -> bytes:
    """One BGR array as JPEG bytes. Raises rather than returning something empty."""
    import cv2  # noqa: PLC0415 — lazy, so headless tests can inject their own encoder

    ok, buf = cv2.imencode(".jpg", rgb, [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)])
    if not ok:
        raise ValueError("cv2.imencode refused the frame")
    return bytes(buf)


class FrameWriter:
    """One camera's frames → `<dir>/<seq>.jpg` + `index.json`, encoded in its own thread.

    The control loop calls `offer()` (never blocks); the thread encodes and writes at whatever pace the disk allows; `stop()` drains what is queued, writes the index, and reports honestly — written, dropped, and write errors are three different numbers on purpose, because summing them would hide which failure happened.
    """

    def __init__(self, camera_name: str, out_dir: Path,
                 encode: Callable[[Any], bytes] = encode_jpeg,
                 queue_frames: int = QUEUE_FRAMES) -> None:
        self.camera_name = camera_name
        self.out_dir = Path(out_dir)
        self.out_dir.mkdir(parents=True, exist_ok=True)
        self._encode = encode
        self._q: queue.Queue[Frame] = queue.Queue(maxsize=queue_frames)
        self._entries: list[list[int]] = []       # [seq, host_stamp_ns], in write order
        self.dropped = 0
        self.write_errors = 0
        self._running = True
        self._thread = threading.Thread(target=self._run,
                                        name=f"frames-{camera_name}", daemon=True)
        self._thread.start()

    def offer(self, frame: Frame) -> None:
        """Hand one fresh frame over. Never blocks: a full queue drops its OLDEST frame, counted."""
        try:
            self._q.put_nowait(frame)
        except queue.Full:
            try:
                self._q.get_nowait()
                self.dropped += 1
            except queue.Empty:
                pass
            try:
                self._q.put_nowait(frame)
            except queue.Full:      # another producer refilled it — count this frame as the drop
                self.dropped += 1

    def _run(self) -> None:
        # The get-with-timeout shape means the thread only exits once the queue is EMPTY and stop() has been called, so stop() is also the flush — item 48's teardown rule (indexes complete before the summary prints) follows from the loop shape rather than from a separate drain step.
        while True:
            try:
                frame = self._q.get(timeout=0.05)
            except queue.Empty:
                if not self._running:
                    return
                continue
            try:
                data = self._encode(frame.rgb)
                (self.out_dir / f"{frame.sequence:06d}.jpg").write_bytes(data)
                self._entries.append([int(frame.sequence), int(frame.host_timestamp_ns)])
            except Exception:  # noqa: BLE001 — a bad frame must cost one image, never the session
                self.write_errors += 1

    @property
    def written(self) -> int:
        return len(self._entries)

    def stop(self) -> dict[str, Any]:
        """Flush the queue, write `index.json`, and return what actually happened.

        ⚠️ The generous join timeout exists for a hung encode: if the thread does not come back, the index is still written with everything recorded so far, and the caller sees `flushed: false` instead of a file that pretends completeness.
        """
        self._running = False
        self._thread.join(timeout=10.0)
        index = {
            "camera": self.camera_name,
            "jpeg_quality": JPEG_QUALITY,
            "written": self.written,
            "dropped": self.dropped,
            "write_errors": self.write_errors,
            "flushed": not self._thread.is_alive(),
            "entries": self._entries,
        }
        (self.out_dir / "index.json").write_text(json.dumps(index))
        return index


class FrameSink:
    """Every camera's writer behind the ONE call the control loop makes per cycle.

    `offer(samples)` takes `CaptureSet.sample()`'s dict and forwards each camera's frame ONLY when its sequence advanced. `CaptureSet` repeats a slow camera's last frame on purpose (duplicates are data at the sampling layer), and storing those repeats would write every 30 fps frame three times at a 100 Hz loop — the dedupe belongs here, where the dataset is made, exactly as `yam/cameras/capture.py`'s docstring promised it would be the recorder's decision.
    """

    def __init__(self, root: Path, camera_names: list[str],
                 encode: Callable[[Any], bytes] = encode_jpeg,
                 queue_frames: int = QUEUE_FRAMES) -> None:
        self.root = Path(root)
        self._writers = {name: FrameWriter(name, self.root / camera_dir_name(name),
                                           encode, queue_frames)
                         for name in camera_names}
        self._last_seq = {name: 0 for name in camera_names}

    @property
    def names(self) -> list[str]:
        return list(self._writers)

    def offer(self, samples: dict[str, Frame | None]) -> None:
        for name, frame in samples.items():
            writer = self._writers.get(name)
            if writer is None or frame is None:
                continue
            if frame.sequence != self._last_seq[name]:
                self._last_seq[name] = frame.sequence
                writer.offer(frame)

    def stop(self) -> dict[str, dict[str, Any]]:
        """Stop every writer; the per-camera indexes, keyed by camera name."""
        return {name: w.stop() for name, w in self._writers.items()}


# ---------------------------------------------------------------- take directories ----
# A take's frames are written while the slot is still unknown (the digit comes AFTER the recording stops), so they land in a pending directory and move under their slot at save time. Every path below stays on one filesystem, so the move is a rename, not a copy.

def pending_frames_dir(recordings_dir: Path, stamp: str) -> Path:
    """Where a still-unsaved take's frames go: `recordings/frames/pending_<stamp>/`."""
    return Path(recordings_dir) / FRAMES_DIRNAME / f"pending_{stamp}"


def attach_frames_to_slot(recordings_dir: Path, pending: Path, slot: str) -> Path:
    """Move a finished take's frames under their slot, replacing any stale set.

    ⛔ The delete-first matters: slots overwrite by design, and a slot's OLD frames left beside a NEW recording would be attributed to it by anything that trusts the directory name. Stale-but-plausible is this stack's signature failure; remove, then rename.
    """
    dest = Path(recordings_dir) / FRAMES_DIRNAME / str(slot)
    if dest.exists():
        shutil.rmtree(dest)
    dest.parent.mkdir(parents=True, exist_ok=True)
    Path(pending).rename(dest)
    return dest


def discard_frames(pending: Path | None) -> None:
    """Delete a discarded or aborted take's frames. Harmless when already gone."""
    if pending is not None and Path(pending).exists():
        shutil.rmtree(pending)


def clear_slot_frames(recordings_dir: Path, slot: str) -> bool:
    """Remove a slot's frames when a FRAMELESS recording saves into it. True if any existed.

    Without this, saving a camera-less take into slot 3 would leave slot 3's old frames on disk, and `check_recordings` (or a person) would read them as belonging to the new file.
    """
    dest = Path(recordings_dir) / FRAMES_DIRNAME / str(slot)
    if dest.exists():
        shutil.rmtree(dest)
        return True
    return False
