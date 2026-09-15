"""Queue fresh camera frames without waiting for JPEG encoding or disk writes.

Stop requests are nonblocking. The writer publishes its index only after the
queue drains; callers must retain its directory until completion is confirmed.
A full queue drops the oldest frame and records that loss in the final report.
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
        self._report: dict[str, Any] | None = None
        self._failure: BaseException | None = None
        self._thread = threading.Thread(target=self._run,
                                        name=f"frames-{camera_name}", daemon=True)
        self._thread.start()

    def offer(self, frame: Frame) -> None:
        """Hand one fresh frame over. Never blocks: a full queue drops its OLDEST frame, counted."""
        if not self._running or self.finished:
            raise RuntimeError("Cannot offer frames to a stopped writer")
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
        try:
            self._drain()
            report = self._snapshot(flushed=True)
            (self.out_dir / "index.json").write_text(json.dumps(report))
            self._report = report
        except BaseException as exc:
            # Report through the owner, never the global robot-thread fault hook.
            self._failure = exc

    def _drain(self) -> None:
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
            except Exception:  # A bad frame is counted; the remaining queue still drains.
                self.write_errors += 1

    @property
    def written(self) -> int:
        return len(self._entries)

    @property
    def finished(self) -> bool:
        """No worker can mutate the directory, including after an index failure."""
        return not self._thread.is_alive()

    def request_stop(self) -> None:
        """Close input and drain queued frames in the worker. Does not wait."""
        self._running = False

    def _snapshot(self, *, flushed: bool) -> dict[str, Any]:
        return {
            "camera": self.camera_name,
            "jpeg_quality": JPEG_QUALITY,
            "written": self.written,
            "dropped": self.dropped,
            "write_errors": self.write_errors,
            "flushed": flushed,
            "entries": list(self._entries),
        }

    def poll_stop(self) -> dict[str, Any] | None:
        """Return the final index, None while busy, or raise the worker failure."""
        if not self.finished:
            return None
        if self._failure is not None:
            raise RuntimeError(f"Frame index failed for {self.camera_name}: {self._failure}") from self._failure
        return self._report

    def stop(self, timeout: float = 10.0) -> dict[str, Any]:
        """Bounded blocking convenience for teardown and standalone tools.

        An unfinished report is diagnostic only. No index is published until the
        worker completes. Use request_stop/poll_stop in the operator loop.
        """
        self.request_stop()
        self._thread.join(timeout=timeout)
        report = self.poll_stop()
        return report if report is not None else self._snapshot(flushed=False)


class FrameSink:
    """Every camera's writer behind the ONE call the control loop makes per cycle.

    `offer(samples)` takes `CaptureSet.sample()`'s dict and forwards each camera's frame ONLY when its sequence advanced. `CaptureSet` repeats a slow camera's last frame on purpose (duplicates are data at the sampling layer), and storing those repeats would write every 30 fps frame three times at a 100 Hz loop — the dedupe belongs here, where the dataset is made, exactly as `yam/cameras/capture.py`'s docstring promised it would be the recorder's decision.
    """

    def __init__(self, root: Path, camera_names: list[str],
                 encode: Callable[[Any], bytes] = encode_jpeg,
                 queue_frames: int = QUEUE_FRAMES) -> None:
        self.root = Path(root)
        self._writers = {}
        self._last_seq = {}
        self._encode = encode
        self._queue_frames = queue_frames
        try:
            self.start(camera_names)
        except BaseException as failure:
            try:
                self.stop()
            except Exception as cleanup:
                failure.add_note(f"Frame writer startup cleanup failed: {cleanup!r}")
            raise

    def start(self, camera_names: list[str]) -> None:
        """Acquire writers into this already-owned sink.

        On failure, the caller still owns every writer acquired so far and must
        request stop. The constructor provides cleanup for standalone callers.
        """
        for name in camera_names:
            if name in self._writers:
                raise ValueError(f"Camera writer already exists: {name}")
            self._writers[name] = FrameWriter(
                name, self.root / camera_dir_name(name), self._encode, self._queue_frames)
            self._last_seq[name] = 0

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

    @property
    def finished(self) -> bool:
        return all(writer.finished for writer in self._writers.values())

    def request_stop(self) -> None:
        """Close every input before waiting for any camera."""
        for writer in self._writers.values():
            writer.request_stop()

    def poll_stop(self) -> dict[str, dict[str, Any]] | None:
        """Return all final indexes only when every writer has terminated."""
        if not self.finished:
            return None
        reports, failures = {}, []
        for name, writer in self._writers.items():
            try:
                reports[name] = writer.poll_stop()
            except Exception as exc:
                failures.append(exc)
        if failures:
            raise ExceptionGroup("Frame writers failed to finish", failures)
        return reports

    def stop(self) -> dict[str, dict[str, Any]]:
        """Stop every writer; the per-camera indexes, keyed by camera name."""
        reports = {}
        failures = []
        for name, writer in self._writers.items():
            try:
                reports[name] = writer.stop()
            except Exception as exc:
                exc.add_note(f"Frame writer: {name}")
                failures.append(exc)
        if failures:
            raise ExceptionGroup("Frame writers failed to stop", failures)
        return reports


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
