"""One take's shared joint timeline, camera ownership and pending disk operation.

Only the operator thread changes lifecycle state. Writers and a single disk job
perform I/O; poll() observes completion without waiting. Neither commands robots.
"""
from __future__ import annotations

from pathlib import Path
import threading
import time
from typing import Any, Callable, Sequence

from yam.cameras.writer import FrameSink, discard_frames
from yam.recording_store import SavedTake, save_take

from yam.recording import Trajectory


class RecordingSession:
    def __init__(self) -> None:
        self.active: Trajectory | None = None
        self.pending: Trajectory | None = None
        self.started_at = 0.0
        self.label = "good"
        self._modes: list[str] = []
        self.frames: Path | None = None
        self.sink: FrameSink | None = None
        self.frame_report: dict[str, Any] | None = None
        self.frame_error: str | None = None
        self._mono0 = 0
        self._discard_requested = False
        self._job: _DiskJob | None = None
        self.saved: SavedTake | None = None
        self.save_error: BaseException | None = None

    def start(self, t: float, *, meta: dict[str, Any], modes: Sequence[str]) -> None:
        """Start only when the previous take has been saved or discarded."""
        if self.busy or self.active is not None or self.pending is not None or self.frames is not None:
            raise RuntimeError("Save or discard the previous recording before starting another")
        self.saved = None
        self.save_error = None
        self.frame_error = None
        self.started_at = t
        self.label = "good"
        self._modes = list(dict.fromkeys(modes))
        self.active = Trajectory(meta=dict(meta))
        self._stamp_method(self.active)

    def elapsed(self, t: float) -> float:
        return t - self.started_at

    def sample(self, t: float, joints: Sequence[float], modes: Sequence[str]) -> None:
        """Append one measured cycle. A frozen take never grows at the save prompt."""
        if self.active is None:
            return
        self.active.append(self.elapsed(t), joints)
        for mode in modes:
            if mode not in self._modes:
                self._modes.append(mode)

    def toggle_label(self, t: float) -> str:
        if self.active is None:
            raise RuntimeError("Labels require an active recording")
        label = "bad" if self.label == "good" else "good"
        self.active.mark(self.elapsed(t), label)
        self.label = label
        return label

    def _stamp_method(self, trajectory: Trajectory) -> None:
        prefix = "sim:" if trajectory.meta.get("simulated", False) else "live:"
        trajectory.meta["method"] = prefix + "+".join(self._modes)

    def freeze(self) -> Trajectory:
        """Stop sampling and finalize provenance for either manual or limit stop."""
        if self.active is None:
            raise RuntimeError("No active recording to stop")
        self.pending, self.active = self.active, None
        if self.sink is not None:
            self.sink.request_stop()
        self.pending.meta["modes"] = list(self._modes)
        self._stamp_method(self.pending)
        return self.pending

    @property
    def finishing(self) -> bool:
        return self.active is None and self.sink is not None and not self.sink.finished

    @property
    def busy(self) -> bool:
        return self.finishing or self._job is not None or self._discard_requested

    @property
    def ready(self) -> bool:
        return (self.pending is not None and not self.busy and not self.frame_error
                and (self.frames is None or self.frame_report is not None))

    def start_frames(self, path: Path, names: list[str], mono0: int,
                     factory: Callable[..., FrameSink] = FrameSink) -> None:
        """Register the directory before acquisition; retain it after failure.

        Acquire an empty sink first, then start its writers under this owner.
        Partial startup therefore retains every acquired writer. Camera readers
        remain owned by the caller.
        """
        if self.active is None or self.frames is not None:
            raise RuntimeError("Camera startup requires a new active take")
        self.frames = Path(path)
        self._mono0 = mono0
        try:
            self.sink = factory(self.frames, [])
            self.sink.start(names)
        except Exception as exc:
            self.frame_error = f"Camera writer startup failed: {exc}"
            raise

    def offer_frames(self, samples: dict[str, Any]) -> None:
        if self.active is not None and self.sink is not None:
            self.sink.offer(samples)

    def fail(self, exc: Exception) -> None:
        """Stop this take after a sampling failure, retaining evidence for discard."""
        self.frame_error = f"Recording failed: {type(exc).__name__}: {exc}"
        if self.active is not None:
            self.freeze()

    def poll(self) -> None:
        """Observe completion; never join a thread or perform filesystem work."""
        if self.active is None and self.sink is not None and self.sink.finished:
            try:
                reports = self.sink.poll_stop()
                if reports is None or any(r is None or r.get("flushed") is not True
                                          for r in reports.values()):
                    raise RuntimeError("Camera writer returned an incomplete report")
                self.frame_report = {"mono0_ns": self._mono0, "per_camera": reports}
            except Exception as exc:
                self.frame_error = f"Camera completion failed: {exc!r}"
            # finished means no writer can touch the directory, even after failure.
            self.sink = None
        if self._job is not None and self._job.done.is_set():
            job, self._job = self._job, None
            if job.failure is not None:
                self.save_error = job.failure
                self._discard_requested = False
            else:
                self.saved = job.result if job.kind == "save" else None
                self._release()
        if self._discard_requested and self.sink is None and self._job is None:
            try:
                self._job = _DiskJob("discard", lambda: discard_frames(self.frames))
            except Exception as exc:
                self.save_error = exc
                self._discard_requested = False

    def request_save(self, root: Path, slot: str,
                     saver: Callable[..., SavedTake] = save_take) -> None:
        """Start publication only when the whole take is ready. Failures keep it."""
        if not self.ready:
            raise RuntimeError(self.frame_error or "Recording files are still finishing")
        self.save_error = None
        self.saved = None
        self._job = _DiskJob("save", lambda: saver(
            self.pending, root, slot, pending_frames=self.frames,
            frame_report=self.frame_report))

    def request_discard(self) -> None:
        """Stop input now; remove files only after all writers have terminated."""
        if self._job is not None:
            raise RuntimeError("A recording disk operation is already in progress")
        if self.active is not None:
            self.freeze()
        self._discard_requested = True
        self.save_error = None
        self.poll()

    def _release(self) -> None:
        self.active = self.pending = None
        self.frames = None
        self.frame_report = None
        self.frame_error = None
        self._discard_requested = False

    def discard(self) -> None:
        """Release a frameless take synchronously. Use request_discard for files."""
        if self.busy or self.frames is not None:
            raise RuntimeError("Recording files require asynchronous discard")
        self._release()

    def shutdown(self, timeout: float = 1.0) -> None:
        """After motor shutdown, allow bounded completion; retain unresolved files.

        Unsaved camera files are preserved on exit for inspection. Joint samples
        still held only in memory are not an automatic recovery recording.
        An already requested save/discard may finish. Never delete behind a writer.
        """
        if self.active is not None:
            self.freeze()
        deadline = time.monotonic() + timeout
        while self.busy and time.monotonic() < deadline:
            self.poll()
            time.sleep(.01)
        self.poll()
        if self.busy or self.frames is not None or self.save_error is not None:
            raise RuntimeError(f"Recording cleanup incomplete; files retained at {self.frames}. "
                               f"{self.frame_error or self.save_error or 'Pending take was not saved.'}")


class _DiskJob:
    """One retained, daemon I/O job; failures return to the recording owner."""
    def __init__(self, kind: str, work: Callable[[], Any]) -> None:
        self.kind = kind
        self.done = threading.Event()
        self.result: Any = None
        self.failure: BaseException | None = None

        def run() -> None:
            try:
                self.result = work()
            except BaseException as exc:
                self.failure = exc
            finally:
                self.done.set()

        self.thread = threading.Thread(target=run, name=f"take-{kind}", daemon=True)
        self.thread.start()
