"""Sample N cameras into named, timestamped frames at the control loop's own moments.

⭐ THE SHAPE, and why it is this way round: the readers (one thread per camera, `yam.cameras.grabber`) run at each camera's own rate, and `CaptureSet.sample()` never blocks — it takes whatever each camera currently has. The control loop stays at its 90 Hz whatever the cameras do, a slow camera simply repeats its last frame, and the per-frame `sequence` makes every repeat visible to the consumer instead of silently padding a dataset with duplicates.

⛔ Duplicates are DATA, not an error. A 30 fps camera sampled at 90 Hz repeats each frame about twice — the recorder decides whether to store repeats or dedupe on `sequence`; this layer's job is to make the repetition impossible to miss (the report counts it per camera).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from yam.cameras.frame import Frame


@dataclass(frozen=True)
class CameraReport:
    """What one camera actually delivered while it was being sampled.

    ⭐ These are the numbers that answer ROADMAP §8.2 item 6's measurement questions: `capture_fps` says what the camera managed (USB bandwidth exhaustion shows here as a LOW number, never as an error — FINDINGS §34.5's warning), `worst_gap_ms` says the longest blind stretch a dataset built on this camera would carry, and `fresh`/`duplicates` say how the camera's rate relates to the sampling rate.
    """

    name: str
    samples: int          # times sample() asked this camera
    fresh: int            # samples that saw a NEW frame
    duplicates: int       # samples that repeated the previous frame
    empty: int            # samples before the camera's first frame arrived
    capture_fps: float    # the reader thread's own rate, over its whole life
    mean_gap_ms: float    # mean time between distinct frames, as observed
    worst_gap_ms: float   # the longest blind stretch observed


def gap_stats_ms(stamps_ns: list[int]) -> tuple[float, float]:
    """(mean, worst) gap in milliseconds between consecutive distinct frame stamps.

    Pure, so the report math is testable without a camera. Fewer than two stamps means no gap was ever observable; both numbers are 0.0 then, and the caller's `fresh` count says why.
    """
    if len(stamps_ns) < 2:
        return 0.0, 0.0
    gaps = [(b - a) / 1e6 for a, b in zip(stamps_ns, stamps_ns[1:])]
    return sum(gaps) / len(gaps), max(gaps)


class CaptureSet:
    """Named readers, sampled together, reported honestly.

    `grabbers` maps a camera NAME (the name that ends up in every `Frame` and in the dataset) to a reader with the `FrameGrabber` API (`newest_stamped()`, `capture_fps()`, `stop()`). The mapping name→device is the operator's (established with `apps/camera_view.py --list`, the measured identification) — this layer records the claim, it cannot verify it (an agent can never open a camera, and two D405s cannot be told apart by any picture they take — FINDINGS §67.12).

    ⚠️ `Frame.rgb` shares the reader's array, no copy — at 90 Hz × 3 cameras, copying every sample would be most of the loop's budget. A consumer that KEEPS a frame copies it; a consumer that only looks does not have to.
    """

    def __init__(self, grabbers: dict[str, Any]) -> None:
        self._grabbers = dict(grabbers)
        self._last_seq: dict[str, int] = {name: 0 for name in self._grabbers}
        self._stamps: dict[str, list[int]] = {name: [] for name in self._grabbers}
        self._samples: dict[str, int] = {name: 0 for name in self._grabbers}
        self._fresh: dict[str, int] = {name: 0 for name in self._grabbers}
        self._empty: dict[str, int] = {name: 0 for name in self._grabbers}

    @property
    def names(self) -> list[str]:
        return list(self._grabbers)

    def sample(self) -> dict[str, Frame | None]:
        """Every camera's newest frame, right now. Never blocks.

        `None` for a camera whose first frame has not arrived yet. `Frame.sequence` is the READER's frame counter, so two samples of the same frame carry the same sequence — that is how a consumer tells a repeat from a new frame.
        """
        out: dict[str, Frame | None] = {}
        for name, grab in self._grabbers.items():
            frame, seq, stamp_ns = grab.newest_stamped()
            self._samples[name] += 1
            if frame is None:
                self._empty[name] += 1
                out[name] = None
                continue
            if seq != self._last_seq[name]:
                self._fresh[name] += 1
                self._last_seq[name] = seq
                self._stamps[name].append(stamp_ns)
            out[name] = Frame(camera_name=name, sequence=seq,
                              camera_timestamp_ns=None,       # this stack cannot know it
                              host_timestamp_ns=stamp_ns, rgb=frame, depth=None)
        return out

    def report(self) -> list[CameraReport]:
        """One `CameraReport` per camera, in the order they were configured."""
        out = []
        for name, grab in self._grabbers.items():
            mean_ms, worst_ms = gap_stats_ms(self._stamps[name])
            fresh = self._fresh[name]
            out.append(CameraReport(
                name=name, samples=self._samples[name], fresh=fresh,
                duplicates=self._samples[name] - fresh - self._empty[name],
                empty=self._empty[name], capture_fps=grab.capture_fps(),
                mean_gap_ms=mean_ms, worst_gap_ms=worst_ms))
        return out

    def stop(self) -> None:
        """Stop every reader. ⚠️ Always called, even after an error — a daemon thread holding a camera keeps the device busy for the next process."""
        for grab in self._grabbers.values():
            grab.stop()
