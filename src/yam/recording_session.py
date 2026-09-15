"""One recording lifecycle and timeline shared by all arms.

The operator loop reads measured joints in layout order and handles keys, slots,
and camera I/O. This object owns whether sampling is active or frozen, elapsed
sample time, labels, and the modes actually observed. It never commands a robot.
"""
from __future__ import annotations

from typing import Any, Sequence

from yam.recording import Trajectory


class RecordingSession:
    def __init__(self) -> None:
        self.active: Trajectory | None = None
        self.pending: Trajectory | None = None
        self.started_at = 0.0
        self.label = "good"
        self._modes: list[str] = []

    def start(self, t: float, *, meta: dict[str, Any], modes: Sequence[str]) -> None:
        """Start only when the previous take has been saved or discarded."""
        if self.active is not None or self.pending is not None:
            raise RuntimeError("Save or discard the previous recording before starting another")
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
        self.pending.meta["modes"] = list(self._modes)
        self._stamp_method(self.pending)
        return self.pending

    def discard(self) -> None:
        """Release both active and frozen trajectories; frame ownership is separate."""
        self.active = self.pending = None
