"""One replay clock, layout, arrival gate and tracking history for all arms.

The operator owns arm commands and confirms the measured start pose before start.
This object computes one target per cycle and observes tracking after commands.
"""
from __future__ import annotations

from typing import Any, Sequence

from yam.recording import Layout, ReplayStep, TrackingLog, Trajectory, replay_step, scrub_step, safe_time_scale


class PlaybackSession:
    def __init__(self) -> None:
        self.active: Trajectory | None = None
        self.pending: Trajectory | None = None
        self.layout: Layout | None = None
        self.arms: list[Any] = []
        self._ready: set[str] = set()
        self.slot = "?"
        self.speed = 1.0
        self.scrub = False
        self.started_at = self.cursor = self.progress_at = 0.0
        self.held_seconds = self.worst_lag = 0.0
        self.previous_target: list[float] | None = None
        self.tracking: TrackingLog | None = None

    def prepare(self, take: Trajectory, layout: Layout, arms: Sequence[Any],
                slot: str, max_joint_speed: float) -> None:
        """Arm a validated take with its own arm order and default time scale."""
        if layout.n_joints != take.n_joints or [a.name for a in arms] != list(layout.arms):
            raise ValueError("Replay arms and samples must match the recording layout")
        self.pending, self.layout, self.arms = take, layout, list(arms)
        self._ready, self.slot, self.scrub = set(), slot, False
        self.speed = min(1.0, safe_time_scale(take.joint_speed(99), max_joint_speed))

    def cancel_pending(self) -> None:
        self.pending = None
        self._ready.clear()

    def credit_arrival(self, name: str, purpose: str) -> list[str] | None:
        """Return waiting arms, or None for an arrival unrelated to this replay."""
        names = [arm.name for arm in self.arms]
        if self.pending is None or name not in names or purpose != "replay":
            return None
        self._ready.add(name)
        return [arm for arm in names if arm not in self._ready]

    def start(self, t: float) -> None:
        """Start after all arrivals and the caller's fresh measured-pose check."""
        if self.pending is None or not self.arms or any(a.name not in self._ready for a in self.arms):
            raise RuntimeError("Every replay arm must arrive at its start before playback")
        self.active, self.pending = self.pending, None
        self.started_at = self.progress_at = t
        self.cursor = self.held_seconds = self.worst_lag = 0.0
        self.previous_target = list(self.active.start_pose() or ())
        self.tracking = TrackingLog(self.active.n_joints)

    def advance(self, measured: Sequence[float], dt: float, *, n_arm: int,
                max_lag: float, deflection: float = 0.0, scrub_max: float = 1.0) -> ReplayStep:
        """Compute one shared target; omit grippers from the lag gate by layout."""
        if self.active is None or self.layout is None:
            raise RuntimeError("No active playback")
        compare = self.layout.tracked_indices(n_arm)
        if self.scrub:
            step = scrub_step(self.active, self.cursor, measured, dt, deflection,
                              max_lag=max_lag, compare=compare, max_rate=scrub_max)
        else:
            step = replay_step(self.active, self.cursor, measured, dt,
                               speed=self.speed, max_lag=max_lag, compare=compare)
        self.cursor = step.cursor
        return step

    def observe(self, step: ReplayStep, measured: Sequence[float], t: float, dt: float) -> None:
        """Update tracking after the operator has issued every arm's command."""
        self.worst_lag = max(self.worst_lag, step.lag)
        if self.tracking is not None and self.previous_target is not None:
            self.tracking.observe(step.target, self.previous_target, measured, dt)
        self.previous_target = list(step.target)
        if step.held:
            self.held_seconds += dt
        else:
            self.progress_at = t

    def finish(self) -> None:
        """Release the active trajectory after completion or mode cancellation."""
        self.active = None
