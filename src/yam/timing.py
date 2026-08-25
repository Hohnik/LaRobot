"""How long each pass of the control loop actually took, and above all the worst one.

⭐⭐ WHY THIS EXISTS. [PERFORMANCE.md](../../docs/PERFORMANCE.md) section 2 and [FINDINGS §77.4](../../docs/FINDINGS.md) end at the same missing number. The loop asks for 100 passes a second and has never achieved it: about 87 on the Mac before any camera code existed, 83 to 84 with cameras. Those are averages, and an average is not what a jerky arm feels. **Nothing in this repo has ever recorded the worst single pass.** Julien's own question was whether the camera work should be moved off the loop, and jitter is the thing that separating processes would fix. Without this number there is nothing to decide with.

⛔ WHAT IT MEASURES, exactly. The interval from the start of one pass to the start of the next, **including the sleep at the bottom of the loop**. So a healthy pass at 100 Hz reads about 10 ms, and a pass longer than 10 ms is a pass that had no headroom left. That interval is also the gap between two commands reaching the arm, which is the number the hardware experiences.

⛔⭐ IT TAKES THE RAW INTERVAL, BEFORE THE LOOP'S OWN CLAMP. `apps/teleop_session.py` clamps its `real_dt` to 100 ms so that one long stall cannot make a playback cursor jump forward and command a step no hand ever taught. That clamp is right and it would hide exactly the stall this class exists to find, so the raw value is recorded here and the clamped one is used for control.

⚠️ WHAT IT CANNOT TELL YOU. Where the time went inside a pass. A worst pass of 60 ms says the loop stalled; it does not say whether it was the CAN round trip, a file write or the status line. `worst_at_s` is kept so the moment can be matched against what the session printed, which is the cheapest next step when a number looks wrong.

⛔⭐⭐ AND THE TRAP THAT MATTERS MOST, because it has already cost this project a day. A **simulated** arm answers in microseconds. A real one is 14 motors over two USB adapters and is the largest single item in a pass. So numbers from `--sim` are the Python-side jitter and nothing else, and quoting them as "the loop's jitter" would be [FINDINGS §76.12](../../docs/FINDINGS.md) again: the camera fix's own rate measurement passed 12 unit tests against a fake handle that answered instantly, and reported 8.6 fps for cameras running at 30. `to_dict()` therefore carries no verdict, only measurements, and whoever reads one has to know which machine it came from.
"""

from __future__ import annotations

from collections.abc import Sequence

__all__ = ["LoopTimer", "DEFAULT_OVER_MS", "KEEP_SLOWEST"]

#: The thresholds worth counting, in milliseconds, and each one is a real question.
#:
#: 15 and 20 are "the loop lost its headroom" at a 10 ms target. **33 is the important one**: the arm follows a moving target with a delay of about 0.033 s ([FINDINGS §66.1](../../docs/FINDINGS.md)), so a pass longer than that is a pass the hardware can notice. 50 and 100 are stalls, and 100 is where the loop's own clamp used to hide them.
DEFAULT_OVER_MS: tuple[float, ...] = (15.0, 20.0, 33.0, 50.0, 100.0)

#: ⭐⭐ HOW MANY SLOW PASSES TO KEEP, and this number was chosen from doing the work by hand. Finding the cause of the first worst pass ever measured took a temporary `print` in the loop, one run, and reading the log to see what had been printed just before it. **One worst pass is not enough to do that**: the session's four slow passes had three different causes. Five costs nothing and turns the same investigation into reading the report.
KEEP_SLOWEST = 5


class LoopTimer:
    """Counts passes of a loop and remembers the worst interval between two of them.

    Cost per pass is one subtraction and a few comparisons, so it can sit in the control loop without being the thing it measures.
    """

    def __init__(self, target_hz: float,
                 over_ms: Sequence[float] = DEFAULT_OVER_MS) -> None:
        if target_hz <= 0:
            raise ValueError(f"target_hz must be positive, got {target_hz!r}")
        self.target_hz = float(target_hz)
        self.target_s = 1.0 / float(target_hz)
        self.over_ms: tuple[float, ...] = tuple(sorted(float(m) for m in over_ms))
        self.count = 0
        self.total_s = 0.0
        self.worst_s = 0.0
        self.worst_at_s = 0.0
        self.over: dict[float, int] = {ms: 0 for ms in self.over_ms}
        #: Passes longer than the target interval, so passes where the loop had no headroom left. Counted directly rather than read off the buckets, because the target does not have to be one of them.
        self.over_target = 0
        #: The slowest passes so far, as `(seconds, loop time)`, longest first. Each entry is a moment to look up in what the session printed.
        self.slowest: list[tuple[float, float]] = []
        #: ⚠️ The first interval is not a pass. It is measured from the loop's zero point, so it reads as a fraction of a millisecond and would drag the mean. It is ignored, which is why `count` is one less than the number of passes the loop ran.
        self.ignored_first = False

    def record(self, interval_s: float, at_s: float = 0.0) -> None:
        """Take one raw pass interval, in seconds, and the loop time it ended at."""
        if not self.ignored_first:
            self.ignored_first = True
            return
        if interval_s < 0:
            return
        self.count += 1
        self.total_s += interval_s
        if interval_s > self.worst_s:
            self.worst_s = interval_s
            self.worst_at_s = at_s
        if interval_s > self.target_s:
            self.over_target += 1
        if len(self.slowest) < KEEP_SLOWEST or interval_s > self.slowest[-1][0]:
            self.slowest.append((interval_s, at_s))
            self.slowest.sort(key=lambda pair: -pair[0])
            del self.slowest[KEEP_SLOWEST:]
        ms = interval_s * 1000.0
        for threshold in self.over_ms:
            if ms > threshold:
                self.over[threshold] += 1

    @property
    def mean_s(self) -> float:
        """Mean pass interval, or 0.0 before any pass has been recorded."""
        return self.total_s / self.count if self.count else 0.0

    @property
    def mean_hz(self) -> float:
        """Passes a second, from the mean interval. 0.0 before any pass."""
        mean = self.mean_s
        return 1.0 / mean if mean > 0 else 0.0


    def to_dict(self) -> dict[str, object]:
        """Measurements only, ready for JSON. ⛔ No verdict: see the module docstring."""
        return {
            "target_hz": round(self.target_hz, 3),
            "passes": self.count,
            "mean_ms": round(self.mean_s * 1000.0, 3),
            "mean_hz": round(self.mean_hz, 2),
            "worst_ms": round(self.worst_s * 1000.0, 3),
            "worst_at_s": round(self.worst_at_s, 3),
            "over_target": self.over_target,
            "over_ms": {f"{ms:g}": self.over[ms] for ms in self.over_ms},
            "slowest": [{"ms": round(s * 1000.0, 3), "at_s": round(at, 3)}
                        for s, at in self.slowest],
        }

    def line(self) -> str:
        """One line for a terminal, worst pass first, because that is the new information."""
        if not self.count:
            return "loop timing: no passes recorded"
        parts = [f"loop: worst pass {self.worst_s * 1000.0:.1f} ms "
                 f"at t={self.worst_at_s:.1f}s",
                 f"mean {self.mean_s * 1000.0:.1f} ms ({self.mean_hz:.0f} Hz "
                 f"against {self.target_hz:.0f})",
                 f"{self.count} passes"]
        loud = [f"{self.over[ms]} over {ms:g} ms" for ms in self.over_ms
                if self.over[ms]]
        if loud:
            parts.append(", ".join(loud))
        return " · ".join(parts)
