#!/usr/bin/env python3
"""Prove `yam/timing.py` reports the worst pass and never quietly flatters the loop.

    uv run tests/test_timing.py

⛔ WHY EACH CHECK IS HERE. This class exists to make one stall visible, so the failure mode that matters is a stall going missing: a clamp, an average, a discarded sample or a bucket that counts the wrong side of its threshold. Every check below is one of those.

⚠️ WHAT THESE CHECKS CANNOT DO, said here because [FINDINGS §76.12](../docs/FINDINGS.md) is exactly this mistake: they feed the class numbers, so they prove the arithmetic and nothing about the loop. A pass interval measured against a simulated arm is Python-side jitter only. Only the real station can say what a real pass costs.
"""

from __future__ import annotations

import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent

from yam.timing import DEFAULT_OVER_MS, KEEP_SLOWEST, LoopTimer  # noqa: E402


def test_the_first_sample_is_discarded() -> None:
    """⭐ The loop's first interval runs from its own zero point, so it is a fraction of a
    millisecond and is not a pass. Counting it would drag the mean upwards for ever."""
    t = LoopTimer(100.0)
    t.record(0.0002, at_s=0.0)
    assert t.count == 0, "the first sample must not be counted"
    assert t.worst_s == 0.0
    t.record(0.010, at_s=0.01)
    assert t.count == 1, "the second sample is the first real pass"


def test_the_worst_pass_survives_a_flood_of_healthy_ones() -> None:
    """⛔ THE WHOLE POINT. One 47 ms stall among 5000 good passes must still be reported,
    which is what an exponential average of the rate cannot do."""
    t = LoopTimer(100.0)
    t.record(0.0)  # discarded
    for i in range(5000):
        t.record(0.0100, at_s=i * 0.01)
    t.record(0.047, at_s=50.0)
    for i in range(5000):
        t.record(0.0100, at_s=50.0 + i * 0.01)
    assert round(t.worst_s, 6) == 0.047, f"the stall was lost: worst is {t.worst_s}"
    assert t.worst_at_s == 50.0, "the moment of the stall must be kept, to match the log"
    assert 99.0 < t.mean_hz < 100.0, f"10 000 good passes should read ~100 Hz, got {t.mean_hz}"


def test_nothing_is_clamped() -> None:
    """⛔ The control loop clamps its own `real_dt` to 100 ms so a stall cannot make a
    playback cursor jump. That clamp is right, and this class must not inherit it: a 400 ms
    stall reported as 100 ms is the defect."""
    t = LoopTimer(100.0)
    t.record(0.0)
    t.record(0.400, at_s=3.0)
    assert round(t.worst_s, 6) == 0.400, "a stall longer than the loop's clamp was clamped"
    assert t.to_dict()["worst_ms"] == 400.0


def test_the_buckets_count_strictly_over_their_threshold() -> None:
    """⚠️ A pass of exactly 33 ms is not over 33 ms. Off-by-one here would turn a healthy
    loop into a reported fault, which is the `check_style` false-alarm pattern
    ([FINDINGS §77.5](../docs/FINDINGS.md)) in a measurement."""
    t = LoopTimer(100.0, over_ms=(33.0,))
    t.record(0.0)
    t.record(0.033)
    assert t.over[33.0] == 0, "exactly at the threshold is not over it"
    t.record(0.0331)
    assert t.over[33.0] == 1


def test_over_target_counts_passes_with_no_headroom() -> None:
    """⭐ At a 100 Hz target a pass longer than 10 ms means the loop had nothing left. The
    target does not have to be one of the buckets, so this is counted separately."""
    t = LoopTimer(100.0)
    t.record(0.0)
    for dt in (0.009, 0.010, 0.011, 0.030):
        t.record(dt)
    assert t.over_target == 2, f"expected the 11 ms and 30 ms passes, got {t.over_target}"
    assert t.over[15.0] == 1, "only the 30 ms pass is over 15 ms"


def test_an_empty_timer_reports_nothing_rather_than_dividing_by_zero() -> None:
    """⛔ It is read on the shutdown path, including after a session that never ran a pass."""
    t = LoopTimer(100.0)
    assert t.count == 0
    assert t.mean_s == 0.0 and t.mean_hz == 0.0
    assert "no passes" in t.line()
    assert t.to_dict()["passes"] == 0


def test_the_report_is_measurements_with_no_verdict() -> None:
    """⛔ FINDINGS §76 is a day spent on lines that reported a claim as a measurement. This
    report may not contain a judgement like "healthy", because whether a number is good
    depends on the machine it came from, and the dictionary does not know that."""
    t = LoopTimer(100.0)
    t.record(0.0)
    t.record(0.010)
    got = t.to_dict()
    for key in ("target_hz", "passes", "mean_ms", "mean_hz", "worst_ms",
                "worst_at_s", "over_target", "over_ms"):
        assert key in got, f"{key} missing from the report"
    flat = repr(got).lower()
    for word in ("ok", "healthy", "fine", "good", "bad", "fault", "warn"):
        assert word not in flat, f"the report contains a verdict: {word!r}"


def test_the_thresholds_include_the_arms_own_response_time() -> None:
    """⭐ 33 ms is the arm's following delay ([FINDINGS §66.1](../docs/FINDINGS.md)), so it is
    the threshold that separates jitter the hardware can notice from jitter it cannot."""
    assert 33.0 in DEFAULT_OVER_MS, "the 33 ms bucket is the one with a physical meaning"
    assert list(DEFAULT_OVER_MS) == sorted(DEFAULT_OVER_MS), "kept in order for the report"


def test_a_negative_interval_is_refused_rather_than_recorded() -> None:
    """⚠️ Time going backwards means the clock, not the loop. Recording it would put a
    negative pass into a report that is read as a measurement."""
    t = LoopTimer(100.0)
    t.record(0.0)
    t.record(0.010)
    t.record(-0.5)
    assert t.count == 1, "a negative interval must not be counted"


def test_the_target_must_be_positive() -> None:
    """⛔ A zero or negative target would divide by zero inside `target_s`, on the loop's
    own construction path."""
    for bad in (0.0, -100.0):
        try:
            LoopTimer(bad)
        except ValueError:
            continue
        raise AssertionError(f"LoopTimer({bad}) should have refused")


def test_the_five_slowest_passes_are_kept_with_their_moments() -> None:
    """⭐⭐ WHY FIVE AND NOT ONE. Finding the cause of the first worst pass ever measured
    needed a temporary `print` in the loop and a log read, because one time is not a pattern.
    The camera-free simulated session had four slow passes with three different causes: the
    mode change into TELEOP, and two during replay. This is what makes that a report read
    rather than a probe written."""
    t = LoopTimer(100.0)
    t.record(0.0)
    for ms, at in ((12, 1.0), (73, 19.3), (40, 24.7), (39, 35.0), (36, 67.5), (11, 70.0),
                   (50, 80.0)):
        t.record(ms / 1000.0, at_s=at)
    got = [(round(s * 1000.0), at) for s, at in t.slowest]
    assert got == [(73, 19.3), (50, 80.0), (40, 24.7), (39, 35.0), (36, 67.5)], got
    assert len(t.slowest) == KEEP_SLOWEST, "the list must not grow without bound"
    assert t.to_dict()["slowest"][0] == {"ms": 73.0, "at_s": 19.3}


def test_a_slow_pass_that_arrives_last_still_displaces_a_faster_one() -> None:
    """⚠️ The obvious bug in a keep-the-worst-N list: it fills up early and then ignores
    everything, so a stall late in a long session is lost. That is the same shape as
    reporting only the average."""
    t = LoopTimer(100.0)
    t.record(0.0)
    for i in range(50):
        t.record(0.011, at_s=float(i))
    t.record(0.200, at_s=99.0)
    assert round(t.slowest[0][0], 6) == 0.200 and t.slowest[0][1] == 99.0


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
