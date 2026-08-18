#!/usr/bin/env python3
"""Tests for the puck scrub (docs/ROADMAP.md §7.6, item 13). No hardware.

    uv run scripts/test_scrub.py

⭐ WHAT THE FEATURE IS. At the play prompt, `j` starts the recording under the PUCK's
control instead of a fixed speed: push forward to play, pull back to rewind, let go to
freeze. A SpaceMouse is spring-centred, so the neutral state is STOPPED — the dial is a
deadman by construction, which is the whole safety argument on a rig with no e-stop.

⛔ THE PROPERTIES THAT MUST HOLD:
1. Zero (and small) deflection means a FROZEN cursor — release-to-stop is the deadman.
2. The cursor clamps at both ends and `finished` is never True: the operator ends a
   scrub by leaving the mode, never the scrub by itself.
3. The lag hold works in both directions: an arm that has fallen `max_lag` behind
   freezes the cursor exactly as in a normal playback.
4. Full deflection scrubs at SCRUB_MAX_RATE — the same 1.5× ceiling a plain playback
   can reach, so scrubbing can never ask for speeds `l` could not.
"""

from __future__ import annotations

import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent

from yam.recording import (  # noqa: E402
    SCRUB_DEADBAND,
    SCRUB_MAX_RATE,
    Trajectory,
    scrub_rate,
    scrub_step,
)
from yam.settings import LADDERS, LIVE_BOUNDS, TUNABLE, adjust  # noqa: E402


def _traj(duration: float = 2.0, n: int = 7) -> Trajectory:
    """A straight-line recording: every joint runs 0 → 1 over `duration` seconds."""
    traj = Trajectory(meta={"arm": "B", "method": "guide"})
    steps = 21
    for i in range(steps):
        t = duration * i / (steps - 1)
        traj.append(t, (t / duration,) * n)
    return traj


def test_releasing_the_puck_freezes_the_cursor() -> None:
    """⛔ Property 1, the deadman. Zero and sub-deadband deflections move nothing."""
    traj = _traj()
    at = traj.pose_at(1.0)
    for defl in (0.0, SCRUB_DEADBAND * 0.99, -SCRUB_DEADBAND * 0.99):
        rs = scrub_step(traj, 1.0, at, 0.05, defl)
        assert rs.cursor == 1.0, f"deflection {defl} moved a released scrub"
        assert not rs.finished


def test_forward_plays_and_backward_rewinds() -> None:
    traj = _traj()
    at = traj.pose_at(1.0)
    fwd = scrub_step(traj, 1.0, at, 0.05, 1.0)
    back = scrub_step(traj, 1.0, at, 0.05, -1.0)
    assert fwd.cursor > 1.0, "full forward did not advance"
    assert back.cursor < 1.0, "full backward did not rewind"
    assert abs(fwd.cursor - 1.0) == abs(1.0 - back.cursor), \
        "forward and backward paces differ for the same deflection"


def test_the_rate_is_linear_past_the_deadband_and_capped() -> None:
    """⭐ Half a push is half the pace — a scrub wheel, never a switch."""
    assert scrub_rate(1.0) == SCRUB_MAX_RATE
    assert scrub_rate(-1.0) == -SCRUB_MAX_RATE
    mid = (1.0 + SCRUB_DEADBAND) / 2.0
    assert abs(scrub_rate(mid) - SCRUB_MAX_RATE / 2.0) < 1e-9
    assert scrub_rate(2.0) == SCRUB_MAX_RATE, "an out-of-range reading must still cap"


def test_the_cursor_clamps_at_both_ends_and_never_finishes() -> None:
    """⛔ Property 2. The end of the recording is the last frame, not an exit."""
    traj = _traj(duration=2.0)
    end = scrub_step(traj, 1.98, traj.pose_at(1.98), 0.5, 1.0)
    assert end.cursor == traj.duration and not end.finished, \
        "the scrub ran past the end or declared itself finished"
    start = scrub_step(traj, 0.02, traj.pose_at(0.02), 0.5, -1.0)
    assert start.cursor == 0.0 and not start.finished, "the scrub ran below zero"


def test_a_lagging_arm_freezes_the_cursor_in_both_directions() -> None:
    """⛔ Property 3. The measured pose is 0.3 rad behind the commanded one, past the
    0.15 default — the cursor must hold whichever way the hand drags."""
    traj = _traj()
    behind = [v - 0.3 for v in traj.pose_at(1.0)]
    for defl in (1.0, -1.0):
        rs = scrub_step(traj, 1.0, behind, 0.05, defl)
        assert rs.held and rs.cursor == 1.0, \
            f"a lagging arm did not hold the cursor at deflection {defl}"


def test_the_time_lapse_dial_scales_the_pace() -> None:
    """⭐ His 2026-08-18 ask: "more than normal speed if I fully press the control
    forward... time lapse speed". A full push at max_rate 3 covers twice the recording
    time a full push at 1.5 covers, and the dial's ladder reaches both of its ends."""
    traj = _traj(duration=8.0)
    at = traj.pose_at(2.0)
    slow = scrub_step(traj, 2.0, at, 0.1, 1.0, max_rate=1.5)
    fast = scrub_step(traj, 2.0, at, 0.1, 1.0, max_rate=3.0)
    assert abs((fast.cursor - 2.0) - 2.0 * (slow.cursor - 2.0)) < 1e-9, \
        "doubling max_rate did not double the full-push pace"
    assert "scrub_max" in TUNABLE, "the dial cannot be saved as a default"
    value = LIVE_BOUNDS["scrub_max"][0]
    for _ in range(12):
        value = adjust("scrub_max", value, True)
    assert value == LIVE_BOUNDS["scrub_max"][1] == LADDERS["scrub_max"][-1], \
        "the dial's + key cannot reach its ceiling"


def test_grippers_can_be_left_out_of_the_lag_check() -> None:
    """⚠️ Same rule as replay_step: jaws sit far from their command while gripping, and
    counting that as lag would stall every scrub that holds an object."""
    traj = _traj(n=7)
    measured = list(traj.pose_at(1.0))
    measured[6] -= 5.0  # the jaw is wildly off; joints 0-5 are perfect
    rs = scrub_step(traj, 1.0, measured, 0.05, 1.0, compare=range(6))
    assert not rs.held and rs.cursor > 1.0, "a gripping jaw stalled the scrub"


def main() -> int:
    tests = [(n, f) for n, f in sorted(globals().items())
             if n.startswith("test_") and callable(f)]
    failed = []
    for name, fn in tests:
        try:
            fn()
            print(f"  ✓ {name}")
        except AssertionError as exc:
            failed.append((name, str(exc)))
            print(f"  ✗ {name}\n      {exc}")
        except Exception as exc:  # noqa: BLE001
            failed.append((name, f"{type(exc).__name__}: {exc}"))
            print(f"  ✗ {name}\n      {type(exc).__name__}: {exc}")
    print(f"\n{len(tests) - len(failed)}/{len(tests)} passed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
