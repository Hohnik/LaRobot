#!/usr/bin/env python3
"""How fast can this MACHINE run the control loop, with nothing attached? Measure it.

    uv run apps/bench_loop.py                 # about 25 s, no hardware, no root
    uv run apps/bench_loop.py --seconds 60    # longer, for a quieter number
    uv run apps/bench_loop.py --json          # one line of JSON, for a log

⭐⭐ WHY THIS EXISTS, and it refuted a number this repo had repeated for a week. [ROADMAP §8.2](../docs/ROADMAP.md) item 14 asked since 2026-08-13 why the loop runs at about 87 passes a second against a 100 Hz target. The candidates on the list were the cameras, machine load and the tracking log. **All of them were wrong.**

The bottom of the control loop is one line, `time.sleep(max(0.0, dt - elapsed))`. It asks the operating system to wake it in whatever is left of a 10 ms pass. **An operating system may wake you later than you asked, and by how much is a property of the operating system.** On macOS it is about 1.9 ms every pass, which is the whole of the Mac's shortfall. On the Linux station it is about 0.37 ms.

So the same loop, running the same code and doing no work at all, reaches **84.3 Hz on the Mac and 97.3 Hz on the station** ([FINDINGS §78.5](../docs/FINDINGS.md)).

⛔ THIS TOUCHES NO HARDWARE. It sleeps, it reads a clock, and it calls `MirrorLink.step` on arrays of zeros. There is no CAN traffic, no camera, no motor, no root and no arm. It is safe to run on any machine at any time, including while someone else is using the bench.

⚠️ WHAT IT CANNOT TELL YOU. What a REAL pass costs on that machine. A real pass also commands and reads 14 motors over two USB adapters, and that traffic is absent here. This is the floor: the rate the loop would reach if the work were free. Compare it against what a real session prints when it stops, which is `src/yam/timing.py`.

⭐ WHY THE MIRROR NUMBER IS IN HERE TOO. Julien asked on 2026-08-20 why copying one arm's joint angles onto another can possibly be slow. It cannot, and this prints the measurement rather than the assurance: the whole mirror decision against a 6-microsecond budget. [docs/LAG.md](../docs/LAG.md) is the full answer.
"""

from __future__ import annotations

import argparse
import json
import platform
import statistics
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "src"))

import numpy as np  # noqa: E402

from yam.mirror import MirrorLink, follower_target  # noqa: E402

#: The loop's own target, so this measures the thing the session actually does. Kept here rather than imported because `apps/teleop_session.py` is 5000 lines and importing it starts a session's worth of module-level work.
CONTROL_HZ = 100.0


def sleep_overshoot(ask_s: float, samples: int) -> list[float]:
    """How much longer than `ask_s` does `time.sleep` actually take, in milliseconds."""
    out = []
    for _ in range(samples):
        t0 = time.perf_counter()
        time.sleep(ask_s)
        out.append((time.perf_counter() - t0 - ask_s) * 1000.0)
    return out


def empty_loop(seconds: float, target_hz: float) -> list[float]:
    """Pass intervals of a loop that does NOTHING but wait, in milliseconds.

    ⛔ The waiting line is copied from `apps/teleop_session.py` deliberately, character for character. A paraphrase here would measure a different program.
    """
    dt = 1.0 / target_hz
    intervals: list[float] = []
    start = time.perf_counter()
    prev = None
    while time.perf_counter() - start < seconds:
        loop_start = time.perf_counter()
        t = loop_start - start
        if prev is not None:
            intervals.append((t - prev) * 1000.0)
        prev = t
        time.sleep(max(0.0, dt - (time.perf_counter() - loop_start)))
    return intervals


def mirror_cost(samples: int) -> tuple[float, float, set[str]]:
    """`(µs for follower_target, µs for the whole step, the link states seen)`, medians.

    ⛔⭐⭐ THE LEADER HERE MOVES SLOWLY ON PURPOSE, AND THE FIRST VERSION OF THIS DID NOT. A `MirrorLink` whose leader runs away from it **stops itself**, and a stopped link returns from `step` early, before the rate limiter and the per-joint gap check. Measuring that mixture gave 5.5 µs, and a real FOLLOWING pass costs about 8. The states seen are returned so the caller can print them, because a benchmark of an early return is worse than no benchmark ([FINDINGS §78.7](../docs/FINDINGS.md)).

    So: the leader moves at 0.2 rad/s, and the follower is fed the command the link produced on the previous pass. That is a link that keeps up, which is the case worth timing.
    """
    dt = 1.0 / CONTROL_HZ
    link = MirrorLink(mode="copy")
    lead = np.zeros(7)
    foll = np.zeros(7)
    for _ in range(50):                      # warm up numpy and the branch predictor
        link.step(lead, foll, dt)
    target_us, step_us, states = [], [], set()
    for i in range(samples):
        lead = np.full(7, 0.2 * i * dt)
        if link.command is not None:
            foll = link.command.copy()
        t0 = time.perf_counter()
        follower_target(lead, "copy")
        target_us.append((time.perf_counter() - t0) * 1e6)
        t0 = time.perf_counter()
        link.step(lead, foll, dt)
        step_us.append((time.perf_counter() - t0) * 1e6)
        states.add(link.state)
    return statistics.median(target_us), statistics.median(step_us), states


def p(values: list[float], q: float) -> float:
    """The value at percentile `q`, without pulling in numpy for one number."""
    if not values:
        return 0.0
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, int(q / 100.0 * len(ordered)))]


def main() -> int:
    ap = argparse.ArgumentParser(
        description="Measure what this machine's control loop can reach with no work in it.")
    ap.add_argument("--seconds", type=float, default=20.0,
                    help="how long to run the empty loop (default 20)")
    ap.add_argument("--samples", type=int, default=300,
                    help="sleep-overshoot samples per size (default 300)")
    ap.add_argument("--target-hz", type=float, default=CONTROL_HZ,
                    help="the loop rate to aim for (default 100)")
    ap.add_argument("--json", action="store_true",
                    help="print one line of JSON instead of a table")
    args = ap.parse_args()

    host = f"{platform.node()} {platform.system()} {platform.release()}"
    dt = 1.0 / args.target_hz

    # ⛔⭐⭐ THE MIRROR TIMING GOES FIRST, AND THE ORDER IS THE MEASUREMENT. Taken AFTER the
    # sleeping loop below it reads 15 to 19 µs; taken first it reads 8 to 10. The loop spends
    # its time asleep, the CPU drops to a low-power state, and the first microsecond-scale
    # measurement afterwards is paying for the clock ramping back up. A millisecond-scale
    # measurement cannot see that, which is why only this one had to move
    # ([FINDINGS §78.7](../docs/FINDINGS.md)).
    target_us, step_us, mirror_states = mirror_cost(3000)
    asks = (dt * 0.8, dt * 0.9)
    over = {f"{a * 1000:.1f}": sleep_overshoot(a, args.samples) for a in asks}
    intervals = empty_loop(args.seconds, args.target_hz)

    mean_ms = statistics.mean(intervals) if intervals else 0.0
    result = {
        "host": host,
        "python": platform.python_version(),
        "target_hz": args.target_hz,
        "sleep_overshoot_ms": {k: round(statistics.median(v), 3) for k, v in over.items()},
        "sleep_overshoot_p95_ms": {k: round(p(v, 95), 3) for k, v in over.items()},
        "empty_loop_passes": len(intervals),
        "empty_loop_mean_ms": round(mean_ms, 3),
        "empty_loop_hz": round(1000.0 / mean_ms, 2) if mean_ms else 0.0,
        "empty_loop_worst_ms": round(max(intervals), 3) if intervals else 0.0,
        "mirror_follower_target_us": round(target_us, 3),
        "mirror_step_us": round(step_us, 3),
        "mirror_states_seen": sorted(mirror_states),
    }

    if args.json:
        print(json.dumps(result))
        return 0

    print(f"\n⭐ {host}, Python {platform.python_version()}")
    print(f"   target {args.target_hz:.0f} Hz, so {dt * 1000:.1f} ms a pass\n")
    print("   time.sleep returns late by:")
    for k, v in over.items():
        print(f"     sleep({k} ms):  median {statistics.median(v):6.3f} ms   "
              f"p95 {p(v, 95):6.3f}   worst {max(v):6.3f}")
    print(f"\n   an EMPTY loop, {len(intervals)} passes over {args.seconds:g} s:")
    print(f"     mean {mean_ms:6.2f} ms a pass  =  {result['empty_loop_hz']:.1f} Hz   "
          f"(worst pass {result['empty_loop_worst_ms']:.2f} ms)")
    print(f"\n   the mirror decision, measured BEFORE the sleeping loop "
          f"(link state: {', '.join(sorted(mirror_states))}):")
    print(f"     follower_target  {target_us:6.1f} µs")
    print(f"     MirrorLink.step  {step_us:6.1f} µs   "
          f"= {step_us / (dt * 1e6) * 100:.3f}% of one pass")
    print("     ⚠️ expect 8 to 10 µs on a busy CPU and 15 to 19 on one just woken from idle")
    gap = args.target_hz - result["empty_loop_hz"]
    print(f"\n⛔ This machine loses {gap:.1f} Hz of the {args.target_hz:.0f} Hz target "
          f"to WAITING, with no work in the loop at all.")
    print("   A real session adds the CAN traffic on top. Its own numbers print when it stops.")
    print("   Reference, 2026-08-20: Mac 84.3 Hz, Linux station 97.3 Hz (FINDINGS §78.5).\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
