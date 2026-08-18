#!/usr/bin/env python3
"""Tests for velocity feedforward (docs/ROADMAP.md §8.2 item 44). No hardware.

    uv run scripts/test_vel_ff.py

⭐ WHAT THE FEATURE IS. The DM motors' MIT-mode frame carries a velocity setpoint and
this stack always sent zero, so every joint's torque had to come from position error —
which is the measured `0.033 s × speed` lag (docs/FINDINGS.md §66.1). With
`SafeRobot.vel_ff > 0`, each command also carries that fraction of the RATE-LIMITED
command's own derivative as the velocity setpoint.

⛔ THE PROPERTIES THAT MUST HOLD, because this touches what 4.3 kg does:
1. `vel_ff = 0.0` (the default) is EXACTLY the old behaviour — position-only commands.
2. The feedforward can never ask for more speed than the rate limiter allows:
   |vel| ≤ max_speed × vel_ff by construction, because it is the derivative of the
   command the limiter itself produced.
3. The jaw (index 6) NEVER receives feedforward — extra torque on a squeezing jaw
   pushes harder into the object, which is how motor 7 was cooked.
4. A wrapped robot without `command_joint_state` falls back to position-only and SAYS
   so once, rather than silently dropping the feature (docs/FINDINGS.md §0).

⚠️ What no test here can say: how much feedforward tightens tracking on the real arm.
The fake records the setpoints and deliberately does not model the benefit — a constant
for it would be invented, not measured (docs/FINDINGS.md §33.3).
"""

from __future__ import annotations

import io
import sys
from contextlib import redirect_stdout
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parent.parent

from yam.fake.arm import FakeArm  # noqa: E402
from yam.settings import LADDERS, LIVE_BOUNDS, LIVE_ORDER, TUNABLE, adjust  # noqa: E402
from yam.robot import VEL_FF_CEILING, SafeRobot  # noqa: E402

N = 7  # 6 arm joints + the jaw, this stack's only real shape


def _pair(vel_ff: float = 0.0, max_speed: float = 1.0) -> tuple[SafeRobot, FakeArm]:
    fake = FakeArm(n_joints=N)
    safe = SafeRobot(fake, max_speed=max_speed, max_lag=0.25)
    safe.vel_ff = vel_ff
    return safe, fake


def test_off_by_default_and_position_only() -> None:
    """⛔ Property 1: the default is 0.0 and the old path runs unchanged."""
    safe, fake = _pair()
    assert safe.vel_ff == 0.0
    safe.command_joint_pos(np.full(N, 0.5))
    assert len(fake.commands) == 1, "the position command did not arrive"
    assert fake.commanded_vels == [], "vel_ff=0 must never send a velocity setpoint"


def test_a_gain_sends_the_limited_commands_own_derivative() -> None:
    """⭐ First call: dt is the nominal 0.02 s, the limiter grants max_speed × dt, so the
    velocity setpoint must be exactly max_speed × vel_ff on every saturated joint."""
    safe, fake = _pair(vel_ff=0.5, max_speed=1.0)
    target = np.full(N, 10.0)  # far away → every joint saturates its budget
    target[6] = 0.5            # jaw asks for a modest close
    safe.command_joint_pos(target)
    assert len(fake.commanded_vels) == 1, "vel_ff>0 must use command_joint_state"
    vel = fake.commanded_vels[0]
    for j in range(6):
        assert abs(vel[j] - 0.5 * 1.0) < 1e-9, \
            f"joint {j}: expected max_speed × vel_ff = 0.5, got {vel[j]}"


def test_the_jaw_never_gets_feedforward() -> None:
    """⛔ Property 3: index 6 is the jaw and its setpoint must be exactly zero even when
    the jaw command itself is moving."""
    safe, fake = _pair(vel_ff=1.0)
    target = np.zeros(N)
    target[6] = 1.0  # only the jaw moves
    safe.command_joint_pos(target)
    assert fake.commanded_vels[0][6] == 0.0, "the jaw received feedforward"


def test_feedforward_is_bounded_by_the_rate_limiter() -> None:
    """⛔ Property 2: across many cycles with a runaway target, no setpoint may exceed
    max_speed × vel_ff — the limiter's budget divided by the same dt it was sized with."""
    safe, fake = _pair(vel_ff=1.0, max_speed=2.0)
    for step in range(50):
        safe.command_joint_pos(np.full(N, 1000.0 + step))
    worst = max(float(np.max(np.abs(v[:6]))) for v in fake.commanded_vels)
    assert worst <= 2.0 + 1e-6, f"feedforward asked for {worst} rad/s past the limiter"


def test_a_gain_above_the_ceiling_is_clamped() -> None:
    """⚠️ A flag can type anything. Above VEL_FF_CEILING the setpoint clamps, so a typo
    like `--vel-ff 50` cannot ask for fifty times the command speed."""
    safe, fake = _pair(vel_ff=50.0, max_speed=1.0)
    safe.command_joint_pos(np.full(N, 10.0))
    worst = float(np.max(np.abs(fake.commanded_vels[0][:6])))
    assert worst <= VEL_FF_CEILING * 1.0 + 1e-9, \
        f"a runaway gain sent {worst} rad/s past the ceiling"


def test_the_ceiling_is_one_number_in_two_files_and_they_agree() -> None:
    """⛔ `yam_robot.VEL_FF_CEILING` clamps the code; `settings.LIVE_BOUNDS` bounds the
    editor and the ladder tops out there. If they drift, the screen shows a value the
    arm silently refuses — the exact silent-disagreement this repo keeps paying for."""
    assert VEL_FF_CEILING == LIVE_BOUNDS["vel_ff"][1], \
        "yam_robot.VEL_FF_CEILING and settings.LIVE_BOUNDS['vel_ff'] disagree"
    assert LADDERS["vel_ff"][-1] == VEL_FF_CEILING, \
        "the ladder's top rung is not the ceiling, so the max is unreachable by key"


def test_a_robot_without_command_joint_state_falls_back_and_says_so_once() -> None:
    """⛔ Property 4: silent feature loss is the fails-by-lying pattern. The fallback
    must keep commanding positions AND print exactly one warning."""

    class Stub:
        def __init__(self) -> None:
            self.q = np.zeros(N)
            self.got: list[np.ndarray] = []

        def get_joint_pos(self) -> np.ndarray:
            return self.q.copy()

        def command_joint_pos(self, q: np.ndarray) -> None:
            self.got.append(np.asarray(q, dtype=float))

    stub = Stub()
    safe = SafeRobot(stub, max_speed=1.0, max_lag=0.25)
    safe.vel_ff = 0.5
    out = io.StringIO()
    with redirect_stdout(out):
        safe.command_joint_pos(np.full(N, 1.0))
        safe.command_joint_pos(np.full(N, 1.0))
    assert len(stub.got) == 2, "the fallback stopped commanding positions"
    assert out.getvalue().count("feedforward is OFF") == 1, \
        "the fallback must warn exactly once, not never and not per cycle"


def test_the_live_setting_exists_and_its_ladder_reaches_both_ends() -> None:
    """⭐ vel_ff is setting 9 on the `n` screen: tunable, saved, bounded 0..3, and both
    OFF (0) and the ceiling must be reachable by key. 1.0 (the physically-motivated
    value) must be a rung; above it is exaggeration, his 2026-08-18 ask."""
    assert "vel_ff" in TUNABLE and "vel_ff" in LIVE_ORDER
    assert LIVE_ORDER.index("vel_ff") == 8, "the help text says setting 9"
    assert LIVE_BOUNDS["vel_ff"] == (0.0, VEL_FF_CEILING)
    assert 1.0 in LADDERS["vel_ff"], "1.0 = exact command speed must stay a rung"
    value = LIVE_BOUNDS["vel_ff"][1]
    for _ in range(15):
        value = adjust("vel_ff", value, False)
    assert value == 0.0, "OFF must be reachable by pressing -"
    for _ in range(15):
        value = adjust("vel_ff", value, True)
    assert value == VEL_FF_CEILING, "the ceiling must be reachable by pressing +"
    assert LADDERS["vel_ff"][0] == 0.0


def test_a_joint_past_its_command_gets_zero_push_immediately() -> None:
    """⛔ FINDINGS §68.3, his jitter report at gain 3: above gain 1 a joint overshoots the
    rate-limited command, and the setpoint kept pushing while the position term pulled
    back — the forward jitter, and the visible pull-back at release. A joint whose
    position error opposes its setpoint now gets zero push, immediately."""
    safe, fake = _pair(vel_ff=3.0, max_speed=1.0)
    safe.command_joint_pos(np.full(N, 10.0))          # drives forward, arm still at 0
    fake.q[:] = 5.0                                    # teleport the arm PAST the command
    safe.command_joint_pos(np.full(N, 10.0))          # command still moving forward
    sent = fake.commanded_vels[-1]
    assert np.all(sent[:6] == 0.0), \
        f"a joint past its command was still pushed: {sent[:6]}"


def test_release_decays_the_push_instead_of_cutting_it() -> None:
    """⭐ The smoothing half of FINDINGS §68.3: when the hand releases, the raw derivative
    drops to zero in one cycle, and the sent setpoint must DECAY over a few cycles (an
    unsmoothed cut is the small "latent movement" he felt below gain 1)."""
    safe, fake = _pair(vel_ff=1.0, max_speed=1.0)
    safe.command_joint_pos(np.full(N, 10.0))          # moving: full setpoint
    first = fake.commanded_vels[-1][0]
    assert first > 0.0
    frozen = np.asarray(fake.cmd, dtype=float).copy() # the pose the limiter last sent
    last = first
    for _ in range(3):                                 # command the SAME pose: raw vel 0
        safe.command_joint_pos(frozen)
        now = fake.commanded_vels[-1][0]
        assert 0.0 <= now < last, f"the release did not decay: {now} after {last}"
        last = now


def test_resync_clears_the_smoothed_setpoint() -> None:
    """⛔ The smoothed feedforward is state; carrying it across a mode change would push
    the new mode's first command with the old mode's speed — the park-spasm family
    (FINDINGS §66.0) one layer down."""
    safe, fake = _pair(vel_ff=1.0)
    safe.command_joint_pos(np.full(N, 10.0))
    assert safe._ff_prev is not None  # noqa: SLF001
    safe.resync()
    assert safe._ff_prev is None, "resync left a stale feedforward behind"  # noqa: SLF001


def test_the_fake_records_what_the_real_motor_would_be_sent() -> None:
    """⭐ The fake's physics ignores the setpoint ON PURPOSE (an unmeasured benefit is an
    invented constant), but it must move exactly as it would for the same positions."""
    plain, plain_fake = _pair(vel_ff=0.0)
    with_ff, ff_fake = _pair(vel_ff=1.0)
    for step in range(10):
        target = np.full(N, 0.1 * (step + 1))
        plain.command_joint_pos(target)
        with_ff.command_joint_pos(target)
    assert len(ff_fake.commanded_vels) == 10
    assert np.allclose(plain_fake.commands, ff_fake.commands, atol=1e-9), \
        "feedforward changed the POSITION commands, which it must never do"


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
