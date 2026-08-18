#!/usr/bin/env python3
"""Tests for the simulated arm, `src/yam/fake/arm.py`. No hardware.

    uv run scripts/test_fake_arm.py

⭐⭐ THE TEST THAT MATTERS MOST IS `test_the_fake_reproduces_the_MEASURED_lag_law`.
Everything else here checks that the fake does what I wrote; that one checks it does
what the **real arm** does, against a law fitted on 2026-08-13 from three playbacks and
then tested on held-out data (docs/ROADMAP.md §8.2 item 11):

    following error ≈ 0.04 to 0.10 rad  +  0.033 s × speed

⛔ Without that test this file would be circular — a fake asserting its own constants,
which is the shape docs/FINDINGS.md §0 warns about: confident, plausible, and proving
nothing. **A simulator nobody has compared to reality is a source of false confidence,
which is worse than no simulator**, because bugs it "clears" get shipped to hardware.
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parent.parent

from yam.fake.arm import (  # noqa: E402
    AMBIENT_C,
    MEASURED_DEADBAND_RAD,
    MEASURED_TAU_S,
    FakeArm,
    build_fake_robot,
)
from yam.robot import motor_temperatures  # noqa: E402


class Clock:
    """A hand-cranked clock, so no test depends on how loaded the machine is."""

    def __init__(self) -> None:
        self.t = 0.0

    def __call__(self) -> float:
        return self.t

    def tick(self, dt: float) -> None:
        self.t += dt


def drive(arm: FakeArm, clock: Clock, speed: float, seconds: float,
          dt: float = 0.01, joint: int = 0) -> float:
    """Ramp one joint's target at `speed` rad/s for `seconds`. Returns the final
    following error on that joint, which is what the measured law predicts."""
    cmd = arm.get_joint_pos()
    steps = int(seconds / dt)
    for _ in range(steps):
        cmd = cmd.copy()
        cmd[joint] += speed * dt
        clock.tick(dt)
        arm.command_joint_pos(cmd)
    return float(abs(cmd[joint] - arm.get_joint_pos()[joint]))


# --------------------------------------------------------------- the anchoring test


def test_the_fake_reproduces_the_MEASURED_lag_law() -> None:
    """⭐⭐ THE ONE TEST THAT MAKES THIS SIMULATOR WORTH TRUSTING.

    The real arm was measured to follow with `0.04 to 0.10 rad + 0.033 s × speed` of
    error, the same on all six arm joints. This drives the fake at four speeds and
    asserts every one lands inside that band.

    ⚠️ The band is wide because the real measurement's constant term was a range, not a
    point. That is honest: a fake pinned to one endpoint would look more precise than
    the evidence it came from.
    """
    for speed in (0.25, 0.5, 1.0, 2.0):
        clock = Clock()
        arm = FakeArm(clock=clock)
        err = drive(arm, clock, speed, seconds=1.5)
        low = 0.04 + 0.033 * speed
        high = 0.10 + 0.033 * speed
        assert low <= err <= high, (
            f"at {speed} rad/s the fake settled {err:.4f} rad behind, outside the "
            f"measured band {low:.4f}-{high:.4f}. Either a constant changed or the "
            f"physics no longer matches docs/ROADMAP.md item 11")


def test_the_error_GROWS_with_speed_rather_than_staying_flat() -> None:
    """⚠️ The band above is wide enough that a fake with NO speed term could sit inside
    it at every speed tested. This pins the slope, so a constant-error fake fails."""
    errs = []
    for speed in (0.25, 2.0):
        clock = Clock()
        arm = FakeArm(clock=clock)
        errs.append(drive(arm, clock, speed, seconds=1.5))
    assert errs[1] > errs[0] + 0.02, (
        f"error barely moved with speed ({errs[0]:.4f} → {errs[1]:.4f}); the "
        f"speed-proportional half of the law is not being reproduced")


# ------------------------------------------------------------------- the basic shape


def test_a_command_is_FOLLOWED_over_time_not_teleported() -> None:
    """⭐⭐ THE WHOLE REASON THIS MODULE EXISTS. The older fakes in
    `scripts/test_park_arms.py` set the measured position to the command, so following
    error cannot exist there and every bug living in that gap is unreachable."""
    clock = Clock()
    arm = FakeArm(clock=clock)
    target = np.full(7, 1.0)
    clock.tick(0.01)
    arm.command_joint_pos(target)
    after_one = arm.get_joint_pos()[0]
    assert 0.0 < after_one < 1.0, (
        f"one cycle moved the joint to {after_one}; it should be part-way, not there")
    for _ in range(400):
        clock.tick(0.01)
        arm.command_joint_pos(target)
    assert abs(arm.get_joint_pos()[0] - 1.0) < 0.06, "it never arrived"


def test_reading_the_position_does_NOT_move_the_arm() -> None:
    """⚠️ A real arm holds its last commanded pose. If the physics advanced on a read,
    an idle arm would drift, and the session reads the position several times a cycle."""
    clock = Clock()
    arm = FakeArm(clock=clock)
    clock.tick(0.01)
    arm.command_joint_pos(np.full(7, 1.0))
    first = arm.get_joint_pos()
    clock.tick(5.0)
    for _ in range(10):
        arm.get_joint_pos()
    assert np.allclose(first, arm.get_joint_pos()), "the arm moved without a command"


def test_a_command_inside_the_deadband_moves_NOTHING() -> None:
    """⭐ Static friction is the constant term in the measured law. A joint asked to
    move 0.01 rad does not move, which is why the real arm settles with residual error
    instead of converging exactly."""
    clock = Clock()
    arm = FakeArm(clock=clock)
    small = np.zeros(7)
    small[0] = MEASURED_DEADBAND_RAD * 0.5
    for _ in range(200):
        clock.tick(0.01)
        arm.command_joint_pos(small)
    assert abs(arm.get_joint_pos()[0]) < 1e-9, (
        f"a {small[0]:.3f} rad command moved the joint to {arm.get_joint_pos()[0]}")


def test_the_wrong_number_of_joints_RAISES() -> None:
    """⛔ This is the two-arm playback bug in one line: a 7-element snapshot fed against
    14-element targets. It was silent on hardware. Here it cannot be."""
    arm = FakeArm(clock=Clock())
    try:
        arm.command_joint_pos(np.zeros(14))
    except ValueError as exc:
        assert "14" in str(exc) or "7" in str(exc)
    else:
        raise AssertionError("a 14-element command on a 7-joint arm was accepted")


# ----------------------------------------------------------------- the failure paths


def test_a_blocked_joint_STOPS_and_the_following_error_grows() -> None:
    """⭐⭐ THE PATH NOTHING COULD REACH WITHOUT HARDWARE. An object in the way is how
    every stall detector, the jaw-block problem (docs/ROADMAP.md §8.2 item 29) and
    SafeRobot's lag clip are actually triggered."""
    clock = Clock()
    arm = FakeArm(clock=clock)
    arm.block(0, -10.0, 0.20)          # joint 0 cannot go past 0.20 rad
    cmd = np.zeros(7)
    for _ in range(300):
        cmd = cmd.copy()
        cmd[0] += 0.01
        clock.tick(0.01)
        arm.command_joint_pos(cmd)
    assert abs(arm.get_joint_pos()[0] - 0.20) < 1e-6, "the block did not hold"
    assert arm.following_error() > 0.5, (
        f"the command should have run far past the block; error is only "
        f"{arm.following_error():.3f} rad")


def test_a_dead_chain_is_visible_the_way_the_session_checks_it() -> None:
    """⚠️ `ArmSession.alive()` reads `robot.motor_chain.running` through `getattr`, so
    this pins the attribute PATH as much as the value."""
    arm = FakeArm(clock=Clock())
    assert arm.motor_chain.running is True
    arm.kill_chain()
    assert arm.motor_chain.running is False


def test_an_unreadable_chain_makes_the_thermal_guard_BLIND() -> None:
    """⭐ `src/yam/robot.py::ThermalGuard` was rewritten specifically because a failed
    temperature read was being treated as a safe temperature. That path needed an
    unplugged cable to reach until now."""
    arm = FakeArm(clock=Clock())
    arm.motor_chain.blind = True
    try:
        arm.motor_chain.read_states()
    except RuntimeError:
        pass
    else:
        raise AssertionError("a blind chain returned states instead of raising")


# --------------------------------------------------- the contract with REAL functions


def test_the_state_fields_satisfy_the_REAL_motor_temperatures() -> None:
    """⛔⭐⭐ THE MISSPELLING TRAP, CHECKED AGAINST THE REAL FUNCTION.

    `motor_temperatures` reads `getattr(s, "temp_mos", 0)`. A field named `temp_mosfet`
    here would not raise: it would read **zero**, and a zero temperature is exactly how
    a thermal guard gets quietly disarmed. So this calls the real function on real fake
    states and insists the answer is a plausible room temperature.
    """
    arm = FakeArm(clock=Clock())
    temps, hottest, jaw = motor_temperatures(arm.states(), 6)
    assert len(temps) == 7, f"expected 7 temperatures, got {len(temps)}"
    assert hottest is not None, "hottest came back None from a healthy arm"
    assert 20.0 < hottest < 40.0, (
        f"hottest is {hottest} °C, which is not a room-temperature arm — the most "
        f"likely cause is a field name that motor_temperatures cannot see")
    assert jaw is not None and 20.0 < jaw < 40.0, f"gripper temperature is {jaw}"


def test_pushing_warms_the_arm_and_releasing_cools_it() -> None:
    """⚠️ Shape only, and the test says so: it asserts the DIRECTION of change, never a
    number, because the thermal model is uncalibrated on purpose."""
    clock = Clock()
    arm = FakeArm(clock=clock)
    arm.block(0, -10.0, 0.0)           # joint 0 cannot move at all
    cmd = np.zeros(7)
    cmd[0] = 2.0                       # so it pushes hard and gets nowhere
    for _ in range(500):
        clock.tick(0.02)
        arm.command_joint_pos(cmd)
    hot = arm.states()[0].temp_mos
    assert hot > AMBIENT_C + 5.0, f"pushing into a block barely warmed it: {hot:.1f} °C"

    arm.cmd = arm.q.copy()             # stop pushing
    for _ in range(2000):
        arm.step(0.02)
    assert arm.states()[0].temp_mos < hot - 5.0, "it never cooled down"


# ------------------------------------------------------ the REAL SafeRobot around it


def test_build_fake_robot_wraps_the_REAL_SafeRobot_and_its_limits_BITE() -> None:
    """⭐⭐ The wrapper is the real safety class, not a copy of it. A copy would test
    the copy. This asks for a jump of one radian and checks the rate limiter refuses."""
    robot, note = build_fake_robot("B", max_speed=1.0, max_lag=0.25)
    assert "SIMULATED" in note and "No gravity" in note, (
        f"the note must say what it cannot do; got {note!r}")
    start = robot.get_joint_pos().copy()
    robot.command_joint_pos(np.full(7, 1.0))
    moved = float(np.max(np.abs(robot.get_joint_pos() - start)))
    assert moved < 0.2, (
        f"one cycle moved {moved:.3f} rad; SafeRobot's rate limit is not in the path")
    assert robot.limited_cycles >= 1, "SafeRobot never recorded that a limit bit"


def test_SafeRobot_s_lag_clip_holds_the_command_near_a_BLOCKED_arm() -> None:
    """⛔⭐⭐ THIS IS THE LIMIT JULIEN ASKED ABOUT, AND THE ONE THAT ACTUALLY BINDS.

    docs/FINDINGS.md §58.3: `max_lag` (0.25 rad), not `max_speed`, is the real ceiling
    on tracking. With the arm blocked, the command cannot run more than `max_lag` past
    the measured position however long it is asked to. **That behaviour has never been
    checkable without a physical obstruction.**

    ⛔⭐⭐ THIS TEST WAS BLIND WHEN FIRST WRITTEN, AND A FALSIFICATION RUN CAUGHT IT.
    It asserted only `gap <= max_lag`. **An arm that is not blocked at all keeps up with
    its command, so its gap is near zero and satisfies that bound trivially** — the test
    passed just as happily with the blocking code deleted. It was the third weak test of
    exactly this shape this week: evidence that cannot tell the claim from its opposite.

    ⭐ The fix is a TWO-SIDED assertion. The command must be held **at** the clip, not
    merely under it: the joint is stuck where the obstacle is, and the gap has been
    driven right up against `max_lag`. Deleting the block now fails the lower bound.
    """
    robot, _ = build_fake_robot("B", max_speed=5.0, max_lag=0.25)
    fake = robot._robot                          # noqa: SLF001 — the fake underneath
    fake.block(0, -10.0, 0.10)
    for _ in range(200):
        robot.command_joint_pos(np.full(7, 3.0))
    gap = abs(fake.cmd[0] - fake.q[0])
    # The arm really is stuck against the obstacle, so the gap is not small by accident.
    assert abs(fake.q[0] - 0.10) < 1e-6, (
        f"joint 0 is at {fake.q[0]:.4f}, not held at the 0.10 rad block — so whatever "
        f"this test measures next, it is not the lag clip")
    assert gap > 0.20, (
        f"the gap is only {gap:.3f} rad against a max_lag of 0.25. The command is not "
        f"being HELD at the clip, so this test would pass with no clip at all")
    assert gap <= 0.25 + 1e-6, (
        f"the command ran {gap:.3f} rad past a blocked joint, but max_lag is 0.25 — "
        f"the following-error clip is not holding")


def test_the_default_limits_come_from_the_REAL_constants() -> None:
    """⚠️ A simulated session must be limited by the same numbers as a real one, or it
    clears runs that hardware would refuse."""
    from yam.robot import SAFE_MAX_LAG, SAFE_MAX_SPEED

    robot, _ = build_fake_robot("B")
    assert robot.max_speed == SAFE_MAX_SPEED, "max_speed drifted from the real default"
    assert robot.max_lag == SAFE_MAX_LAG, "max_lag drifted from the real default"


def test_two_arms_can_be_built_and_are_INDEPENDENT() -> None:
    """⭐ The whole point is running the N-arm loop. Two fakes sharing state would make
    every mirror and two-arm-recording test meaningless."""
    b, _ = build_fake_robot("B")
    g, _ = build_fake_robot("G")
    b.command_joint_pos(np.full(7, 1.0))
    assert np.allclose(g.get_joint_pos(), 0.0), "commanding B moved G"
    assert b.num_dofs() == g.num_dofs() == 7


def test_resync_re_anchors_the_limiter_the_way_a_mode_change_needs() -> None:
    """⛔ `SafeRobot.resync()` exists because a stale cached command is what snapped the
    arm on 2026-08-10. The session calls it on every mode transition."""
    robot, _ = build_fake_robot("B", max_speed=1.0)
    for _ in range(20):
        robot.command_joint_pos(np.full(7, 1.0))
    robot.resync()
    assert robot._last_t is None, "resync left the timestamp in place"  # noqa: SLF001
    assert np.allclose(robot._last_cmd, robot.get_joint_pos()), (  # noqa: SLF001
        "resync did not re-anchor the command to the measured position")


def test_the_REAL_shutdown_path_disables_every_simulated_motor() -> None:
    """⭐⭐ THE MOST SAFETY-CRITICAL CODE IN THE PROJECT, TESTABLE WITHOUT HARDWARE AT LAST.

    `yam_robot.shutdown_robot()` stops the chain, then walks `motor_list` calling
    `motor_interface.motor_off(id)`, then closes — in that order, because `close()` does not
    disable despite announcing that it has. It returns the IDs whose `motor_off` genuinely
    succeeded.

    ⛔ Every one of those calls sits inside `except Exception: pass`. So a fake lacking these
    attributes does not raise: it silently reports **zero motors disabled**, and
    "confirmed disabled: []" reads as a fake being a fake rather than as a shutdown having
    failed. That is why the fake provides them.

    ⚠️ Until now this path had only ever been exercised by Julien pressing Ctrl-C on a live
    arm, which is a poor way to test the code that decides whether 4.3 kg is released.
    """
    from yam.robot import shutdown_robot

    robot, _ = build_fake_robot("B")
    fake = robot._robot                          # noqa: SLF001
    disabled = shutdown_robot(robot)
    assert disabled == [1, 2, 3, 4, 5, 6, 7], (
        f"shutdown_robot reported {disabled}, not all seven motors")
    assert fake.motor_chain.disabled == [1, 2, 3, 4, 5, 6, 7], (
        "motor_off was not actually called for every motor")
    assert fake.motor_chain.running is False, "the chain was left running"
    assert fake.motor_chain.closed is True, "the chain was never closed"


def test_the_chain_is_STOPPED_BEFORE_the_motors_are_disabled() -> None:
    """⛔ Order is the whole point of `shutdown_robot`, and its docstring says why: the
    control thread runs at 250 Hz and will otherwise be mid-`set_control` when the bus
    closes underneath it, which produced a thread-death traceback on the first real run."""
    from yam.robot import shutdown_robot

    robot, _ = build_fake_robot("B")
    fake = robot._robot                          # noqa: SLF001
    seen = []
    original = fake.motor_chain.motor_interface.motor_off

    def watched(motor_id, original=original):  # noqa: ANN001, ANN202
        seen.append(("motor_off", bool(fake.motor_chain.running)))
        original(motor_id)

    fake.motor_chain.motor_interface.motor_off = watched
    shutdown_robot(robot)
    assert seen, "motor_off was never called"
    assert all(running is False for _, running in seen), (
        "a motor was disabled while the chain was still running — the 250 Hz control "
        "thread would have been mid-command when the bus went away")


def test_a_still_puck_reports_no_deflection_and_the_button_field_exists() -> None:
    """⚠️ The session reads `read()` and a `buttons` attribute. A missing `buttons` would be
    read through `getattr(..., 0)` and so would not raise, which is why it is asserted."""
    from yam.fake.arm import StillPuck

    puck = StillPuck()
    assert puck.read() == [0.0] * 6
    assert puck.buttons == 0


def main() -> int:
    tests = [(n, f) for n, f in sorted(globals().items())
             if n.startswith("test_") and callable(f)]
    failed = []
    for name, fn in tests:
        try:
            fn()
            print(f"  ✓ {name}")
        except Exception as exc:  # noqa: BLE001
            failed.append(name)
            print(f"  ✗ {name}\n      {type(exc).__name__}: {exc}")
    print(f"\n{len(tests) - len(failed)}/{len(tests)} passed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
