#!/usr/bin/env python3
"""Tests for the thermal guard. No hardware, no CAN bus, no motors.

    uv run scripts/test_thermal_guard.py

⛔ WHY THESE EXIST. Motor 7 has been cooked three times on this rig. The thermal
guard is the only thing standing between the gripper and a stall burn, and on
2026-08-12 it turned out to have three holes — all found by reading the code
against what it claimed to do, none of which had ever fired on hardware:

1. Any exception from `chain.read_states()` set the hottest motor to **0.0**, so
   the stop comparison could not fire and the status line printed a calm
   `hottest 0°C`. Thermal protection was gone and nothing said so.
2. `TEMP_WARN = 55.0` was printed in the startup plan and, by exhaustive grep,
   used **nowhere else**. The session promised a warning it never issued.
3. Nothing distinguished "everything is cool" from "I cannot see".

Every test below is one of those, written so it fails if the hole comes back.
"""

from __future__ import annotations

import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "src"))

from yam_robot import ThermalGuard, motor_temperatures  # noqa: E402


class FakeState:
    """One motor's state, as the chain reports it."""

    def __init__(self, temp_mos=0.0, temp_rotor=0.0):  # noqa: ANN001
        self.temp_mos = temp_mos
        self.temp_rotor = temp_rotor


def test_the_hotter_of_the_two_sensors_wins() -> None:
    """Each motor reports MOSFET and rotor temperature. Picking one and ignoring
    the other would miss whichever is actually overheating."""
    temps, hottest, _ = motor_temperatures([FakeState(30, 52), FakeState(48, 20)], 6)
    assert temps == [52.0, 48.0]
    assert hottest == 52.0


def test_an_empty_state_list_reports_UNKNOWN_not_zero() -> None:
    """⛔⭐ THE CORE OF THE BUG. Zero degrees is a plausible-looking number that no
    motor in a warm room produces, and it silently passes every `>=` check there
    is. An absent reading must be None so the guard can tell "cool" from "blind"."""
    _, hottest, jaw = motor_temperatures([], 6)
    assert hottest is None, "an empty read must not be reported as 0 °C"
    assert jaw is None


def test_the_gripper_is_reported_separately_from_the_hottest_motor() -> None:
    """⭐ Motors 2-3 carry 4.3 kg and idle at 41-42 °C; motor 7 idles at 31-36 °C.
    A gripper climbing 33 → 41 °C is invisible inside a max() — and watching motor
    7 plateau is the whole test of the 2π frame fix."""
    states = [FakeState(41), FakeState(42), FakeState(42), FakeState(35),
              FakeState(34), FakeState(33), FakeState(41)]
    _, hottest, jaw = motor_temperatures(states, 6)
    assert hottest == 42.0
    assert jaw == 41.0, "the gripper must be readable on its own"
    assert jaw < hottest, "and this is exactly the case where max() would hide it"


def test_a_six_motor_chain_has_no_jaw_temperature() -> None:
    """`--no-gripper` runs 6 motors. Indexing state 7 would raise, and the old code
    swallowed that exception — killing temperature monitoring along with it."""
    _, hottest, jaw = motor_temperatures([FakeState(40)] * 6, 6)
    assert hottest == 40.0
    assert jaw is None


def test_a_cool_arm_produces_neither_stop_nor_warning() -> None:
    guard = ThermalGuard()
    for _ in range(500):
        verdict = guard.update(42.0, jaw=35.0)
        assert verdict.stop_reason is None and verdict.warning is None
    assert guard.max_seen == 42.0
    assert guard.max_jaw_seen == 35.0


def test_the_55_degree_warning_is_actually_issued() -> None:
    """⛔⭐ DEFECT 2. The startup plan printed "warn 55°C, stop 65°C" while
    TEMP_WARN appeared nowhere else in the codebase. A promise the code cannot
    keep is the same class of defect as a refusal naming the wrong arm."""
    guard = ThermalGuard(warn_at=55.0, stop_at=65.0)
    assert guard.update(54.0).warning is None
    verdict = guard.update(56.0, motor=2)
    assert verdict.warning is not None and "56" in verdict.warning
    assert verdict.stop_reason is None, "a warning must not stop the session"


def test_the_warning_does_not_repeat_every_cycle() -> None:
    """At 100 Hz a warning per cycle would bury the readout it is trying to
    interrupt, and a motor sitting on the line would print 100 times a second."""
    guard = ThermalGuard(warn_at=55.0)
    assert guard.update(56.0).warning is not None
    assert all(guard.update(56.0).warning is None for _ in range(100))


def test_the_warning_re_arms_after_the_motor_cools() -> None:
    """Warn once per excursion, not once per session — a motor that heats, cools
    and heats again is worth hearing about twice."""
    guard = ThermalGuard(warn_at=55.0, rearm_below=3.0)
    assert guard.update(56.0).warning is not None
    assert guard.update(51.0).warning is None      # cooled well below the line
    assert guard.update(56.0).warning is not None  # and back up again


def test_the_stop_fires_before_the_firmware_trips() -> None:
    guard = ThermalGuard(stop_at=65.0)
    verdict = guard.update(65.0, motor=6)
    assert verdict.stop_reason is not None
    assert "motor 7" in verdict.stop_reason, "report the motor NUMBER a human sees"


def test_a_single_failed_read_warns_but_does_not_stop() -> None:
    """A dropped CAN read is a hiccup. Ending a session on one would make the
    guard the thing that most often stops work, and guards people resent get
    disabled."""
    guard = ThermalGuard(blind_cycles=100)
    verdict = guard.update(None)
    assert verdict.stop_reason is None
    assert verdict.warning is not None and "BLIND" in verdict.warning


def test_being_blind_says_so_ONCE_then_stays_quiet() -> None:
    guard = ThermalGuard(blind_cycles=100)
    assert guard.update(None).warning is not None
    assert all(guard.update(None).warning is None for _ in range(50))


def test_persistent_blindness_STOPS_the_session() -> None:
    """⛔⭐ DEFECT 1, stated as a test. Losing the instrument is not the same as a
    good reading. Running on without it is a decision, and it is not one the code
    gets to make silently."""
    guard = ThermalGuard(blind_cycles=100)
    reasons = [guard.update(None).stop_reason for _ in range(100)]
    assert all(r is None for r in reasons[:99]), "stopped too early"
    assert reasons[99] is not None, "never stopped — the guard is blind and carrying on"
    assert "blind" in reasons[99].lower() or "unreadable" in reasons[99].lower()


def test_recovering_from_blindness_is_announced_and_resets_the_count() -> None:
    """Otherwise a session that hiccups 99 times over an hour dies on the hundredth
    for no reason a human could connect to anything."""
    guard = ThermalGuard(blind_cycles=100)
    for _ in range(99):
        guard.update(None)
    verdict = guard.update(40.0)
    assert verdict.warning is not None and "again" in verdict.warning
    assert guard.blind == 0
    assert all(guard.update(None).stop_reason is None for _ in range(99)), (
        "the blind counter did not reset, so unrelated hiccups accumulate")


def test_a_blind_cycle_does_not_corrupt_the_maximum_seen() -> None:
    """The end-of-session summary is the evidence for whether the gripper frame fix
    held. A failed read must not enter that record as a number."""
    guard = ThermalGuard()
    guard.update(48.0, jaw=39.0)
    guard.update(None)
    assert guard.max_seen == 48.0
    assert guard.max_jaw_seen == 39.0


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
