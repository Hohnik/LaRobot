#!/usr/bin/env python3
"""Tests for the per-arm status row. No hardware.

    uv run tests/test_status_row.py

⭐⭐ WHY THIS FILE EXISTS. The heartbeat row is what a human reads to know what an arm is
doing, and until 2026-08-14 it was sixty lines inside `teleop_session.main()`'s control
loop — so **its first execution was always on the arm**, one second into a session, and a
formatting slip in it would present as the session dying for no visible reason.

⛔ The rows it prints are also the place three separate defects were caught by *looking*:
a fabricated `0 °C` from a failed temperature read ([FINDINGS §24.1](../docs/FINDINGS.md)),
33 seconds of an arm sinking in GUIDE with nothing on screen measuring it
([§11](../docs/FINDINGS.md)), and an invisible workspace wall
([§41.1](../docs/FINDINGS.md)). Every one of those is now a line in this row, which makes
the row itself worth proving.

⚠️ These tests pin CONTENT, not layout. Column positions are cosmetic and Julien changes
them; a missing temperature or a silently-swallowed warning is not.
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "apps"))  # ⛔ the app script is not a package; a test OF it imports it as a file
sys.path.insert(0, str(REPO / "scripts"))

from yam.session import ArmSession  # noqa: E402
from teleop_session import status_row  # noqa: E402


class FakeRobot:
    def __init__(self, q=None) -> None:  # noqa: ANN001
        self.q = np.array(q if q is not None else [0.1, -0.2, 0.3, 0.0, 0.0, 0.0, 0.5])

    def get_joint_pos(self):  # noqa: ANN201
        return self.q.copy()

    def command_joint_pos(self, q) -> None:  # noqa: ANN001
        self.q = np.asarray(q, dtype=float).copy()

    def num_dofs(self) -> int:
        return len(self.q)


class FakeTeleop:
    """Only what the row reads: where the tip is, how far the goal leads, the throttle."""

    def __init__(self, ee=(0.3, 0.0, 0.2), lead=(0.0, 0.0), speed_scale=1.0) -> None:  # noqa: ANN001
        self._ee = np.array(ee, dtype=float)
        self._lead = lead
        self.speed_scale = speed_scale
        self.max_lead_m, self.max_lead_rad = 0.05, 0.25
        # The SLOWED message prints the MEASURED pair (item 21): the joint rate the IK
        # asked for against the cap. 7.9 over 1.5 matches the FINDINGS §41.2 episode.
        self.requested_rate, self.max_joint_rate = 7.9, 1.5

    def ee_position(self):  # noqa: ANN201
        return self._ee.copy()

    def lead(self):  # noqa: ANN201
        return self._lead


def arm_in(mode, hottest=44.0, jaw=33.0, **kw):  # noqa: ANN001, ANN003, ANN201
    arm = ArmSession(FakeRobot(), name=kw.pop("name", "B"))
    arm.mode = mode
    arm.hottest, arm.jaw_temp = hottest, jaw
    if mode == "teleop":
        arm.teleop = FakeTeleop(**kw)
        arm.home_ee = np.array([0.3, 0.0, 0.2])
    return arm


def test_the_row_names_its_own_arm_and_mode() -> None:
    """⭐ With two arms the label is the only thing telling the rows apart."""
    row = status_row(arm_in("hold", name="G"), " t=  12.0s", 0.60, 0.0)
    assert row.startswith("[G HOLD"), row
    assert "t=  12.0s" in row


def test_CONTROLS_is_shown_by_its_name_not_by_the_internal_one() -> None:
    """⚠️ The mode is called `map` in the code and CONTROLS everywhere Julien reads. The
    row must use his name — the internal one appears in no document he has."""
    assert "CONTROLS" in status_row(arm_in("map"), "", 0.60, 0.0)


def test_a_failed_temperature_read_says_BLIND_and_never_a_number() -> None:
    """⛔ FINDINGS §24.1: a thrown read became 0 °C, which disarmed the thermal stop and
    printed a calm "hottest 0°C". The row is the only place a human would have seen it."""
    row = status_row(arm_in("hold", hottest=None, jaw=None), "", 0.60, 0.0)
    assert "BLIND" in row and "??" in row
    assert "0°C" not in row, f"a fabricated temperature is in the row: {row}"


def test_the_jaw_temperature_is_shown_separately() -> None:
    """⭐ The shoulder runs hotter than the gripper all session, so `hottest` cannot show
    whether the 2π gripper frame fix held. The jaw number can."""
    row = status_row(arm_in("hold", hottest=44.0, jaw=33.0), "", 0.60, 0.0)
    assert "hottest   44°C" in row and "jaw   33°C" in row


def test_a_six_motor_arm_still_reports_a_row() -> None:
    """⚠️ `--no-gripper` leaves six motors and no jaw temperature. An IndexError here
    would kill the session one second in."""
    arm = ArmSession(FakeRobot(q=[0.0] * 6), name="B")
    arm.mode, arm.hottest, arm.jaw_temp = "hold", 40.0, None
    row = status_row(arm, "", 0.60, 0.0)
    assert "hottest   40°C" in row and "jaw" not in row


def test_GUIDE_reports_the_drift_from_where_it_went_weightless() -> None:
    """⛔ The instrument that was missing on 2026-08-10, when the arm sank to its own stops
    over 33 seconds while the readout said 35 °C (FINDINGS §11)."""
    arm = arm_in("guide")
    arm.guide_ref = np.zeros(7)
    arm.robot.q = np.array([0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0])
    row = status_row(arm, "", 0.60, 0.0)
    assert "drift 0.250 rad" in row, row
    assert "14.3°" in row, f"the degrees reading is missing or wrong: {row}"


def test_TELEOP_shows_how_much_reach_is_left() -> None:
    """⭐ The wall Julien hit on 2026-08-13 was invisible. Now the row carries it."""
    arm = arm_in("teleop", ee=(0.3, 0.0, 0.2))
    row = status_row(arm, "", 0.60, 0.0)
    assert "reach 0.36/0.60m" in row, row
    assert "AT THE EDGE" not in row, "0.36 m of 0.60 is not the edge"


def test_the_edge_warning_fires_at_the_edge() -> None:
    arm = arm_in("teleop", ee=(0.58, 0.0, 0.05))
    assert "AT THE EDGE" in status_row(arm, "", 0.60, 0.0)


def test_the_floor_warning_only_appears_when_the_tip_is_low() -> None:
    """⚠️ Reads as "you are down at the desk", not as an alarm — it is where a pick
    happens."""
    high = status_row(arm_in("teleop", ee=(0.3, 0.0, 0.3)), "", 0.60, 0.0)
    low = status_row(arm_in("teleop", ee=(0.3, 0.0, 0.02)), "", 0.60, 0.0)
    assert "above the floor" not in high
    assert "2cm above the floor" in low, low


def test_a_stuck_solver_is_named_rather_than_left_as_odd_behaviour() -> None:
    """⭐ A goal running far ahead of the achieved pose means a joint limit, a singularity
    or something in the way. It used to present only as the arm behaving strangely."""
    arm = arm_in("teleop", lead=(0.049, 0.0))
    assert "STUCK lead" in status_row(arm, "", 0.60, 0.0)


def test_a_throttled_twist_says_so_with_the_measured_numbers() -> None:
    """⛔ Item 21: the old message asserted "near the reach limit" as the cause while the
    arm stood in a comfortable pose (FINDINGS §41.2). The row now prints what was
    actually measured — the joint rate the IK asked for, against the cap — and must
    never name an unmeasured cause again."""
    arm = arm_in("teleop", speed_scale=0.19)
    row = status_row(arm, "", 0.60, 0.0)
    assert "SLOWED to 19%" in row
    assert "asked for 7.9 rad/s, cap 1.5" in row, "the measured pair is gone"
    assert "reach limit" not in row, "a guessed cause is back in the SLOWED message"


def test_teleop_rows_name_their_control_frame() -> None:
    """⭐ Item 28: `v` aims at one arm, so two arms can be driven in different frames and
    nothing on screen said which was which. TELEOP carries `/w`·`/t`·`/c` in its bracket,
    padded to the same 8 columns as CONTROLS so the rows stay aligned."""
    arm = arm_in("teleop")
    assert "[B TELEOP/w" in status_row(arm, "", 0.60, 0.0)
    arm.frame = "tool"
    assert "[B TELEOP/t" in status_row(arm, "", 0.60, 0.0)


def test_the_session_facts_appear_once_across_several_rows() -> None:
    """⭐ The clock belongs to the session, not to an arm. Two arms with two clocks on
    screen would invite reading them as two different times."""
    b, g = arm_in("hold", name="B"), arm_in("hold", name="G")
    lead = " t=  12.0s  ⏺ REC   5.1s"
    rows = [status_row(b, lead, 0.60, 0.0),
            status_row(g, " " * len(lead), 0.60, 0.0)]
    assert sum("REC" in r for r in rows) == 1, rows
    assert rows[1].startswith("[G HOLD"), rows[1]


def test_the_row_reads_and_never_commands() -> None:
    """⛔ A readout that moves an arm is a readout nobody can trust. The row is called once
    a second from a loop that is holding 4.3 kg."""
    arm = arm_in("teleop")
    before = arm.robot.q.copy()
    status_row(arm, "", 0.60, 0.0)
    assert np.array_equal(arm.robot.q, before), "the status row moved the arm"


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
