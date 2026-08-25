#!/usr/bin/env python3
"""Tests for `check_arms_match.py`'s pure logic. No hardware, no CAN bus.

    uv run tests/test_arms_match.py

⛔ WHY THIS FILE EXISTS. The script it tests is a **safety check**: it answers
"did a third party change something in these motors?" and a wrong answer here is
worse than no answer, because a clean verdict is what clears a control loop to
run (FINDINGS §37.6).

⭐ The test that matters most is `test_a_configured_register_is_caught_by_the_falsifier`.
The script exempts `inertia` and `flux` from the arm-vs-arm diff on the grounds
that they hold measured per-motor data. **That exemption is where a real signal
would hide**, so the script re-derives the claim every run instead of trusting
it, and this test proves the re-derivation can actually fail.

⚠️ The first version of the script's verdict printed "nothing has moved since the
baseline" on a run where no baseline existed and nothing had been compared.
`test_no_baseline_is_not_reported_as_unchanged` pins that, because
`changed == 0` is true both when nothing moved and when nothing was looked at.
"""

from __future__ import annotations

import io
import json
import sys
from contextlib import redirect_stdout
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "apps"))    # ⛔ app and check scripts are files, not packages;
sys.path.insert(0, str(REPO / "checks"))  # a test OF one imports it from its directory
sys.path.insert(0, str(REPO / "scripts"))

from check_arms_match import (  # noqa: E402
    EXPECTED_TO_DIFFER,
    GRIPPER_ID,
    MOTOR_IDS,
    compare,
    compare_to_baseline,
    per_unit_evidence,
)

# Two DM4340 (gear_ratio 40) joints and two DM4310 (gear_ratio 10) joints is
# enough shape to exercise the model grouping: the falsifier needs at least two
# motors of the SAME model on the SAME arm to have a within-arm spread at all.
DM4340, DM4310 = 40.0, 10.0


def arm(
    flux: dict[int, float],
    inertia: dict[int, float] | None = None,
    overrides: dict[int, dict] | None = None,
) -> dict[int, dict]:
    """One arm's rows, with sane values everywhere the test does not care.

    ⚠️ `overrides` is a positional dict rather than `**kwargs` because its keys are
    motor IDs, and Python keyword names must be strings.
    """
    overrides = overrides or {}
    rows: dict[int, dict] = {}
    for motor_id in MOTOR_IDS:
        rows[motor_id] = {
            "id": motor_id,
            "master_id": motor_id + 0x10,
            "gear_ratio": DM4340 if motor_id <= 3 else DM4310,
            "sw_ver": 303,
            "timeout": 8000,
            "KT_value": 0.0,
            "gear_eff": 0.9,
            "OT_value": 80.0,
            "inertia": (inertia or {}).get(motor_id, 1.7e-05 + motor_id * 1e-08),
            "flux": flux.get(motor_id, 0.0044),
        }
        rows[motor_id].update(overrides.get(motor_id, {}))
    return rows


def captured(fn, *args) -> str:
    buf = io.StringIO()
    with redirect_stdout(buf):
        fn(*args)
    return buf.getvalue()


# ── the arm-vs-arm diff ──────────────────────────────────────────────────────


def test_two_identical_arms_report_no_unexpected_difference() -> None:
    flux = {m: 0.0044 + m * 1e-06 for m in MOTOR_IDS}
    a, b = arm(flux), arm(flux)
    assert compare("B", a, "G", b) == 0


def test_a_changed_timeout_is_the_signal() -> None:
    """⛔ `timeout` 0 means the safety timeout is DISABLED, which is the one register
    whose change I2RT warn can produce uncontrolled torque. It must never be silent."""
    flux = {m: 0.0044 + m * 1e-06 for m in MOTOR_IDS}
    a, b = arm(flux), arm(flux, overrides={4: {"timeout": 0}})
    out = captured(compare, "B", a, "G", b)
    assert compare("B", a, "G", b) == 1
    assert "timeout" in out and "DIFFERS" in out


def test_per_unit_registers_differing_is_not_reported_as_a_fault() -> None:
    a = arm({m: 0.00440 for m in MOTOR_IDS}, inertia={m: 1.70e-05 for m in MOTOR_IDS})
    b = arm({m: 0.00445 for m in MOTOR_IDS}, inertia={m: 1.72e-05 for m in MOTOR_IDS})
    out = captured(compare, "B", a, "G", b)
    assert compare("B", a, "G", b) == 0
    assert "expected" in out
    assert "DIFFERS" not in out


def test_a_partial_read_refuses_rather_than_diffing() -> None:
    """⛔ A diff over a partial read would report 'the arms agree' about motors it
    never asked. Returning -1 is the refusal; FINDINGS §0 rule 5."""
    flux = {m: 0.0044 for m in MOTOR_IDS}
    a, b = arm(flux), arm(flux)
    del b[GRIPPER_ID]
    assert compare("B", a, "G", b) == -1


def test_every_expected_to_differ_register_explains_itself() -> None:
    """A bare exemption list months later is unreadable, and this one hides signal."""
    for reg, why in EXPECTED_TO_DIFFER.items():
        assert len(why) > 40, f"{reg} needs a real reason, not '{why}'"


# ── the falsifier: is a register really per-unit? ─────────────────────────────


def test_a_configured_register_is_caught_by_the_falsifier() -> None:
    """⭐⭐ THE TEST THIS FILE EXISTS FOR.

    A register that is CONFIGURED reads identically on every motor of a model,
    so its within-arm spread is exactly zero. If it then differs between the
    arms, somebody wrote it. The exemption must not survive that.
    """
    a = arm({m: 0.00440 for m in MOTOR_IDS})   # identical within each model
    b = arm({m: 0.00500 for m in MOTOR_IDS})   # identical too, but different arm
    out = captured(per_unit_evidence, {"B": a, "G": b})
    assert "0.00%" in out, out
    assert "looks CONFIGURED" in out, out


def test_measured_scatter_passes_the_falsifier() -> None:
    """The real 2026-08-14 shape: it scatters within one arm as well as between."""
    a = arm({1: 0.004448, 2: 0.004377, 3: 0.004438, 4: 0.004406,
             5: 0.004364, 6: 0.004412, 7: 0.004475})
    b = arm({1: 0.004453, 2: 0.004456, 3: 0.004446, 4: 0.004425,
             5: 0.004428, 6: 0.004449, 7: 0.004438})
    out = captured(per_unit_evidence, {"B": a, "G": b})
    assert "looks CONFIGURED" not in out, out
    assert "measured per-motor data" in out


def test_the_falsifier_needs_two_motors_of_one_model_and_says_nothing_otherwise() -> None:
    """⚠️ With one motor per model there is no within-arm spread to measure, and
    reporting 0.00% would then be a claim the data cannot support."""
    a = {1: arm({})[1]}
    b = {1: arm({})[1] | {"flux": 0.005}}
    out = captured(per_unit_evidence, {"B": a, "G": b})
    assert "looks CONFIGURED" not in out, out


# ── the baseline diff ────────────────────────────────────────────────────────


def as_baseline(arms: dict[str, dict[int, dict]]) -> dict:
    return {
        "read_at": "2026-08-14T09:10:39+02:00",
        "commit": "2f70f6e",
        "arms": {a: {str(m): row for m, row in rows.items()} for a, rows in arms.items()},
    }


def test_an_unchanged_read_matches_its_baseline() -> None:
    flux = {m: 0.0044 + m * 1e-06 for m in MOTOR_IDS}
    arms = {"B": arm(flux), "G": arm(flux)}
    assert compare_to_baseline(arms, as_baseline(arms)) == 0


def test_a_moved_register_is_caught_even_when_BOTH_arms_moved_together() -> None:
    """⭐⭐ This is the hole the arm-vs-arm diff cannot see. A tool that wrote the
    same register on both arms leaves them agreeing with each other perfectly."""
    flux = {m: 0.0044 + m * 1e-06 for m in MOTOR_IDS}
    before = {"B": arm(flux), "G": arm(flux)}
    baseline = as_baseline(before)

    after = {"B": arm(flux, overrides={5: {"timeout": 0}}),
             "G": arm(flux, overrides={5: {"timeout": 0}})}
    assert compare("B", after["B"], "G", after["G"]) == 0, "the arms still agree — that is the hole"
    assert compare_to_baseline(after, baseline) == 2, "and the baseline must catch it on both arms"


def test_a_per_unit_register_moving_IS_a_fault_against_the_baseline() -> None:
    """⛔ `inertia` and `flux` are exempt from the arm-vs-arm diff only. A measured
    constant burned into a motor has no reason to move between two readings."""
    flux = {m: 0.0044 for m in MOTOR_IDS}
    before = {"B": arm(flux)}
    after = {"B": arm({**flux, 3: 0.0051})}
    assert compare_to_baseline(after, as_baseline(before)) == 1


def test_an_arm_absent_from_the_baseline_is_skipped_not_silently_passed() -> None:
    flux = {m: 0.0044 for m in MOTOR_IDS}
    baseline = as_baseline({"B": arm(flux)})
    out = captured(compare_to_baseline, {"B": arm(flux), "G": arm(flux)}, baseline)
    assert "not in the baseline" in out
    assert compare_to_baseline({"B": arm(flux), "G": arm(flux)}, baseline) == 0


def test_no_baseline_is_not_reported_as_unchanged() -> None:
    """⚠️ The defect found by reading the script's own output on 2026-08-14: with no
    baseline the count of moved registers is 0, which is the same number as 'nothing
    moved'. The script must therefore carry a separate 'did we look?' flag — this
    test pins that the two states are distinguishable at all."""
    source = (REPO / "checks" / "check_arms_match.py").read_text()
    assert "baseline_checked" in source, "the did-we-look flag is gone"
    assert "was NOT asked on this run" in source, "the honest no-baseline wording is gone"


def test_the_baseline_on_disk_is_readable_and_carries_provenance() -> None:
    """The baseline is measured evidence in config/, so it must be committed and sane."""
    path = REPO / "config" / "motor_registers.json"
    if not path.exists():
        return  # a fresh clone has no baseline until someone runs the read
    data = json.loads(path.read_text())
    assert data["commit"] and data["read_at"], "a baseline with no provenance cannot be trusted"
    assert set(data["arms"]) == {"B", "G"}, data["arms"].keys()
    for arm_name, rows in data["arms"].items():
        assert len(rows) == 7, f"{arm_name} has {len(rows)} motors, expected 7"
        for motor_id, row in rows.items():
            assert row["timeout"] == 8000, f"{arm_name} motor {motor_id} timeout is {row['timeout']}, not 8000"


def main() -> int:
    tests = [(n, f) for n, f in sorted(globals().items()) if n.startswith("test_") and callable(f)]
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
