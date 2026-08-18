#!/usr/bin/env python3
"""Diff every readable register on both arms against each other.

    uv run scripts/check_arms_match.py            # dry run — prints the plan, sends nothing
    uv run scripts/check_arms_match.py --yes      # transmits register reads only

⭐ WHY THIS EXISTS

On 2026-08-13 a colleague borrowed arm G and ran unknown code against its
motors. DM motors have **writable** registers: sub-command ``0x55`` writes one
and ``0xAA`` saves it to flash, so a change survives a power cycle. Nothing in
this repo ever writes a register, but another tool might, and a changed register
would not raise anything — it would quietly alter what every reading means.
FINDINGS §37.6 asks for exactly this check and names the reason:

    "The two arms should agree on every register except the per-unit `inertia`.
     A difference is the signal."

⭐ WHY IT IS A SCRIPT AND NOT A TABLE IN A DOCUMENT

Six separate measurements written into these documents went stale inside two
days (FINDINGS §33.3, §34.7). A table of live values is a cache with no
invalidation. A script that reads the thing it describes cannot go stale, which
is why ``check_rig.py``, ``check_recordings.py`` and ``check_links.py`` exist.
This is the fourth.

⭐ WHY IT IS AGENT-SAFE

Register reads only: arbitration ID ``0x7FF``, sub-command ``0x33``. That asks
the motor's firmware for a stored value. It cannot enable a motor and it cannot
command motion under any circumstances. Contrast ``ping_motors.py``, which sends
an **enable** frame — that one is still safe, but it changes motor state, and
FINDINGS §37.6 records that it may itself have cleared the latched state it was
sent to measure.

⭐⭐ IT ASKS TWO DIFFERENT QUESTIONS, AND THE SECOND IS THE STRONGER ONE

  1. **Do the two arms agree?** Every register identical except the per-unit
     ones. ⛔ **This has a hole:** a tool that wrote the *same* register on
     *both* arms passes it silently. It was still the right check on
     2026-08-13, because only arm G was lent out.
  2. **Has anything changed since we last looked?** ``--save-baseline`` writes
     every value to ``config/motor_registers.json``; later runs diff against it.
     Here **nothing** may differ, per-unit values included, and that closes the
     hole in question 1. ``config/`` holds measured evidence rather than
     settings (FINDINGS §32.3), which is why the baseline belongs there and is
     committed.

⭐ AND IT RE-DERIVES "PER-UNIT" RATHER THAN TRUSTING THIS FILE FOR IT

Calling a register "per-unit calibration" is a claim, and a wrong one hides the
signal. So the script measures it: for each expected-to-differ register it
compares the spread **within** one arm across motors of the *same model* against
the difference **between** the arms. Per-unit data scatters about equally both
ways. A configured constant is identical within a model, so a between-arm
difference would tower over a within-arm spread of zero. The evidence is printed
every run instead of being cached in prose.

⛔⛔ WHAT THIS CHECK CANNOT SEE, AND IT IS THE RISK §37.6 ACTUALLY RAISED

§37.6 asks for "control mode, and the ``PMAX``/``VMAX``/``TMAX`` scaling limits".
**None of those four is readable through this path**, and the reason matters:

  * The vendored SDK's ``register_addr_map`` (``i2rt/motor_config_tool/utils.py``)
    exposes ten registers and none of them is a scaling limit or a control mode.
    Their register addresses are not published anywhere in this checkout.
  * ⛔ **Guessing an address would be worse than not reading it.** A read of the
    wrong address returns four plausible bytes, and calling them "VMAX" is
    exactly the confident-plausible-wrong failure this repo is built around
    (FINDINGS §0).
  * ⭐ **And the scaling is not read from the motor anyway.** The SDK decodes
    every feedback frame using hardcoded Python constants —
    ``MotorConstants.POSITION_MAX = 12.5``, ``VELOCITY_MAX = 45``,
    ``TORQUE_MAX = 54`` in ``i2rt/motor_drivers/utils.py``. If a motor's *stored*
    limits were changed, the motor would encode its feedback on a different
    scale than the SDK assumes, and every position, velocity and temperature
    reading would be silently wrong by a constant factor. **No register read
    detects that.** What detects it is a decoded value compared against a known
    physical truth — see FINDINGS §38.1 for the one such check already on
    record, and what it does and does not bound.

So a clean result here means "every register the SDK can read is identical on
both arms". It does not mean "nothing was changed".
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))
from yam.can import (  # noqa: E402
    ARM_SERIALS,
    YAM_BITRATE,
    add_i2rt_to_path,
    open_raw_can_interface,
)

JOINT_IDS = [1, 2, 3, 4, 5, 6]
GRIPPER_ID = 0x07
MOTOR_IDS = [*JOINT_IDS, GRIPPER_ID]

REPO = Path(__file__).resolve().parent.parent
BASELINE = REPO / "config" / "motor_registers.json"

# ⚠️ Registers that are EXPECTED to differ between two arms, with the reason.
# Anything not listed here differing is the signal this script exists to find.
# ⛔ These are exempt ONLY from the arm-vs-arm diff. The baseline diff holds them
# to the same standard as everything else, because a per-unit value has no reason
# to move on its own — if one does, something wrote it.
EXPECTED_TO_DIFFER = {
    "inertia": "per-unit calibration data — two motors of the same model report "
    "slightly different values. Treating this as a fault was a real bug on "
    "2026-08-10 (identify_arm.py::signature)",
    "flux": "the permanent-magnet flux linkage, measured per motor. Added 2026-08-14, "
    "the first time this register was ever read here: all 14 motors sit within 2.5% "
    "of each other and no two agree, including motors of the same model on the same "
    "arm, which is the signature of measured data rather than a configured constant",
}

# The register whose value names the motor model. DM43**40** reports 40.0,
# DM43**10** reports 10.0 — the part number *is* the gear ratio.
MODEL_REGISTER = "gear_ratio"


def label(motor_id: int) -> str:
    return "gripper" if motor_id == GRIPPER_ID else f"joint {motor_id}"


def read_arm(arm: str, bitrate: int, registers: list[str]) -> tuple[dict[int, dict], list[str]]:
    """Read every register from every motor on one arm.

    Returns ``(rows, skipped)``. ``skipped`` names registers the first motor
    could not answer: an unsupported register costs 3 attempts x 20 retries per
    motor inside the SDK, so once one motor refuses a register we stop asking
    the rest for it. Without that a single unsupported register turns a ten
    second check into a slow one, and slow checks stop being run.
    """
    add_i2rt_to_path()
    from i2rt.motor_config_tool.utils import get_special_message_response  # noqa: PLC0415

    iface = open_raw_can_interface(bitrate=bitrate, arm=arm)
    rows: dict[int, dict] = {}
    skipped: list[str] = []
    try:
        for motor_id in MOTOR_IDS:
            row: dict = {}
            for reg in registers:
                if reg in skipped:
                    continue
                try:
                    row[reg] = get_special_message_response(iface, motor_id, reg)
                except Exception:  # noqa: BLE001
                    row[reg] = None
                    if motor_id == MOTOR_IDS[0]:
                        skipped.append(reg)
            numeric = [v for v in row.values() if v is not None]
            # Defence in depth against the transmit-echo bug (src/yam/can.py):
            # decoding our own request yields a flawless set of zeros, which is
            # indistinguishable from a successful read unless it is called out.
            if numeric and all(v == 0 for v in numeric):
                print(
                    f"  ⚠️  {arm} {label(motor_id)}: every register read as 0 — that is the "
                    "signature of reading our own transmit echo, NOT a motor reply. "
                    "Treating as no reply."
                )
                continue
            if numeric:
                rows[motor_id] = row
                print(f"  ✓ {arm} {label(motor_id):<9} {len(numeric)} registers read")
            else:
                print(f"  ✗ {arm} {label(motor_id):<9} no reply to any register")
    finally:
        iface.close()
    return rows, skipped


def per_unit_evidence(arms: dict[str, dict[int, dict]]) -> None:
    """Test, rather than assert, that each EXPECTED_TO_DIFFER register is per-unit.

    ⭐ The falsifier: a register holding *measured* per-motor data scatters even
    between two motors of the identical model on the identical arm. A register
    holding a *configured* constant does not — every motor of that model would
    report the same number, so a within-arm spread of exactly zero beside a
    non-zero between-arm difference would mean somebody wrote it.

    This is [FINDINGS §0](../docs/FINDINGS.md) rule 5 applied to the exemption
    list: prefer a test that could falsify the claim over one that agrees with it.
    """
    print("\n=== is each 'expected to differ' register really per-unit measured data? ===")
    print("   (a spread of 0.00% WITHIN one arm beside a non-zero difference BETWEEN them")
    print("    would mean the register is configured, not measured — and so a real signal)")

    names = sorted(arms)
    for reg in EXPECTED_TO_DIFFER:
        # ⛔ `None` means "no two motors of one model were available, so the spread
        # is NOT MEASURABLE" — which is a different statement from "the spread is
        # zero". Using 0.0 for both made a one-motor-per-model read look like proof
        # that the register is configured, and a test caught it (2026-08-14). Every
        # count in this repo that conflated "nothing found" with "nothing looked at"
        # has been a defect (FINDINGS §0).
        within_worst: float | None = None
        within_where = ""
        for arm_name in names:
            rows = arms[arm_name]
            models: dict[float, list[int]] = {}
            for motor_id, row in rows.items():
                models.setdefault(row.get(MODEL_REGISTER), []).append(motor_id)
            for model, ids in models.items():
                vals = [rows[i].get(reg) for i in ids if isinstance(rows[i].get(reg), (int, float))]
                if len(vals) < 2 or not any(vals):
                    continue
                spread = (max(vals) - min(vals)) / (sum(vals) / len(vals)) * 100
                if within_worst is None or spread > within_worst:
                    within_worst = spread
                    within_where = f"{arm_name} model {model} over joints {sorted(ids)}"

        between_worst = 0.0
        between_where = ""
        if len(names) == 2:
            first, second = arms[names[0]], arms[names[1]]
            for motor_id in MOTOR_IDS:
                va = first.get(motor_id, {}).get(reg)
                vb = second.get(motor_id, {}).get(reg)
                if not isinstance(va, (int, float)) or not isinstance(vb, (int, float)):
                    continue
                mean = (va + vb) / 2
                if mean == 0:
                    continue
                diff = abs(va - vb) / mean * 100
                if diff > between_worst:
                    between_worst = diff
                    between_where = label(motor_id)

        shown = "not measurable" if within_worst is None else f"{within_worst:5.2f}%"
        print(f"\n  {reg}:")
        print(f"    widest spread WITHIN one arm, same model : {shown}   ({within_where})")
        print(f"    widest difference BETWEEN the two arms   : {between_worst:5.2f}%   ({between_where})")
        if within_worst is None:
            print("    ⚠️ no two motors of one model replied, so there is no within-arm spread to")
            print("       compare against. This run cannot say whether the register is measured or")
            print("       configured — it is not evidence either way.")
        elif within_worst == 0.0 and between_worst > 0.0:
            print("    ⛔ ZERO within, non-zero between → this register looks CONFIGURED, not")
            print("       measured. Remove it from EXPECTED_TO_DIFFER and treat it as the signal.")
        elif within_worst > 0.0:
            print("    ✓ it scatters within one arm too, so it is measured per-motor data and its")
            print("      differing between arms carries no information about anybody's code.")


def compare_to_baseline(arms: dict[str, dict[int, dict]], baseline: dict) -> int:
    """Diff today's read against the stored baseline. Nothing at all may differ.

    ⭐ This is the check that survives a tool having written the *same* register on
    *both* arms, which an arm-vs-arm diff cannot see. Per-unit values are held to
    the same standard here: a measured constant burned into a motor has no reason
    to move between two readings, so if one does, something wrote it.
    """
    changed = 0
    stored = baseline.get("arms", {})
    print(f"\n=== today vs the baseline of {baseline.get('read_at', '?')} "
          f"(commit {baseline.get('commit', '?')}) ===")
    for arm_name in sorted(arms):
        if arm_name not in stored:
            print(f"  ·  {arm_name}: not in the baseline, so there is nothing to compare. "
                  "Re-run with --save-baseline to include it.")
            continue
        for motor_id in MOTOR_IDS:
            now = arms[arm_name].get(motor_id, {})
            was = stored[arm_name].get(str(motor_id), {})
            for reg, value in now.items():
                if reg not in was:
                    continue
                if was[reg] != value:
                    changed += 1
                    print(f"  ⛔ {arm_name} {label(motor_id):<9} {reg:<12} "
                          f"was {was[reg]!r}  now {value!r}  ← CHANGED")
    if changed == 0:
        print("  ✓ every register on every motor reads exactly what it read then.")
    return changed


def save_baseline(arms: dict[str, dict[int, dict]], registers: list[str], skipped: dict) -> None:
    """Write the full read to ``config/motor_registers.json``, with provenance."""
    from yam.provenance import dt_now, git_commit  # noqa: PLC0415

    payload = {
        "read_at": dt_now(),
        "commit": git_commit(),
        "registers": registers,
        "skipped": skipped,
        "note": "Baseline for scripts/check_arms_match.py. Every value is a register READ "
                "from a motor; nothing here is a setting and nothing here should be edited "
                "by hand. If a value moves between two runs, something wrote it. "
                "See docs/FINDINGS.md §38.",
        "arms": {a: {str(m): row for m, row in rows.items()} for a, rows in arms.items()},
    }
    BASELINE.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")
    print(f"\n✓ baseline written to {BASELINE.relative_to(REPO)} "
          f"({sum(len(r) for r in arms.values())} motors). Commit it — config/ is measured evidence.")


def compare(a_name: str, a: dict[int, dict], b_name: str, b: dict[int, dict]) -> int:
    """Print every register that differs. Returns the number of unexpected ones."""
    unexpected = 0
    expected = 0

    missing_a = [m for m in MOTOR_IDS if m not in a]
    missing_b = [m for m in MOTOR_IDS if m not in b]
    if missing_a or missing_b:
        print(f"\n⛔ Not every motor replied — {a_name} missing {missing_a}, {b_name} missing {missing_b}.")
        print("   A diff over a partial read cannot support 'the arms agree'. Fix the read first.")
        return -1

    print(f"\n=== {a_name} vs {b_name}, register by register ===")
    for motor_id in MOTOR_IDS:
        for reg in a[motor_id]:
            va, vb = a[motor_id].get(reg), b[motor_id].get(reg)
            if va == vb:
                continue
            if reg in EXPECTED_TO_DIFFER:
                expected += 1
                print(f"  ·  {label(motor_id):<9} {reg:<12} {va!r:>26}  vs {vb!r:<26} (expected)")
            else:
                unexpected += 1
                print(f"  ⛔ {label(motor_id):<9} {reg:<12} {va!r:>26}  vs {vb!r:<26}  ← DIFFERS")

    if expected:
        for reg, why in EXPECTED_TO_DIFFER.items():
            print(f"\n  · '{reg}' differing is expected: {why}.")
    return unexpected


def main() -> int:
    ap = argparse.ArgumentParser(
        description="Diff every SDK-readable register on both arms. Register reads only; cannot energise."
    )
    ap.add_argument("--yes", action="store_true", help="actually transmit register reads (default: dry run)")
    ap.add_argument("--bitrate", type=int, default=YAM_BITRATE)
    ap.add_argument(
        "--arms",
        nargs=2,
        default=["B", "G"],
        metavar=("A", "B"),
        help="the two arms to compare, by label. Selected by adapter serial, never by index.",
    )
    ap.add_argument(
        "--save-baseline",
        action="store_true",
        help="write this read to config/motor_registers.json as the reference for later runs. "
        "⚠️ Only do this when the motors are believed good: it overwrites the reference a "
        "future change would be measured against.",
    )
    args = ap.parse_args()

    a_name, b_name = args.arms
    for name in (a_name, b_name):
        if name not in ARM_SERIALS:
            print(f"⛔ unknown arm '{name}'. Known: {sorted(ARM_SERIALS)}")
            return 2

    add_i2rt_to_path()
    from i2rt.motor_config_tool.utils import register_addr_map  # noqa: PLC0415

    # Read the register list FROM the SDK rather than copying it here, so this
    # script cannot fall behind the vendored driver the way a written list would.
    registers = list(register_addr_map)

    print(f"arms to compare    : {a_name} vs {b_name}  (by adapter serial)")
    print(f"motors per arm     : {MOTOR_IDS}")
    print(f"registers          : {len(registers)} — {', '.join(registers)}")
    print("frames sent        : 0x7FF sub-command 0x33 (READ). No enable, no write, no save.")
    print("⛔ scaling limits and control mode are NOT in this list — see this script's docstring.\n")

    if not args.yes:
        print("DRY RUN — nothing transmitted. Re-run with --yes.")
        return 0

    print(f"--- reading {a_name} ---")
    a_rows, a_skipped = read_arm(a_name, args.bitrate, registers)
    print(f"--- reading {b_name} ---")
    b_rows, b_skipped = read_arm(b_name, args.bitrate, registers)

    for name, skipped in ((a_name, a_skipped), (b_name, b_skipped)):
        if skipped:
            print(f"\n⚠️  {name}: these registers are in the SDK's map but this motor family does "
                  f"not answer them, so they were skipped: {skipped}")

    if a_skipped != b_skipped:
        print(f"\n⛔ The two arms skipped DIFFERENT registers ({a_skipped} vs {b_skipped}). "
              "That is itself a difference between the arms — investigate before reading the diff below.")

    arms = {a_name: a_rows, b_name: b_rows}

    unexpected = compare(a_name, a_rows, b_name, b_rows)
    if unexpected < 0:
        return 1

    per_unit_evidence(arms)

    # ⛔ `baseline_checked` exists because the first version of this verdict printed
    # "nothing has moved since the baseline" on a run where there WAS no baseline and
    # nothing had been compared. `changed == 0` is true both when nothing moved and
    # when nothing was looked at, and reporting those as the same thing is the exact
    # defect class this repo is built around (FINDINGS §0): confident, plausible,
    # wrong, and no exception raised. Found by reading the script's own output.
    changed = 0
    baseline_checked = False
    if BASELINE.exists():
        changed = compare_to_baseline(arms, json.loads(BASELINE.read_text()))
        baseline_checked = True
    else:
        print(f"\n⚠️  No baseline at {BASELINE.relative_to(REPO)}, so the stronger question — "
              "'has anything changed since we last looked?' — was NOT asked on this run.")
        print("   Re-run with --save-baseline to create one; the run after that can ask it.")

    if args.save_baseline:
        save_baseline(arms, registers, {a_name: a_skipped, b_name: b_skipped})

    print()
    if unexpected == 0 and changed == 0:
        print(f"✓ VERDICT: every readable register is identical on {a_name} and {b_name}, apart from "
              "the per-unit values shown above.")
        if baseline_checked:
            print("  ✓ And nothing has moved since the baseline.")
        else:
            print("  ⚠️ Nothing was compared against a baseline on this run, so this says nothing")
            print("     about whether a value changed — only that the two arms agree today.")
        print("  ⛔ It does NOT mean nothing was changed. Scaling limits and control mode are not")
        print("     readable here, and the SDK decodes feedback with its own hardcoded constants.")
        print("     See this script's docstring and docs/FINDINGS.md §38.")
        return 0

    if changed:
        print(f"⛔ VERDICT: {changed} register(s) have MOVED since the baseline. Something wrote them.")
    if unexpected:
        print(f"⛔ VERDICT: {unexpected} register(s) differ between the arms that should not.")
    print("  Do NOT run a control loop until it is understood. A changed register does not raise;")
    print("  it silently alters what every reading means (docs/FINDINGS.md §37.6).")
    return 1


if __name__ == "__main__":
    sys.exit(main())
