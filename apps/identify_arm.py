#!/usr/bin/env python3
"""Ask the arm what it is, without energising a single motor.

    uv run apps/identify_arm.py            # dry run — prints the plan, sends nothing
    uv run apps/identify_arm.py --yes      # transmits register reads only

⭐ WHY THIS IS THE RIGHT FIRST CONTACT

I2RT's own `ping_motors.py` answers "is this motor alive?" by calling
`motor_on()`, which sends the enable frame `…FC`. That energises a motor on a
physical arm. This script never does that. It uses the register-read path
instead — arbitration ID `0x7FF`, sub-command `0x33` — which asks the motor's
firmware for a stored value and cannot command motion under any circumstances.
(Contrast: `0x55` writes a register, `0xAA` saves it to memory. Neither is used
here, and neither is wrapped in `src/yam/can.py`.)

So this transmits, but nothing it transmits can move the arm.

⭐ HOW IT IDENTIFIES THE ARM

Deliberately **without relying on absolute values**, which would mean hardcoding
motor specs I cannot verify. Instead it reads `gear_ratio` and `KT_value` from
every motor and groups joints that report identical values. The *pattern* is the
discriminator, and it comes straight from I2RT's own configs:

    joints 1-3 alike, 4-6 alike  ->  yam / yam_pro / yam_ultra_v1
    joints 1-4 alike, 5-6 alike  ->  yam_ultra_v2   (DM4340 on joint 4)

The gripper is motor 0x07. Joints 5 and 6 are DM4310 on every variant, so the
gripper matching them means a 4310-family gripper (linear/crank/flexible_4310);
differing means DM3507, i.e. linear_3507.

⚠️ What this CANNOT settle: `yam`, `yam_pro` and `yam_ultra_v1` share an
identical motor layout, so they are indistinguishable over CAN. Separating those
needs the physical label or a mass measurement (4.292 / 4.349 / 4.521 kg).
"""

from __future__ import annotations

import argparse
import struct
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))
from yam.can import (  # noqa: E402
    ARM_SERIALS,
    DEFAULT_ARM,
    YAM_BITRATE,
    add_i2rt_to_path,
    open_raw_can_interface,
)

ARM_IDS = [1, 2, 3, 4, 5, 6]
GRIPPER_ID = 0x07

# Read-only registers. Deliberately no writes.
REGISTERS = ["id", "master_id", "gear_ratio", "KT_value", "sw_ver", "timeout", "inertia"]


def signature(row: dict) -> tuple:
    """The fields that identify a motor *model*.

    ⚠️ `inertia` must NOT be in here, and that was a real bug on 2026-08-10: it
    is per-unit calibration data, so two motors of the same model report
    slightly different values (1.7169e-05 vs 1.7414e-05 on joints 1 and 3) and
    every joint looked like its own model.

    `gear_ratio` is the discriminator and it is beautifully direct — the Damiao
    part number *is* the gear ratio: DM43**40** reports 40.0, DM43**10** reports
    10.0. `sw_ver` partitions identically and is kept as a cross-check.
    `KT_value` reads 0.0 on every motor here, so it carries no information.
    """
    return (row.get("gear_ratio"), row.get("sw_ver"))


MODEL_BY_GEAR_RATIO = {40.0: "DM4340", 10.0: "DM4310"}


def model_of(row: dict) -> str:
    return MODEL_BY_GEAR_RATIO.get(row.get("gear_ratio"), f"unknown(gear_ratio={row.get('gear_ratio')})")


def classify(sigs: dict[int, tuple]) -> str:
    present = [i for i in ARM_IDS if i in sigs]
    if len(present) < 6:
        return f"cannot classify — only joints {present} replied"

    group_a = sigs[1]
    alike = [i for i in ARM_IDS if sigs[i] == group_a]

    if alike == [1, 2, 3]:
        return "yam / yam_pro / yam_ultra_v1  (joint 4 differs from 1-3 → DM4310 there)"
    if alike == [1, 2, 3, 4]:
        return "yam_ultra_v2  (joint 4 matches 1-3 → DM4340 there)"
    return f"unexpected layout — joints matching joint 1: {alike}"


REG_ID = 8          # the "id" register — every DM motor has one
REG_GEAR_RATIO = 20


def scan(low: int, high: int, bitrate: int, confirmed: bool, arm: str = DEFAULT_ARM) -> int:
    """Sweep CAN IDs for anything that answers a register read.

    Bypasses I2RT's retry wrapper deliberately: a non-existent motor costs 20
    retries × 3 attempts there, which makes a 32-ID sweep take minutes. One
    request and a short timeout per ID is all a presence check needs.

    Still register reads only — this cannot enable or move anything.
    """
    print(f"scanning CAN motor IDs {low}..{high} — one register read each, no retries")
    print("traffic: 0x7FF sub-command 0x33 (READ). No enable, no setpoint.\n")
    if not confirmed:
        print("DRY RUN — nothing transmitted. Re-run with --yes.")
        return 0

    import can  # noqa: PLC0415

    iface = open_raw_can_interface(bitrate=bitrate, arm=arm)
    bus = iface.bus
    found: list[tuple[int, float]] = []
    try:
        for motor_id in range(low, high + 1):
            for reg in (REG_ID, REG_GEAR_RATIO):
                bus.send(
                    can.Message(
                        arbitration_id=0x7FF,
                        data=[motor_id, 0x00, 0x33, reg, 0x00, 0x00, 0x00, 0x00],
                        is_extended_id=False,
                    )
                )
                reply = bus.recv(timeout=0.02)
                if reply is not None and reg == REG_GEAR_RATIO:
                    ratio = struct.unpack("<f", bytes(reply.data)[4:8])[0]
                    found.append((motor_id, ratio))
                    print(f"  ✓ id {motor_id:>3} responded   gear_ratio={ratio}")
                    break
                if reply is None:
                    break
    finally:
        iface.close()

    print(f"\n{len(found)} device(s) on the bus: {[m for m, _ in found]}")
    print("No motor was enabled; nothing was commanded.")
    if len(found) <= 7:
        print(
            "\n→ One arm's worth of motors on this bus. Confirmed 2026-08-10: each YAM has its OWN\n"
            "  CANable and its own bus, and BOTH arms use the identical motor IDs 1-7. Nothing in a\n"
            "  CAN frame distinguishes the arms — only the adapter serial does. That is why every\n"
            "  script here selects by serial and never by index (src/yam/can.py)."
        )
    else:
        print("\n→ More than 7 devices: something else shares this bus. Investigate before commanding.")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description="Identify the YAM variant over CAN, without enabling motors.")
    ap.add_argument("--yes", action="store_true", help="actually transmit register reads (default: dry run)")
    ap.add_argument("--ids", type=int, nargs="+", default=[*ARM_IDS, GRIPPER_ID])
    ap.add_argument("--bitrate", type=int, default=YAM_BITRATE)
    ap.add_argument("--arm", default=DEFAULT_ARM, choices=sorted(ARM_SERIALS), help="WHICH ARM. Selected by serial, never by index.")
    ap.add_argument(
        "--scan",
        nargs=2,
        type=int,
        metavar=("LOW", "HIGH"),
        help="sweep a CAN ID range for ANY responding motor and exit. Answers 'what else is on "
        "this bus' — e.g. whether the second arm shares it. One read per ID, no retries.",
    )
    args = ap.parse_args()

    if args.scan:
        return scan(args.scan[0], args.scan[1], args.bitrate, args.yes, args.arm)

    print(f"motor IDs to query : {args.ids}")
    print(f"registers          : {', '.join(REGISTERS)}")
    print("frames sent        : 0x7FF sub-command 0x33 (READ). No enable, no write, no save.\n")

    if not args.yes:
        print("DRY RUN — nothing transmitted. Re-run with --yes.")
        return 0

    add_i2rt_to_path()
    from i2rt.motor_config_tool.utils import get_special_message_response  # noqa: PLC0415

    iface = open_raw_can_interface(bitrate=args.bitrate, arm=args.arm)
    print("bus open (normal mode — adapter is an active CAN node, but sends only reads)\n")

    rows: dict[int, dict] = {}
    try:
        for motor_id in args.ids:
            row: dict = {}
            for reg in REGISTERS:
                try:
                    row[reg] = get_special_message_response(iface, motor_id, reg)
                except Exception as exc:  # noqa: BLE001
                    row[reg] = f"<{type(exc).__name__}>"
            replied = any(not isinstance(v, str) for v in row.values())
            # Defence in depth against the transmit-echo bug (see yam_can.py):
            # decoding our own request yields a flawless set of zeros, which is
            # indistinguishable from success unless it is called out explicitly.
            numeric = [v for v in row.values() if not isinstance(v, str)]
            if replied and numeric and all(v == 0 for v in numeric):
                print(
                    f"  ⚠️  motor {motor_id}: every register read as 0 — that is the signature of "
                    "reading our own transmit echo, NOT a motor reply. Treating as no reply."
                )
                replied = False
            if replied:
                rows[motor_id] = row
                label = "gripper" if motor_id == GRIPPER_ID else f"joint {motor_id}"
                print(f"  ✓ {label:<9} " + "  ".join(f"{k}={row[k]}" for k in REGISTERS))
            else:
                print(f"  ✗ motor {motor_id}: no reply to any register")
    finally:
        iface.close()
        print("\nbus closed. No motor was ever enabled.")

    if not rows:
        print(
            "\nNothing replied at all. In order of likelihood: the arm is not powered,\n"
            "CAN wiring/termination, the bitrate is not 1 Mbit/s, or different motor IDs."
        )
        return 1

    sigs = {mid: signature(row) for mid, row in rows.items()}
    print("\n=== motor models (from gear_ratio; sw_ver cross-checks the split) ===")
    for mid in sorted(sigs):
        label = "gripper" if mid == GRIPPER_ID else f"joint {mid}"
        print(f"  {label:<9} {model_of(rows[mid]):<8} gear_ratio={sigs[mid][0]}  sw_ver={sigs[mid][1]}")

    print(f"\n⭐ ARM VARIANT: {classify(sigs)}")

    timeouts = {rows[m].get("timeout") for m in rows}
    if timeouts == {8000}:
        print("\n✓ SAFETY TIMEOUT: register reads 8000 on every motor — enabled, factory default.")
        print(
            "  8000 is the raw register value I2RT's own set_timeout.py writes to turn the timeout\n"
            "  ON (it writes 0 to turn it OFF), and their README documents the resulting behaviour as\n"
            "  400 ms — so the unit is 50 µs, not ms. A motor with no command for 400 ms enters\n"
            "  damping mode by itself. This is the state you want; do not 'fix' it."
        )
    else:
        print(f"\n⛔ SAFETY TIMEOUT reads {timeouts}, which is NOT the expected 8000.")
        print(
            "  0 means the timeout is DISABLED — I2RT warn that a failed gravity-compensation loop\n"
            "  can then produce uncontrolled torque. Do not run any control loop until this is\n"
            "  understood. Re-enable with i2rt/motor_config_tool/set_timeout.py --timeout"
        )

    if GRIPPER_ID in sigs:
        ref = [sigs[i] for i in (5, 6) if i in sigs]
        if ref and sigs[GRIPPER_ID] in ref:
            print("⭐ GRIPPER: matches joints 5/6 → DM4310 family (linear_4310 / crank_4310 / flexible_4310)")
        elif ref:
            print("⭐ GRIPPER: differs from joints 5/6 → DM3507 → linear_3507")
        else:
            print("⭐ GRIPPER: replied, but joints 5/6 did not, so there is nothing to compare against")
    else:
        print("⭐ GRIPPER: motor 0x07 did not reply — either no gripper, or a different ID")

    return 0


if __name__ == "__main__":
    sys.exit(main())
