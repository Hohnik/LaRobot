#!/usr/bin/env python3
"""Ask each YAM motor whether it is there. FIRST SCRIPT THAT EVER TRANSMITS.

    uv run scripts/ping_motors.py            # dry run — prints the plan, sends nothing
    uv run scripts/ping_motors.py --yes      # actually transmits

⛔ READ BEFORE RUNNING WITH --yes

This is the macOS port of I2RT's own `i2rt/motor_config_tool/ping_motors.py`.
Theirs opens SocketCAN; this one goes through the CANable via gs_usb (see
`src/yam_can.py`). The CAN traffic is byte-for-byte what I2RT's tool sends.

Per motor ID it sends exactly two frames:

    enable   0xFF FF FF FF FF FF FF FC     then reads the reply
    disable  0xFF FF FF FF FF FF FF FD

The reply to the enable frame *is* the motor's state — position, velocity,
torque, voltage, temperatures, error code. That reply is the whole point: it is
the only way to learn whether a motor is alive, because DM motors never speak
unless spoken to.

Why enabling should not move the arm: the motors run in MIT mode, and enabling
does not by itself command a torque. No position, velocity or torque setpoint is
ever sent by this script, and each motor is disabled again immediately. I2RT's
factory-default 400 ms command timeout is an additional net — a motor that stops
receiving commands disables itself.

That said, this energises motors on a physical arm. It is not a read-only
operation and it is not risk-free:
  · Make sure the arm is clear of people, cables and anything it could strike.
  · Have the power switch reachable.
  · A motor holding a stale setpoint from a previous session could twitch.

⛔⭐⭐ THE SHARP EDGE, AND UNTIL 2026-08-14 THIS PARAGRAPH WAS A LIE

`motor_on()` loops `clean_error()` until a motor's error code reads normal, so
pinging a faulted motor **clears the fault**. It does that with the root log level
forced to ERROR, and the two messages that name the fault are logged at WARNING
and INFO, so **the diagnosis is suppressed and the final reading looks healthy.**

⛔ This file used to claim that `--attempt-error-clear` was "default off" and
"controls whether we let it try at all". **The flag was parsed, documented, shown
in `--help`, and read by nothing.** The clear loop ran on every `--yes`. That is
why arm G's red flashing lights stopped after the 18:00 ping on 2026-08-13, and
why the fault type was never learned. FINDINGS §39.

✅ **The flag now works.** By default a latched fault is **reported and left
alone**, naming the code and what the LED looks like. Pass
`--attempt-error-clear` to restore the vendor behaviour and clear it.

⭐ What the lights mean (DAMIAO DM-J4340-2EC manual, "Indicator status"):
green steady = enabled; **red steady = disabled, which is normal for a powered
arm nobody is commanding**; **red flashing = a latched fault.**
"""

from __future__ import annotations

import argparse
import sys
import time
from contextlib import nullcontext
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))
from yam_can import (  # noqa: E402
    ARM_SERIALS,
    DEFAULT_ARM,
    YAM_BITRATE,
    YAM_MOTOR_TYPES,
    MotorFaultNotCleared,
    add_i2rt_to_path,
    describe_motor_error,
    do_not_clear_motor_faults,
    open_motor_interface,
)

DEFAULT_IDS = [1, 2, 3, 4, 5, 6, 7]  # 6 arm joints + gripper


def describe(info: object) -> str:
    """Format a FeedbackFrameInfo.

    ⚠️ Note the type. `motor_on()` returns a **FeedbackFrameInfo**
    (position/velocity/torque/temperature_mos/temperature_rotor), NOT the
    similarly-named `MotorInfo` (pos/vel/eff/voltage/temp_mos/temp_rotor) that
    the module also defines. Reading the wrong field names does not raise — it
    silently yields "?" for every value, which is what happened on 2026-08-10.
    """

    def g(attr: str, fmt: str = "{:.4f}") -> str:
        val = getattr(info, attr, None)
        if val is None:
            return "?"
        try:
            return fmt.format(val)
        except (TypeError, ValueError):
            return str(val)

    err = getattr(info, "error_code", "?")
    err_txt = f"{err} ({getattr(info, 'error_message', '?')})"
    return (
        f"pos={g('position'):>9} rad  vel={g('velocity'):>8}  torque={g('torque'):>8}  "
        f"T_mos={g('temperature_mos', '{:.0f}'):>3}°C  T_rot={g('temperature_rotor', '{:.0f}'):>3}°C  "
        f"err={err_txt}"
    )


def main() -> int:
    ap = argparse.ArgumentParser(description="Ping YAM motors over the CANable (macOS).")
    ap.add_argument("--yes", action="store_true", help="actually transmit (default: dry run)")
    ap.add_argument("--ids", type=int, nargs="+", default=DEFAULT_IDS)
    ap.add_argument("--bitrate", type=int, default=YAM_BITRATE)
    ap.add_argument("--arm", default=DEFAULT_ARM, choices=sorted(ARM_SERIALS), help="WHICH ARM. Selected by serial, never by index.")
    ap.add_argument(
        "--motor-type",
        default=None,
        help="force one motor type for every motor. Default: the measured per-motor map "
        "(YAM_MOTOR_TYPES). Affects reply DECODING only, never what is transmitted.",
    )
    ap.add_argument(
        "--attempt-error-clear",
        action="store_true",
        help="allow the vendor retry loop to CLEAR a latched motor fault (it can also spin on a "
        "hard fault). Default off, which reports the fault and leaves it in place so it can be "
        "diagnosed. ⚠️ Until 2026-08-14 this flag was wired to nothing and clearing always happened.",
    )
    args = ap.parse_args()

    types = {mid: (args.motor_type or YAM_MOTOR_TYPES.get(mid, "DM4310")) for mid in args.ids}

    print(f"ARM              : {args.arm}  (serial {ARM_SERIALS[args.arm]})")
    print(f"motor IDs to ping : {args.ids}")
    print(f"bitrate           : {args.bitrate}")
    print(f"decode as         : {types}")
    print("frames per motor  : enable (…FC) then disable (…FD)")
    if args.attempt_error_clear:
        print("on a latched fault: ⚠️ CLEAR IT (…FB), repeatedly, until the code reads normal")
    else:
        print("on a latched fault: report it and LEAVE IT ALONE, so it can be diagnosed")
    print()

    if not args.yes:
        print("DRY RUN — nothing was transmitted. Re-run with --yes to actually ping.")
        return 0

    # add_i2rt_to_path() must run before this import: open_motor_interface() also
    # calls it, but that happens later and the import would already have failed.
    add_i2rt_to_path()
    from i2rt.motor_drivers.dm_driver import MotorType  # noqa: PLC0415

    motor_types = {mid: getattr(MotorType, name) for mid, name in types.items()}

    iface = open_motor_interface(bitrate=args.bitrate, arm=args.arm)
    print("bus open (normal mode — the adapter is now an active CAN node)\n")

    alive: list[int] = []
    enabled: list[int] = []
    faulted: list[tuple[int, int | None]] = []

    def ping_one(motor_id: int) -> None:
        """Enable one motor for a frame, report what it said, disable it again."""
        try:
            info = iface.motor_on(motor_id, motor_types[motor_id])
            enabled.append(motor_id)
            alive.append(motor_id)
            print(f"  ✓ motor {motor_id} ({types[motor_id]}): {describe(info)}")
        except MotorFaultNotCleared as fault:
            # ⛔ NOT a communication failure. The motor answered, and what it said
            # was "I am in a fault". This is the branch that used to be invisible:
            # the vendor cleared the fault and reported a healthy motor instead.
            alive.append(motor_id)
            faulted.append((motor_id, fault.code))
            print(f"  ⛔ motor {motor_id} ({types[motor_id]}): LATCHED FAULT, left in place")
            print(f"       {describe_motor_error(fault.code)}")
        except Exception as exc:  # noqa: BLE001
            print(f"  ✗ motor {motor_id}: no reply ({type(exc).__name__})")
        finally:
            try:
                iface.motor_off(motor_id)
                if motor_id in enabled:
                    enabled.remove(motor_id)
            except Exception as exc:  # noqa: BLE001
                print(f"    ⚠️  motor {motor_id} did not confirm disable: {type(exc).__name__}")
            time.sleep(0.02)

    try:
        # ⭐ This context manager is the whole of the fix. Inside it the vendor's
        # clean_error refuses and raises MotorFaultNotCleared instead, carrying the
        # code the parser recorded on its way past. Outside it, nothing changes.
        with do_not_clear_motor_faults() if not args.attempt_error_clear else nullcontext():
            for motor_id in args.ids:
                ping_one(motor_id)
    finally:
        for motor_id in list(enabled):
            try:
                iface.motor_off(motor_id)
            except Exception:  # noqa: BLE001, S110
                pass
        iface.close()
        print("\nbus closed; every motor was sent a disable frame.")

    print(f"\nonline motors: {alive}")
    if not alive:
        print(
            "Nothing replied. In order of likelihood: the arm is not powered, the CAN\n"
            "wiring/termination is wrong, the bitrate is not 1 Mbit/s, or the motor IDs\n"
            "differ on this arm."
        )
        return 1
    if len(alive) < len(args.ids):
        print(f"missing: {[i for i in args.ids if i not in alive]}")

    if faulted:
        print(f"\n⛔ {len(faulted)} motor(s) are holding a LATCHED FAULT, and it was NOT cleared:")
        for motor_id, code in faulted:
            print(f"   motor {motor_id}: {describe_motor_error(code)}")
        print(
            "\n   These motors will be showing a FLASHING RED light. Write down the codes\n"
            "   before doing anything else — they are the only record of what happened,\n"
            "   and the next enable frame from any tool will clear them (docs/FINDINGS.md §39).\n"
            "   To clear them deliberately:  uv run scripts/ping_motors.py --arm "
            f"{args.arm} --yes --attempt-error-clear"
        )
        return 1

    print("✓ no motor is holding a fault.")
    if not args.attempt_error_clear:
        print(
            "  ⭐ And that is a real reading rather than a cleared one: error clearing was OFF,\n"
            "     so a latched fault would have been reported instead of erased."
        )
    return 0


if __name__ == "__main__":
    sys.exit(main())
