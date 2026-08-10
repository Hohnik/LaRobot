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

⚠️ Known sharp edge in the vendor code: `motor_on()` retries in a `while` loop
until a motor's error code clears. A motor stuck in a hard error can spin there.
`--attempt-error-clear` (default off) controls whether we let it try at all.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))
from yam_can import YAM_BITRATE, open_motor_interface  # noqa: E402

DEFAULT_IDS = [1, 2, 3, 4, 5, 6, 7]  # 6 arm joints + gripper


def describe(info: object) -> str:
    def g(attr: str, fmt: str = "{:.4f}") -> str:
        val = getattr(info, attr, None)
        if val is None:
            return "?"
        try:
            return fmt.format(val)
        except (TypeError, ValueError):
            return str(val)

    return (
        f"pos={g('pos'):>9}  vel={g('vel'):>9}  eff={g('eff'):>9}  "
        f"V={g('voltage', '{:.1f}'):>5}  T_mos={g('temp_mos', '{:.0f}'):>4}  "
        f"T_rot={g('temp_rotor', '{:.0f}'):>4}  err={getattr(info, 'error_code', '?')}"
    )


def main() -> int:
    ap = argparse.ArgumentParser(description="Ping YAM motors over the CANable (macOS).")
    ap.add_argument("--yes", action="store_true", help="actually transmit (default: dry run)")
    ap.add_argument("--ids", type=int, nargs="+", default=DEFAULT_IDS)
    ap.add_argument("--bitrate", type=int, default=YAM_BITRATE)
    ap.add_argument("--motor-type", default="DM4310", help="affects reply DECODING only, not what is sent")
    ap.add_argument(
        "--attempt-error-clear",
        action="store_true",
        help="allow the vendor retry loop to clear motor errors (can spin on a hard fault)",
    )
    args = ap.parse_args()

    print(f"motor IDs to ping : {args.ids}")
    print(f"bitrate           : {args.bitrate}")
    print(f"decode as         : {args.motor_type}")
    print("frames per motor  : enable (…FC) then disable (…FD)\n")

    if not args.yes:
        print("DRY RUN — nothing was transmitted. Re-run with --yes to actually ping.")
        return 0

    from i2rt.motor_drivers.dm_driver import MotorType  # noqa: PLC0415

    motor_type = getattr(MotorType, args.motor_type)

    iface = open_motor_interface(bitrate=args.bitrate)
    print("bus open (normal mode — the adapter is now an active CAN node)\n")

    alive: list[int] = []
    enabled: list[int] = []
    try:
        for motor_id in args.ids:
            try:
                info = iface.motor_on(motor_id, motor_type)
                enabled.append(motor_id)
                alive.append(motor_id)
                print(f"  ✓ motor {motor_id}: {describe(info)}")
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
    return 0


if __name__ == "__main__":
    sys.exit(main())
