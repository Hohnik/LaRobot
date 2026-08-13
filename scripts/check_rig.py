#!/usr/bin/env python3
"""What state is every device on this rig in? One command, one pasteable answer.

    uv run scripts/check_rig.py
    uv run scripts/check_rig.py --raw      # also list hubs and everything else

⭐ It reads the USB bus and prints. Nothing is opened for control, nothing is energised,
nothing moves. Safe for the agent to run under [HANDOFF §4](../docs/HANDOFF.md) rule 1.

⛔⭐ WHY THIS EXISTS. [FINDINGS §32](../docs/FINDINGS.md) established that *"is it plugged
in?"* is the wrong first question on this rig, because three separate failures in two days
were all **a device that is present but not in the state the code assumes**: a camera
claimed by macOS's own driver, a camera reporting two different serial numbers, and both
CAN adapters running their firmware-update bootloader instead of their firmware. The useful
question is *"what state is it in?"*, and until now answering it took several different
commands and a person who already knew which ones.

⭐ It also answers the recurrence questions [FINDINGS §32](../docs/FINDINGS.md) asks to
record if the DFU fault comes back: whether it is one adapter or both, and whether the hub
re-enumerated. Run it BEFORE touching anything, and paste the output.

⚠️ It cannot tell you whether something was knocked or replugged, and it cannot prove a
motor is alive — that needs a poll, which transmits. `scripts/probe_can.py` is the next step
up and is still silent; `scripts/ping_motors.py` transmits and is Julien's to run.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "src"))

from yam_can import ARM_SERIALS, DFU_PID, DFU_VID  # noqa: E402

#: The working CANable. Measured from `probe_can.py`'s own device line, `1d50:606f`.
CANDLELIGHT = (0x1D50, 0x606F)
#: 3Dconnexion's own VID. `src/spacemouse.py` also accepts Logitech's 0x046D for older
#: units, but both pucks here report the modern one.
SPACEMOUSE_VID = 0x256F
#: Intel RealSense D405. ⚠️ Two of them report the SAME vid:pid, which is the whole
#: difficulty in ROADMAP §7.1 — but they do NOT report the same serial, and this reads it.
D405 = (0x8086, 0x0B5B)
#: Hubs and dock plumbing. Listed only under --raw, so the interesting lines stay visible.
INFRASTRUCTURE_VIDS = {0x05E3, 0x0BDA, 0x1A40, 0x291A, 0x0B95}


def field(dev, name: str) -> str:
    """A USB string descriptor, or `"?"`. ⚠️ Reading one can raise on a claimed device."""
    try:
        return str(getattr(dev, name) or "")
    except Exception:  # noqa: BLE001
        return "?"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--raw", action="store_true",
                    help="also list hubs and anything unrecognised")
    args = ap.parse_args()

    try:
        import usb.core
    except Exception as exc:  # noqa: BLE001
        print(f"⛔ cannot read the USB bus: {type(exc).__name__}: {exc}")
        print("   pyusb is a dependency of this project — try `uv sync`.")
        return 1

    devices = []
    for dev in usb.core.find(find_all=True) or []:
        devices.append({
            "vid": dev.idVendor, "pid": dev.idProduct,
            "bus": dev.bus, "addr": dev.address,
            "serial": field(dev, "serial_number"),
            "product": field(dev, "product"),
        })
    if not devices:
        # ⚠️ An earlier version called this "a fault", which is wrong and was written
        # without ever seeing the case. `usb.core` lists external devices; on a laptop
        # with its dock unplugged the correct answer is zero, and that is what Julien's
        # desk looks like when he goes home. Saying "fault" would send the next reader
        # hunting for a problem that is a disconnected cable. FINDINGS §37.4.
        print("⭕ Nothing is attached to USB at all.")
        print("   That is the expected reading with the dock or hub unplugged, and it is")
        print("   NOT by itself a fault. Plug the rig in and run this again.")
        return 1

    def pick(*, vid=None, pid=None):
        return [d for d in devices
                if (vid is None or d["vid"] == vid) and (pid is None or d["pid"] == pid)]

    print(f"RIG STATE — {len(devices)} USB devices. Reads only; nothing was energised.\n")

    # ---------------------------------------------------------- the two arms ----
    working = {d["serial"]: d for d in pick(vid=CANDLELIGHT[0], pid=CANDLELIGHT[1])}
    dfu = pick(vid=DFU_VID, pid=DFU_PID)
    print("CAN adapters — one per arm")
    problems = []
    for arm, serial in ARM_SERIALS.items():
        if serial in working:
            d = working[serial]
            print(f"  {arm}  {serial}  ✓ running its firmware   bus {d['bus']} addr {d['addr']}")
        elif any(serial.startswith(x["serial"]) for x in dfu if x["serial"]):
            print(f"  {arm}  {serial}  ⛔ DFU BOOTLOADER — no arm can run")
            problems.append(f"adapter {arm} is in DFU mode")
        else:
            print(f"  {arm}  {serial}  ⛔ ABSENT from the bus")
            problems.append(f"adapter {arm} is not attached")
    if dfu:
        # ⛔ The serial is truncated to 12 characters in DFU mode, so it never matches
        # ARM_SERIALS. Print what was actually read rather than a guess at which arm.
        print(f"  ⛔ {len(dfu)} board(s) in DFU mode, serials truncated to 12 chars: "
              f"{', '.join(x['serial'] or '?' for x in dfu)}")
        print("     Recovery ladder, cheapest first: docs/FINDINGS.md §32.0")
    else:
        print("  no board is in DFU mode.")

    # ---------------------------------------------------------- the two pucks ----
    mice = pick(vid=SPACEMOUSE_VID)
    print(f"\nSpaceMice — {len(mice)} attached")
    for d in mice:
        print(f"  {d['product']:<20} bus {d['bus']} addr {d['addr']}  "
              f"serial {d['serial'] or '(empty, by design)'}")
    if len(mice) < 2:
        print("  ⚠️ Two are expected. Serials are empty on these units, so which puck "
              "drives which arm\n     is settled by asking the operator to wiggle one "
              "(docs/FINDINGS.md §5).")

    # ------------------------------------------------------------- the cameras ----
    cams = pick(vid=D405[0], pid=D405[1])
    others = [d for d in devices
              if d["vid"] not in INFRASTRUCTURE_VIDS
              and (d["vid"], d["pid"]) not in {CANDLELIGHT, D405, (DFU_VID, DFU_PID)}
              and d["vid"] != SPACEMOUSE_VID]
    print(f"\nCameras — {len(cams)} RealSense D405 attached")
    for d in cams:
        print(f"  D405  serial {d['serial']}  bus {d['bus']} addr {d['addr']}")
    if len(cams) > 1:
        print("  ⚠️ MORE THAN ONE D405. The identification trick used elsewhere in this "
              "repo asks each\n     camera for a picture size only one model supports, and "
              "two D405s support the same\n     sizes — so that method cannot separate "
              "them (docs/ROADMAP.md §7.1).")
        print("  ⭐ They DO have distinct USB serials, printed above, read with no root. "
              "What is still\n     unsolved is mapping a serial to an OpenCV camera index: "
              "macOS's enumeration order\n     is not OpenCV's (docs/FINDINGS.md §22). Use "
              "the wiggle method for which-camera-is-\n     on-which-arm (docs/FINDINGS.md "
              "§28.6).")
    for d in others:
        print(f"  other  {d['product']:<34} {d['vid']:#06x}:{d['pid']:#06x}  "
              f"serial {d['serial']}")

    # -------------------------------------------------------------- topology ----
    if args.raw:
        print("\nEverything on the bus (hubs included — this is the "
              '"did the hub re-enumerate?" answer)')
        for d in sorted(devices, key=lambda x: (x["bus"], x["addr"])):
            print(f"  bus {d['bus']} addr {d['addr']:>3}  {d['vid']:#06x}:{d['pid']:#06x}  "
                  f"{d['product']:<34} {d['serial']}")
    else:
        buses = sorted({d["bus"] for d in devices})
        print(f"\nTopology: {len(devices)} devices across bus(es) {buses}. "
              "Use --raw for the full list,")
        print("  which is what to paste if the DFU fault recurs and the question is "
              "whether the hub\n  re-enumerated (docs/FINDINGS.md §32).")

    print()
    if problems:
        print(f"⛔ VERDICT: not ready — {'; '.join(problems)}.")
        return 1
    print("✓ VERDICT: both arms are on the bus and running their firmware.")
    print("  ⚠️ That does NOT prove a motor is alive. This never transmits, so it cannot. "
          "The next\n     silent step is `uv run scripts/probe_can.py`; a poll transmits "
          "and is Julien's to run.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
