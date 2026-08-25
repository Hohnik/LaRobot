#!/usr/bin/env python3
"""Passively watch the CAN bus through the CANable, without ever transmitting.

    uv run apps/probe_can.py              # 5 s, listen-only, 1 Mbit/s
    uv run apps/probe_can.py --seconds 20

⛔ SAFETY — why this cannot disturb the arm.

The adapter is started with GS_CAN_MODE_LISTEN_ONLY, which puts the CAN
transceiver into *silent* mode: the controller drives no dominant bits at all,
so it does not even send acknowledge bits, let alone data or error frames. A
listen-only node is electrically invisible to the bus. That is a stronger
guarantee than "we simply never call send()" — a normal-mode node still ACKs
every frame it hears, and at a *wrong* bitrate it would inject error frames.

This script verifies listen-only was actually granted and REFUSES to listen if
the adapter would not honour it, rather than silently falling back to a mode
that talks.

⚠️ EXPECTATION — silence here is NOT a failure.

The YAM's DM motors are request/response devices: they answer when polled and
say nothing otherwise. With nothing else driving the bus, zero frames is the
*normal* result even when the arm is perfectly healthy and correctly wired.
So this probe can prove the adapter and the stack work, and it can reveal any
other traffic present, but it cannot by itself prove the arm is alive. That
takes a poll, which transmits — a separate, deliberate step.
"""

from __future__ import annotations

import argparse
import sys
import time
from collections import Counter
from pathlib import Path

from gs_usb.constants import GS_CAN_MODE_HW_TIMESTAMP, GS_CAN_MODE_LISTEN_ONLY
from gs_usb.gs_usb import GsUsb
from gs_usb.gs_usb_frame import GsUsbFrame

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))
from yam.can import YAM_BITRATE, patch_gs_usb_for_macos  # noqa: E402

DEFAULT_BITRATE = YAM_BITRATE  # I2RT documents 1 Mbit/s for the YAM


def apply_bitrate(dev: GsUsb, bitrate: int) -> str:
    """Set the bit timing, preferring gs_usb's table and falling back to python-can.

    gs_usb.set_bitrate() only knows a couple of controller clocks. python-can can
    derive timings for any fclk, so it is the more general path — but it is the
    fallback rather than the default because the table is the vendor's own.
    """
    try:
        if dev.set_bitrate(bitrate):
            return "gs_usb.set_bitrate table"
    except Exception as exc:  # noqa: BLE001
        print(f"  set_bitrate() raised ({type(exc).__name__}: {exc}); computing timings instead")

    import can

    timing = can.BitTiming.from_sample_point(
        f_clock=dev.device_capability.fclk_can, bitrate=bitrate, sample_point=87.5
    )
    prop_seg = 1
    dev.set_timing(
        prop_seg=prop_seg,
        phase_seg1=timing.tseg1 - prop_seg,
        phase_seg2=timing.tseg2,
        sjw=timing.sjw,
        brp=timing.brp,
    )
    return f"computed (fclk={dev.device_capability.fclk_can} Hz)"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--seconds", type=float, default=5.0)
    ap.add_argument("--bitrate", type=int, default=DEFAULT_BITRATE)
    ap.add_argument("--max-print", type=int, default=20, help="frames to print in full")
    args = ap.parse_args()

    # ⛔⭐ REFUSE ON LINUX RATHER THAN FAIL OBSCURELY (ROADMAP §8.2 item 49). This tool speaks
    # to the adapter through libusb to get gs_usb's LISTEN-ONLY mode, which is the property
    # that makes it electrically invisible. On Linux the kernel driver owns the adapter, so
    # libusb cannot claim it and the failure surfaces as an unrelated-looking USB error. The
    # Linux equivalent is a listen-only SocketCAN interface plus candump, and naming it is
    # more useful than a traceback.
    from yam.platform import IS_LINUX  # noqa: PLC0415

    if IS_LINUX:
        print("⛔ This probe is macOS-only, on purpose.\n")
        print("   It uses libusb to put the adapter in gs_usb LISTEN-ONLY mode, and on Linux")
        print("   the kernel driver owns the adapter, so libusb cannot claim it.\n")
        print("   The Linux equivalent, listen-only so the node stays electrically invisible:")
        print("     sudo ip link set can0 down")
        print("     sudo ip link set can0 type can bitrate 1000000 listen-only on")
        print("     sudo ip link set can0 up")
        print("     candump -td -x can0        # Ctrl-C to stop")
        print("     sudo ip link set can0 down && sudo ip link set can0 type can \\")
        print("       bitrate 1000000 listen-only off && sudo ip link set can0 up\n")
        print("   ⚠️ Leave listen-only OFF afterwards, or no motor will ever answer: a")
        print("      listen-only node cannot ACK, which is exactly why it is safe to watch")
        print("      with and useless to drive with.")
        print("   ⭐ For device state instead, `uv run checks/check_platform.py` works "
              "everywhere.")
        return 1

    patch_gs_usb_for_macos()

    devs = GsUsb.scan()
    if not devs:
        print("No candlelight/gs_usb adapter found. Is the CANable plugged in?")
        return 1
    if len(devs) > 1:
        print(f"⚠️  {len(devs)} adapters found; using the first.")
    dev = devs[0]
    print(f"adapter: {dev}")
    print(f"  serial={dev.serial_number}  bus={dev.bus} address={dev.address}")

    cap = dev.device_capability
    if not (cap.feature & GS_CAN_MODE_LISTEN_ONLY):
        print(
            "\n⛔ REFUSING TO CONTINUE: this adapter does not advertise listen-only mode.\n"
            "   Opening it would put a talking node on the bus, which is not what this\n"
            "   script promises. Nothing was started; nothing was transmitted."
        )
        return 2

    how = apply_bitrate(dev, args.bitrate)
    print(f"  bitrate={args.bitrate} via {how}")

    dev.start(flags=GS_CAN_MODE_LISTEN_ONLY | GS_CAN_MODE_HW_TIMESTAMP)
    granted = dev.device_flags & GS_CAN_MODE_LISTEN_ONLY
    if not granted:
        dev.stop()
        print("\n⛔ listen-only was requested but NOT granted. Stopped without listening.")
        return 2
    print("  ✓ listen-only granted — the transceiver is silent, it cannot even ACK\n")

    print(f"listening {args.seconds:g} s …")
    seen = 0
    ids: Counter[int] = Counter()
    frame = GsUsbFrame()
    deadline = time.time() + args.seconds
    try:
        while time.time() < deadline:
            if dev.read(frame=frame, timeout_ms=10):
                seen += 1
                ids[frame.arbitration_id] += 1
                if seen <= args.max_print:
                    payload = bytes(frame.data)[: frame.can_dlc].hex(" ")
                    print(f"  id={frame.arbitration_id:#05x} dlc={frame.can_dlc} data={payload}")
    except KeyboardInterrupt:
        print("\ninterrupted.")
    finally:
        dev.stop()

    print(f"\n{seen} frame(s) in {args.seconds:g} s.")
    if ids:
        print("distinct arbitration IDs:")
        for arb_id, count in sorted(ids.items()):
            print(f"  {arb_id:#05x}  ×{count}")
    else:
        print(
            "Bus is quiet. Read the expectation note at the top of this file before\n"
            "concluding anything: DM motors only speak when polled, so silence is the\n"
            "expected reading for a healthy, idle arm."
        )
    print("\nNothing was transmitted. The bus was never driven.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
