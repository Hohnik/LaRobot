#!/usr/bin/env python3
"""Probe what teleop hardware is attached and whether we can actually talk to it.

Read-only. Opens the SpaceMouse for a short listen and never writes to any device.
Nothing here can move a robot.

    uv run scripts/probe_hardware.py
"""

from __future__ import annotations

import sys
import time

import hid

SPACEMOUSE_VIDS = {0x256F, 0x046D}  # 3Dconnexion; older units shipped under Logitech


def enumerate_all() -> list[dict]:
    return hid.enumerate()


def describe(d: dict) -> str:
    return (
        f"{d.get('manufacturer_string') or '?':<24} "
        f"{d.get('product_string') or '?':<28} "
        f"VID={d['vendor_id']:#06x} PID={d['product_id']:#06x} "
        f"usage_page={d.get('usage_page'):#06x} usage={d.get('usage'):#04x}"
    )


def main() -> int:
    devices = enumerate_all()
    print(f"=== {len(devices)} HID interface(s) enumerated ===\n")

    space = [d for d in devices if d["vendor_id"] in SPACEMOUSE_VIDS]
    others = [d for d in devices if d["vendor_id"] not in SPACEMOUSE_VIDS]

    print("--- 3Dconnexion / SpaceMouse ---")
    for d in space:
        print("  " + describe(d))
    if not space:
        print("  none found")

    print(f"\n--- everything else ({len(others)}) ---")
    for d in others:
        print("  " + describe(d))

    if not space:
        print("\nNo SpaceMouse interface. Is it plugged in?")
        return 1

    # A SpaceMouse exposes several interfaces; the multi-axis controller is the one
    # that streams motion. usage_page 0x01 (generic desktop) / usage 0x08 (multi-axis).
    target = next(
        (d for d in space if d.get("usage_page") == 0x01 and d.get("usage") == 0x08),
        space[0],
    )
    print(f"\n=== opening: {target.get('product_string')} (path={target['path']!r}) ===")

    # The PyPI `hidapi` package (cython-hidapi) exposes hid.device()/open_path().
    # The differently-named `hid` package exposes hid.Device(path=...). Support both,
    # rather than guessing which is installed.
    h = None
    try:
        if hasattr(hid, "device"):
            h = hid.device()
            h.open_path(target["path"])
        else:
            h = hid.Device(path=target["path"])
    except OSError as exc:
        # Only an OSError here is plausibly a permissions/claim problem.
        print(f"\n✗ COULD NOT OPEN (OSError): {exc}")
        print(
            "\nOn macOS this is usually permissions, not wiring:\n"
            "  System Settings → Privacy & Security → Input Monitoring → allow the terminal.\n"
            "A running 3Dconnexion driver can also claim the device exclusively."
        )
        return 2
    except Exception as exc:  # noqa: BLE001
        print(f"\n✗ COULD NOT OPEN ({type(exc).__name__}): {exc}")
        print("  Not a permissions symptom — this looks like a library/API problem.")
        return 2

    print("✓ opened. Listening 5 s — move the SpaceMouse now.\n")
    seen, deadline = 0, time.time() + 5.0
    h.set_nonblocking(True)
    while time.time() < deadline:
        data = h.read(64)
        if data:
            seen += 1
            if seen <= 12:
                print(f"  report id={data[0]:#04x} len={len(data):<3} {list(data[1:9])}")
        else:
            time.sleep(0.005)
    h.close()

    print(f"\n{seen} report(s) received.")
    if seen == 0:
        print("Opened but silent — either it was not moved, or another process holds it.")
        return 3
    print("✓ The SpaceMouse is readable from Python.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
