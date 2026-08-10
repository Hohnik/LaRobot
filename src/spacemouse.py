"""Finding, opening and safely releasing the SpaceMouse on macOS.

Shared by `scripts/probe_hardware.py` and `src/spacemouse_live.py`. It exists
because those two had their own copies of the device-selection logic, and the
webcam bug fixed on 2026-08-08 was fixed in only one of them.

⛔ READ THIS BEFORE OPENING THE DEVICE — it can take over the cursor.

hidapi on macOS opens HID devices **exclusively** (`hid_darwin_get_open_exclusive()`
returns 1, verified on this machine). While we hold the SpaceMouse, macOS's own
HID stack stops receiving its reports entirely. For teleop that is exactly right
— you do not want the pointer flying around while driving a robot.

The hazard is the handover. macOS keeps the *last* report it saw. Seize the
device while the puck is deflected and macOS is left holding a non-zero pointer
delta with no zeroing report ever arriving, so the cursor drifts in that
direction forever and fights the real mouse. That happened on 2026-08-10 and
cost Julien control of the machine until he unplugged the device.

`countdown_hands_off()` is the fix: let the puck return to centre, so the state
macOS latches is zero. Same reasoning applies on close — release with the puck
at rest and macOS resumes cleanly.
"""

from __future__ import annotations

import sys
import time
from typing import Any

import hid

SPACEMOUSE_VID = 0x256F  # 3Dconnexion's own
LEGACY_VID = 0x046D      # Logitech — 3Dconnexion shipped under it for years
VIDS = {SPACEMOUSE_VID, LEGACY_VID}

MULTI_AXIS_USAGE_PAGE = 0x01  # Generic Desktop
MULTI_AXIS_USAGE = 0x08       # Multi-axis Controller


def find_device() -> dict | None:
    """Pick the SpaceMouse's motion interface.

    The order matters, and the reason is specific to this dock. VID 0x046D is
    Logitech, which older 3Dconnexion units shipped under — but it is *also* the
    C920 webcam on this same hub, and a webcam presents HID interfaces too. So a
    Logitech device is accepted only if it independently identifies as
    multi-axis; blind fallback is allowed for 3Dconnexion's own VID and nothing
    else. Picking the webcam would open cleanly, print a plausible product
    string, and then never report motion — indistinguishable from a decode bug.
    """
    cands = [d for d in hid.enumerate() if d["vendor_id"] in VIDS]
    multi = [
        d
        for d in cands
        if d.get("usage_page") == MULTI_AXIS_USAGE_PAGE and d.get("usage") == MULTI_AXIS_USAGE
    ]
    if multi:
        return multi[0]
    own = [d for d in cands if d["vendor_id"] == SPACEMOUSE_VID]
    return own[0] if own else None


def is_multi_axis(info: dict) -> bool:
    return (
        info.get("usage_page") == MULTI_AXIS_USAGE_PAGE
        and info.get("usage") == MULTI_AXIS_USAGE
    )


def countdown_hands_off(seconds: int = 3) -> None:
    """Give the puck time to centre before we seize the device.

    See the module docstring: opening mid-deflection strands macOS with a
    latched pointer delta and the cursor drifts until the device is unplugged.
    """
    print("⚠️  TAKE YOUR HANDS OFF THE SPACEMOUSE and let it centre.")
    print("    macOS is about to lose this device to us — that is intended — but if")
    print("    the puck is deflected at that moment, the cursor will drift and fight")
    print("    your real mouse until the device is unplugged.")
    for remaining in range(seconds, 0, -1):
        sys.stdout.write(f"\r    seizing in {remaining} … ")
        sys.stdout.flush()
        time.sleep(1.0)
    print("\r    seizing now.        ")


def open_device(info: dict) -> Any:
    """Open the device, supporting both hidapi Python bindings.

    The PyPI `hidapi` package (cython-hidapi) exposes hid.device()/open_path();
    the differently-named `hid` package exposes hid.Device(path=...). Support
    both rather than guessing which is installed.
    """
    if hasattr(hid, "device"):
        handle = hid.device()
        handle.open_path(info["path"])
        return handle
    return hid.Device(path=info["path"])


def describe(info: dict) -> str:
    return (
        f"{info.get('product_string')} "
        f"[{info['vendor_id']:#06x}:{info['product_id']:#06x}] "
        f"usage_page={info.get('usage_page'):#06x} usage={info.get('usage'):#04x}"
    )
