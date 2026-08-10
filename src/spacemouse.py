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


FULL_SCALE = 350.0  # raw counts at full deflection; nominal
DEFAULT_DEADZONE = 0.06


class TwistReader:
    """Turn a stream of HID reports into six normalised axes in [-1, 1].

    Lives here, once, because this decode already existed in two files and is
    about to be needed by a third — and the last time device logic was
    duplicated, a bug fix landed in only one copy (see `find_device`).

    Non-blocking: `read()` drains whatever reports have arrived and returns the
    latest known deflection, so a caller running at 100 Hz never blocks and
    never falls behind.

    Axis order is `[x, y, z, roll, pitch, yaw]`. This unit uses the **split**
    report shape — `0x01` carries translation, `0x02` rotation — confirmed
    against raw bytes on 2026-08-10; the combined 13-byte `0x01` shape is
    handled too, since 3Dconnexion firmware differs between units.
    """

    def __init__(self, handle: Any, deadzone: float = DEFAULT_DEADZONE, full_scale: float = FULL_SCALE):
        self._handle = handle
        self._deadzone = deadzone
        self._full_scale = full_scale
        self._t = [0.0, 0.0, 0.0]
        self._r = [0.0, 0.0, 0.0]
        self.buttons = 0

    def _scale(self, raw: int) -> float:
        v = max(-1.0, min(1.0, raw / self._full_scale))
        return 0.0 if abs(v) < self._deadzone else v

    def read(self) -> list[float]:
        import struct

        while True:
            data = self._handle.read(64)
            if not data:
                break
            rid, payload = data[0], bytes(data[1:])
            if rid == 0x01 and len(payload) >= 12:
                vals = struct.unpack("<6h", payload[:12])
                self._t = [self._scale(v) for v in vals[:3]]
                self._r = [self._scale(v) for v in vals[3:]]
            elif rid == 0x01 and len(payload) >= 6:
                self._t = [self._scale(v) for v in struct.unpack("<3h", payload[:6])]
            elif rid == 0x02 and len(payload) >= 6:
                self._r = [self._scale(v) for v in struct.unpack("<3h", payload[:6])]
            elif rid == 0x03 and payload:
                self.buttons = int.from_bytes(payload[:2], "little")
        return [*self._t, *self._r]


def describe(info: dict) -> str:
    return (
        f"{info.get('product_string')} "
        f"[{info['vendor_id']:#06x}:{info['product_id']:#06x}] "
        f"usage_page={info.get('usage_page'):#06x} usage={info.get('usage'):#04x}"
    )
