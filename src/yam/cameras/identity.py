"""Join a camera's USB serial to its AVFoundation identity — no root, no camera permission.

⭐⭐ THE MEASURED FACT THIS ENCODES ([FINDINGS §70.15](../../../docs/FINDINGS.md)): on macOS, a USB camera's AVFoundation `uniqueID` is one 64-bit number in hex — **locationID in the high 32 bits, then VID (16), then PID (16)**. Verified against all three cameras on this rig, digit for digit, against the ioreg capture of FINDINGS §70.6. The locationID names the physical PORT PATH, ioreg reports it beside the USB serial with no privileges, and AVFoundation reports the uniqueID without opening any camera — so the chain **serial → locationID → uniqueID** closes two identical D405s apart WITHOUT covering a lens.

⚠️ What this chain still cannot do alone: name an OpenCV INDEX. OpenCV exposes no uniqueID, and its index order is not any macOS list order ([FINDINGS §22](../../../docs/FINDINGS.md)). The join to an index still comes from measurement (the mode probe for distinct models) or one physical confirmation (cover a lens) — but once made, the hint can be keyed by uniqueID and RE-DERIVED per session from the serial, so it survives replugs into the same port and names the right camera after a port change is re-confirmed once.
"""

from __future__ import annotations

import re
import subprocess

#: One ioreg device block: we need these three lines from each.
_NAME = re.compile(r'"USB Product Name" = "([^"]*)"')
_SERIAL = re.compile(r'"USB Serial Number" = "([^"]*)"')
_LOCATION = re.compile(r'"locationID" = (\d+)')
_VID = re.compile(r'"idVendor" = (\d+)')
_PID = re.compile(r'"idProduct" = (\d+)')


def usb_unique_id(location_id: int, vid: int, pid: int) -> str:
    """The AVFoundation uniqueID a USB camera at this port will report.

    The packing is the measured one: `(locationID << 32) | (vid << 16) | pid`, printed
    as bare hex with a `0x` prefix and no leading zeros — exactly how AVFoundation
    prints it (`0x122000080860b5b` on this rig's arm-B D405).
    """
    return hex((location_id << 32) | (vid << 16) | pid)


def parse_ioreg(text: str) -> list[dict]:
    """Every USB device in an `ioreg -p IOUSB -w0 -l` dump, as plain dicts.

    Pure, so the parser is tested against a canned dump of the real rig — the live
    command changes with every replug, a fixture does not.
    """
    devices: list[dict] = []
    # ioreg nests children with +-o markers; each device's properties follow its marker.
    for chunk in re.split(r"\+-o ", text)[1:]:
        name = _NAME.search(chunk)
        loc = _LOCATION.search(chunk)
        if not (name and loc):
            continue
        serial = _SERIAL.search(chunk)
        vid, pid = _VID.search(chunk), _PID.search(chunk)
        devices.append({
            "name": name.group(1),
            "serial": serial.group(1) if serial else "",
            "location_id": int(loc.group(1)),
            "vid": int(vid.group(1)) if vid else None,
            "pid": int(pid.group(1)) if pid else None,
        })
    return devices


def read_ioreg() -> list[dict]:
    """The live USB tree. ⚠️ A dated reading the moment it returns — never cache it."""
    out = subprocess.run(["ioreg", "-p", "IOUSB", "-w0", "-l"],
                         capture_output=True, text=True, check=True)
    return parse_ioreg(out.stdout)


def unique_id_for_serial(serial: str, devices: list[dict] | None = None) -> str | None:
    """The AVFoundation uniqueID of the USB camera carrying `serial`, or None.

    ⛔ Refuses (None) rather than guessing when the serial is absent or the device
    lacks VID/PID — an invented identity here would select the WRONG camera with
    full confidence, the exact failure this module exists to end.
    """
    for dev in devices if devices is not None else read_ioreg():
        if dev["serial"] == serial and dev["vid"] is not None and dev["pid"] is not None:
            return usb_unique_id(dev["location_id"], dev["vid"], dev["pid"])
    return None


def devices_matching_serial(prefix: str, devices: list[dict]) -> list[dict]:
    """Every USB device whose serial STARTS WITH `prefix` (and has VID/PID).

    ⭐ Prefixes exist because Julien dictates his commands: `d405:2553` is speakable and `d405:255323071773` is not. The caller refuses on zero matches and on more than one — a prefix that is ambiguous today (it never is on this rig: `2553` vs `2603`) must be typed longer, never guessed at.
    """
    prefix = prefix.strip()
    if not prefix:
        return []
    return [d for d in devices
            if d["serial"] and d["serial"].startswith(prefix)
            and d["vid"] is not None and d["pid"] is not None]


# ------------------------------------------------------ the same question, on Linux ----
# ⭐⭐ On Linux the chain above is unnecessary: `/dev/v4l/by-id` already names every camera by
# model AND serial and points straight at `/dev/videoN`, which is OpenCV's index. So the
# Linux resolver is a match against that listing, and the two identical D405s that cost this
# repo days on macOS ([FINDINGS §70.15](../../../docs/FINDINGS.md)) separate by serial with no
# hint file, no mode probe and no lens-covering. ROADMAP §8.2 item 49.

#: Words a person types that do not appear literally in a by-id name. The D405 calls itself
#: "Intel_R__RealSense_TM__Depth_Camera_405", so `d405` has to reach it through `405`.
LINUX_ALIASES = {"d405": "405", "realsense": "realsense", "intel": "realsense",
                 "c920": "c920", "logitech": "c920", "webcam": "webcam"}


def _flat(text: str) -> str:
    return "".join(ch for ch in text.lower() if ch.isalnum())


def linux_camera_for_spec(spec: str, cameras: list) -> object:
    """The one `V4lCamera` a `--cameras` spec names, or `ValueError` saying why not.

    Accepted spellings, the same three the session takes on macOS:
      * ``2``                 — a raw OpenCV index, matched against the by-id device node
      * ``c920``              — a model word (aliases above), which must match exactly one camera
      * ``d405:2603``         — model plus any unique SERIAL PREFIX; Julien dictates commands,
                                so a speakable prefix has to work ([FINDINGS §71.5](../../../docs/FINDINGS.md))

    ⛔ Refuses on zero matches AND on more than one. Never picks the first: with two identical
    D405s attached, "pick the first" is a 50% chance of recording the wrong arm's view under
    the right name, which is this stack's signature failure mode ([FINDINGS §0](../../../docs/FINDINGS.md)).
    """
    spec = spec.strip()
    if not cameras:
        raise ValueError("no cameras are listed in /dev/v4l/by-id — is anything plugged in? "
                         "`uv run checks/check_platform.py` prints the raw listing.")
    if spec.isdigit():
        want = int(spec)
        hit = [c for c in cameras if c.index == want]
        if not hit:
            listing = "\n".join(f"    {c.index}: {c.model} {c.serial}" for c in cameras)
            raise ValueError(f"no camera is at index {want}. Present:\n{listing}")
        return hit[0]

    model_part, _, serial_prefix = spec.partition(":")
    want_model = _flat(LINUX_ALIASES.get(_flat(model_part), model_part))
    matches = [c for c in cameras if want_model in _flat(c.model)]
    if serial_prefix:
        matches = [c for c in matches if c.serial.startswith(serial_prefix.strip())]
    if not matches:
        listing = "\n".join(f"    {c.model}  serial {c.serial or '(none reported)'}  "
                            f"index {c.index}" for c in cameras)
        raise ValueError(f"no camera matches {spec!r}. Present:\n{listing}")
    if len(matches) > 1:
        listing = "\n".join(f"    {c.model}  serial {c.serial or '(none reported)'}  "
                            f"index {c.index}" for c in matches)
        raise ValueError(f"{spec!r} matches {len(matches)} cameras:\n{listing}\n"
                         "  Add more of the serial to pick one.")
    return matches[0]
