"""Which machine is this, and how does IT name the devices? One module, both answers.

⭐⭐ WHY THIS EXISTS (ROADMAP §8.2 item 49, [FINDINGS §74.0](../../docs/FINDINGS.md)). The whole walkthrough ran on macOS, which the vendor SDK does not support, and Julien's team runs the real station on a Linux PC. Every device-identity trick this repo proved is therefore written twice: once for macOS (AVFoundation uniqueIDs, ioreg, gs_usb over libusb) and once for Linux (sysfs, `/dev/v4l/by-id`, SocketCAN). This module is the ONLY place that branches on the operating system, so the rest of the code keeps asking the same question and never learns where it is running.

⭐ THE GOOD NEWS, and it reshapes the camera problem: **on Linux the identity questions this repo spent days on are answered by the kernel.** `/dev/v4l/by-id/` names every camera by model AND USB serial and symlinks it straight to `/dev/videoN`, which is exactly OpenCV's index — so serial → index needs no hint file, no mode probe and no lens-covering ([FINDINGS §70.15](../../docs/FINDINGS.md) was the macOS workaround for a problem Linux does not have). Same for CAN: a candleLight adapter is a SocketCAN interface (`can0`), and sysfs carries its USB serial, so arm → interface resolves by serial with no index guessing.

⛔ HONESTY ABOUT WHAT IS PROVEN HERE, because this repo's own rule is to never present an assumption as a measurement. The parsers below are tested against fixtures **written from the documented sysfs and by-id formats, NOT captured from Julien's PC** — no Linux machine has been reached yet. So:
  • the tests prove the parsers do what their author intended;
  • `checks/check_platform.py` prints the RAW text beside the parse, so the FIRST run on the real machine either confirms the format or shows exactly how it differs;
  • every fixture in `tests/test_platform.py` is labelled as hand-written.
**Until that first run, treat Linux device naming as designed-and-unverified.** [FINDINGS §74.0](../../docs/FINDINGS.md) carries the same warning where it will be read.
"""

from __future__ import annotations

import re
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path

__all__ = [
    "IS_MACOS", "IS_LINUX", "platform_name", "platform_note",
    "CanLink", "parse_can_links", "read_can_links",
    "V4lCamera", "parse_v4l_by_id", "read_v4l_cameras", "udev_capture_nodes",
    "camera_permission_note",
]

IS_MACOS = sys.platform == "darwin"
IS_LINUX = sys.platform.startswith("linux")

#: ⭐ 1 Mbit/s, the bitrate every YAM motor on this rig expects (`yam/can.py`). On Linux the interface carries the bitrate itself and must be brought up with it BEFORE any Python touches the bus, which is a `sudo` action and therefore the operator's: `sudo ip link set can0 up type can bitrate 1000000`.
CAN_BITRATE = 1_000_000

#: ARPHRD_CAN. `/sys/class/net/<iface>/type` holds this for a CAN interface and nothing else does, so it is how a CAN link is told from an ethernet or wifi one without parsing names.
ARPHRD_CAN = 280


def platform_name() -> str:
    """`"macOS"`, `"Linux"`, or the raw `sys.platform` for anything else."""
    return "macOS" if IS_MACOS else "Linux" if IS_LINUX else sys.platform


def platform_note() -> str:
    """One line saying which device-naming world this process is in."""
    if IS_MACOS:
        return ("macOS: CAN over gs_usb/libusb (adapter INDEX, verified by serial after "
                "opening); cameras via AVFoundation uniqueIDs plus a confirmed index hint")
    if IS_LINUX:
        return ("Linux: CAN over SocketCAN (interface name resolved from the USB serial in "
                "sysfs); cameras via /dev/v4l/by-id, whose names carry model and serial")
    return f"{sys.platform}: unsupported — no device naming is implemented for it"


def camera_permission_note() -> str:
    """Who may open a camera here. ⭐ The answer differs by OS and it changes who does the work.

    macOS grants camera CAPTURE per application, so the permission Julien granted covers his
    terminal and can never cover an agent's shell ([FINDINGS §61.3](../../docs/FINDINGS.md)) —
    which is why every camera measurement in this repo is a command handed to him. **Linux has
    no such gate**: membership of the `video` group is enough, so on the Linux PC an agent can
    open cameras itself and the whole hand-it-over dance disappears.
    """
    if IS_MACOS:
        return ("macOS grants camera capture PER APP, so only the operator's own terminal can "
                "open a camera. An agent shell can never be granted it (FINDINGS §61.3).")
    if IS_LINUX:
        return ("Linux has no per-app camera gate — anything in the `video` group may open "
                "/dev/video*. An agent CAN run cameras here, unlike on the Mac.")
    return "unknown platform: assume an agent may not open a camera."


# ----------------------------------------------------------------- CAN, on Linux ----

@dataclass(frozen=True)
class CanLink:
    """One SocketCAN interface and the USB adapter behind it.

    `serial` is the adapter's USB serial number, which is what `yam/can.py::ARM_SERIALS` keys arms by, so `interface` can be resolved for an arm without ever trusting an enumeration order. `state`/`bitrate` come from the link itself: an interface that exists but is DOWN cannot carry a frame, and this repo has already paid once for treating presence as readiness ([FINDINGS §33.0](../../docs/FINDINGS.md)).
    """

    interface: str            # "can0"
    serial: str               # the USB serial of the adapter behind it, "" if unreadable
    state: str                # "UP" / "DOWN" / "?" — from `ip link`
    bitrate: int | None       # as configured on the link, None if unset or unreadable


#: `ip -details link show` prints the state on the header line and the bitrate on a later
#: one. Two small patterns beat one big one: a kernel that changes the layout then breaks
#: one field instead of the whole parse.
_IP_HEADER = re.compile(r"^\d+:\s+(?P<iface>[\w.-]+):\s+<(?P<flags>[^>]*)>")
_IP_BITRATE = re.compile(r"\bbitrate\s+(?P<bitrate>\d+)")


def parse_can_links(ip_output: str, serials: dict[str, str] | None = None) -> list[CanLink]:
    """Parse `ip -details link show type can` into `CanLink`s. Pure, so it is testable anywhere.

    `serials` maps interface name → USB serial (read from sysfs by `read_can_links`, injected
    by the tests). A missing entry becomes `""` rather than a guess: a wrong serial here would
    drive the wrong arm, which is the one mistake this whole scheme exists to prevent.
    """
    serials = serials or {}
    links: list[CanLink] = []
    current: dict | None = None
    for line in ip_output.splitlines():
        header = _IP_HEADER.match(line.strip()) or _IP_HEADER.match(line)
        if header:
            if current:
                links.append(_finish_can_link(current, serials))
            flags = header.group("flags").split(",")
            current = {"iface": header.group("iface"),
                       "state": "UP" if "UP" in flags else "DOWN",
                       "bitrate": None}
            continue
        if current is None:
            continue
        rate = _IP_BITRATE.search(line)
        if rate:
            current["bitrate"] = int(rate.group("bitrate"))
    if current:
        links.append(_finish_can_link(current, serials))
    return links


def _finish_can_link(found: dict, serials: dict[str, str]) -> CanLink:
    return CanLink(interface=found["iface"], serial=serials.get(found["iface"], ""),
                   state=found["state"], bitrate=found["bitrate"])


def sysfs_can_serial(interface: str, root: Path | None = None) -> str:
    """The USB serial of the adapter behind a SocketCAN interface, or `""`.

    The path walk is the standard one: `/sys/class/net/<iface>/device` is the USB INTERFACE
    directory (`1-3:1.0`), and the `serial` attribute lives one level up on the USB DEVICE
    (`1-3`). Both `..` hops are needed and `resolve()` follows the symlink out of `/sys/class`
    into the real device tree first, or the parent would be `/sys/class/net`.
    """
    base = Path(root or "/sys/class/net") / interface / "device"
    try:
        usb_iface = base.resolve(strict=True)
    except (OSError, RuntimeError):
        return ""
    for candidate in (usb_iface.parent / "serial", usb_iface / "serial"):
        try:
            return candidate.read_text().strip()
        except OSError:
            continue
    return ""


def read_can_links() -> list[CanLink]:
    """Every CAN interface on THIS machine, with its adapter serial. Linux only; `[]` elsewhere.

    ⚠️ A dated reading the moment it returns — the same rule as every other device listing in
    this repo. Ask again rather than caching.
    """
    if not IS_LINUX:
        return []
    try:
        out = subprocess.run(["ip", "-details", "link", "show", "type", "can"],
                             capture_output=True, text=True, check=False)
    except OSError:
        return []
    names = [m.group("iface") for m in
             (_IP_HEADER.match(line) for line in out.stdout.splitlines()) if m]
    serials = {name: sysfs_can_serial(name) for name in names}
    return parse_can_links(out.stdout, serials)


# ------------------------------------------------------------- cameras, on Linux ----

@dataclass(frozen=True)
class V4lCamera:
    """One camera as Linux names it. ⭐ `index` is directly OpenCV's index, no probing.

    The by-id symlink resolves to `/dev/videoN`, and OpenCV's V4L2 backend opens `VideoCapture(N)` as exactly that node — so on Linux the serial→index join that cost this repo days on macOS ([FINDINGS §70.15](../../docs/FINDINGS.md), [§22](../../docs/FINDINGS.md)) is a symlink read.

    ⚠️ `serial` is `""` for a camera that does not report one. The C920 is exactly that case ([FINDINGS §70.6](../../docs/FINDINGS.md)), so it is still identified by model, on both platforms.
    """

    device: str               # "/dev/video0"
    index: int                # what OpenCV's VideoCapture(index) opens — the FIRST node
    model: str                # "Intel_R__RealSense_TM__Depth_Camera_405"
    serial: str               # "255323071773", or "" when the camera reports none
    by_id: str                # the full by-id link name, kept for the report and for auditing
    #: ⛔⭐ EVERY video node this one physical camera exposes, ascending. MEASURED on the
    #: station 2026-08-19: the C920 shows 2 and **a D405 shows SIX** (colour, depth, infrared
    #: and metadata streams all live under one USB device). `index` is the first of them,
    #: which is a CHOICE and not a measurement — on this rig's D405s nobody has yet confirmed
    #: which node carries COLOUR on Linux. Carrying the whole list is what makes that
    #: question askable instead of invisible ([FINDINGS §75.5](../../docs/FINDINGS.md)).
    nodes: tuple[int, ...] = ()
    #: ⛔⭐ The subset of `nodes` that can actually CAPTURE, from udev's own
    #: `ID_V4L_CAPABILITIES` (readable with no root and no `video` group). MEASURED on the
    #: station: a D405's six nodes are three capture streams each paired with a METADATA
    #: node, and a metadata node **opens successfully and delivers nothing** — the silent
    #: wrong answer this repo exists to refuse. Empty when udev could not be asked, in which
    #: case `index` falls back to the first node and says so
    #: ([FINDINGS §75.6](../../docs/FINDINGS.md)).
    capture_nodes: tuple[int, ...] = ()


#: A `/dev/v4l/by-id` entry: `usb-<vendor>_<model>_<serial>-video-index<N>`. The serial is the
#: last underscore-separated field of the middle part, and it is ABSENT on a camera that does
#: not report one — hence the optional group rather than a required one.
_BY_ID = re.compile(r"^usb-(?P<body>.+?)-video-index(?P<node>\d+)$")


def parse_v4l_by_id(listing: dict[str, str],
                    capture_nodes: set[int] | None = None) -> list[V4lCamera]:
    """Turn a `{by-id name: symlink target}` mapping into cameras. Pure and order-stable.

    ⛔⭐ `capture_nodes` is the set of `/dev/videoN` numbers udev marks as capture-capable. When it is given, `index` is the first node **in that set** rather than simply the first node — because a UVC camera's other nodes are metadata streams that open successfully and deliver nothing, which is the silent-wrong-answer trap of exactly this repo's favourite kind. When it is omitted (udev unavailable), `index` falls back to the first node and `capture_nodes` stays empty so a caller can tell the two situations apart.
    """
    # ⭐ Group by DEVICE first (the body of the by-id name), because one physical camera
    # publishes several nodes and the old version silently kept only the first — which
    # threw away the fact that a D405 has six of them.
    per_device: dict[str, list[tuple[int, str, str]]] = {}
    for name in sorted(listing):
        match = _BY_ID.match(name)
        if not match:
            continue
        node = Path(listing[name]).name              # "video0" from "../../video0"
        digits = "".join(ch for ch in node if ch.isdigit())
        if not node.startswith("video") or not digits:
            continue
        per_device.setdefault(match.group("body"), []).append((int(digits), node, name))

    cameras: list[V4lCamera] = []
    for body, found in per_device.items():
        found.sort()
        mine = tuple(i for i, _, _ in found)
        capture = tuple(i for i in mine if capture_nodes and i in capture_nodes)
        # ⭐ Prefer a capture node; fall back to the first node only when udev said nothing.
        chosen = capture[0] if capture else found[0][0]
        first_index, first_node, first_name = next(
            entry for entry in found if entry[0] == chosen)
        # The serial is the trailing field when it looks like one: cameras that report no
        # serial simply end with the model, and inventing a serial from a model word would
        # be the wrong-identity failure this module exists to prevent.
        model, _, tail = body.rpartition("_")
        serial = tail if model and _plausible_serial(tail) else ""
        cameras.append(V4lCamera(device=f"/dev/{first_node}", index=first_index,
                                 model=(model or body) if serial else body,
                                 serial=serial, by_id=first_name,
                                 nodes=mine, capture_nodes=capture))
    return sorted(cameras, key=lambda c: c.index)


def _plausible_serial(text: str) -> bool:
    """A trailing by-id field that is a serial rather than part of the model name.

    Serials on this rig's cameras are long digit strings (`255323071773`). Requiring digits
    and a length of 6+ keeps model words (`C920`, `405`) from being read as serials, and the
    consequence of being wrong here is only a missing serial, never a wrong one.
    """
    return len(text) >= 6 and any(ch.isdigit() for ch in text) and "_" not in text


def udev_capture_nodes() -> set[int]:
    """Which `/dev/videoN` numbers udev marks as CAPTURE-capable. `set()` when it cannot say.

    ⭐ Read from the udev database with `udevadm info`, which needs **no root and no `video`
    group** — so this question is answerable before any permission is granted, which is
    exactly when it is needed. The property is `ID_V4L_CAPABILITIES`: `:capture:` for a real
    stream, a bare `:` for the metadata node that accompanies it.
    """
    if not IS_LINUX:
        return set()
    found: set[int] = set()
    for node in sorted(Path("/dev").glob("video*")):
        digits = "".join(ch for ch in node.name if ch.isdigit())
        if not digits:
            continue
        try:
            out = subprocess.run(["udevadm", "info", "--query=property",
                                  f"--name={node}"],
                                 capture_output=True, text=True, check=False, timeout=10)
        except (OSError, subprocess.SubprocessError):
            return set()
        for line in out.stdout.splitlines():
            if line.startswith("ID_V4L_CAPABILITIES=") and "capture" in line:
                found.add(int(digits))
    return found


def read_v4l_cameras(root: Path | None = None) -> list[V4lCamera]:
    """Every camera on THIS machine, from `/dev/v4l/by-id`. Linux only; `[]` elsewhere."""
    if not IS_LINUX and root is None:
        return []
    by_id_dir = Path(root or "/dev/v4l/by-id")
    try:
        entries = {p.name: str(p.readlink()) for p in by_id_dir.iterdir() if p.is_symlink()}
    except OSError:
        return []
    return parse_v4l_by_id(entries, udev_capture_nodes() or None)
