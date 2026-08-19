#!/usr/bin/env python3
"""Tests for the platform layer — the Linux device-naming parsers.

    uv run tests/test_platform.py

✅⭐ **THESE FIXTURES ARE REAL CAPTURES, taken from the station `lavita@10.64.9.60` on 2026-08-19** ([FINDINGS §75.5](../docs/FINDINGS.md)). They were hand-written from documented formats for a few hours, and this file said so; the moment the machine was reachable with hardware attached they were replaced by its actual output, verbatim. **That upgrade is the point** — a fixture written from documentation tests its author's belief, and a fixture cut from the real machine tests reality.

⭐ What the capture corrected: a D405 publishes **SIX** video nodes on Linux (colour, depth, infrared and metadata under one USB device), not one, and the vendor and product strings BOTH appear in the by-id name so the model text is doubled. The first version of the parser kept only `-video-index0` entries and would have silently discarded the fact that the other five exist.
"""

from __future__ import annotations

import sys
import tempfile
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent

from yam.platform import (  # noqa: E402
    CAN_BITRATE,
    classify_v4l_node,
    parse_sysfs_usb,
    with_colour_nodes,
    V4lCamera,
    camera_permission_note,
    parse_can_links,
    parse_v4l_by_id,
    platform_name,
    read_v4l_cameras,
    sysfs_can_serial,
)

#: ✅ REAL CAPTURE from the station, 2026-08-19: both candleLight adapters attached and both
#: still DOWN, which is their state after every boot until someone runs the `ip link set`
#: command. ⭐ Kept exactly as `ip -details link show type can` printed it, tabs included.
IP_FIXTURE = """\
7: can0: <NOARP,ECHO> mtu 16 qdisc noop state DOWN mode DEFAULT group default qlen 10
    link/can  promiscuity 0  allmulti 0 minmtu 16 maxmtu 16 
    can state STOPPED restart-ms 0 
	  gs_usb: tseg1 1..256 tseg2 1..128 sjw 1..128 brp 1..512 brp_inc 1
	  clock 160000000 numtxqueues 1 numrxqueues 1 parentbus usb parentdev 1-4:1.0 
8: can1: <NOARP,ECHO> mtu 16 qdisc noop state DOWN mode DEFAULT group default qlen 10
    link/can  promiscuity 0  allmulti 0 minmtu 16 maxmtu 16 
    can state STOPPED restart-ms 0 
	  gs_usb: tseg1 1..256 tseg2 1..128 sjw 1..128 brp 1..512 brp_inc 1
	  clock 160000000 numtxqueues 1 numrxqueues 1 parentbus usb parentdev 3-3:1.0 
"""

#: ✅ REAL CAPTURE, same machine and moment: an UP interface has a `bitrate` line, which the
#: captured pair above does not, so this one line is kept separately to exercise that branch.
#: ⚠️ Synthesised from the documented `ip` output for an UP link — labelled, because no
#: interface on the station has been brought up yet.
IP_UP_FIXTURE = """\
7: can0: <NOARP,UP,LOWER_UP,ECHO> mtu 16 qdisc pfifo_fast state UP mode DEFAULT group default qlen 10
    link/can  promiscuity 0  allmulti 0 minmtu 16 maxmtu 16 
    can state ERROR-ACTIVE restart-ms 0 
          bitrate 1000000 sample-point 0.750
"""

#: ✅ REAL CAPTURE from the station, 2026-08-19, verbatim: the C920 (2 nodes, NO serial in the
#: name because the device reports none) and ONE D405 (**6 nodes**, its serial at the end, and
#: the vendor and product strings both present so the model text appears twice). The second
#: D405 was still on the Mac's hub at capture time. ⭐ A second D405 line is added below to
#: exercise the two-identical-cameras case, built by substituting the other real serial.
_D405 = "usb-Intel_R__RealSense_TM__Depth_Camera_405_Intel_R__RealSense_TM__Depth_Camera_405"
BY_ID_FIXTURE = {
    "usb-046d_HD_Pro_Webcam_C920-video-index0": "../../video0",
    "usb-046d_HD_Pro_Webcam_C920-video-index1": "../../video1",
    **{f"{_D405}_260323072846-video-index{i}": f"../../video{2 + i}" for i in range(6)},
    # ⚠️ Constructed, not captured: the same real name with this rig's OTHER real serial, so
    # the "two identical D405s" case is covered. Its node numbers continue the sequence.
    **{f"{_D405}_255323071773-video-index{i}": f"../../video{8 + i}" for i in range(6)},
}


def test_the_real_capture_reads_as_two_down_interfaces_with_their_serials() -> None:
    # The serials are the station's real ones: can0 is arm G, can1 is arm B (FINDINGS §75.5).
    links = parse_can_links(IP_FIXTURE, {"can0": "20593383594E5018",
                                         "can1": "2081337C594E5018"})
    assert [l.interface for l in links] == ["can0", "can1"]
    assert all(l.state == "DOWN" for l in links), \
        "both adapters are DOWN after every boot until `ip link set` runs — presence is not " \
        "readiness, and a DOWN interface cannot carry a frame"
    assert all(l.bitrate is None for l in links), "a DOWN link has no bitrate configured"
    assert links[0].serial == "20593383594E5018" and links[1].serial == "2081337C594E5018"


def test_an_up_interface_reports_its_bitrate() -> None:
    links = parse_can_links(IP_UP_FIXTURE, {"can0": "20593383594E5018"})
    assert len(links) == 1
    assert links[0].state == "UP" and links[0].bitrate == CAN_BITRATE


def test_a_missing_serial_is_empty_rather_than_wrong() -> None:
    links = parse_can_links(IP_FIXTURE, {})
    assert all(l.serial == "" for l in links), \
        "a wrong serial here would drive the WRONG ARM; empty is the only safe unknown"


def test_no_can_interfaces_parses_to_nothing() -> None:
    assert parse_can_links("", {}) == []


def test_sysfs_serial_reads_the_usb_device_one_level_above_the_interface() -> None:
    with tempfile.TemporaryDirectory() as d:
        root = Path(d)
        # The documented layout: /sys/class/net/can0/device -> the USB INTERFACE (1-3:1.0),
        # whose PARENT (1-3) holds `serial`.
        usb_device = root / "devices" / "1-3"
        usb_iface = usb_device / "1-3:1.0"
        usb_iface.mkdir(parents=True)
        (usb_device / "serial").write_text("2081337C594E5018\n")
        net = root / "class" / "net" / "can0"
        net.mkdir(parents=True)
        (net / "device").symlink_to(usb_iface)
        assert sysfs_can_serial("can0", root=root / "class" / "net") == "2081337C594E5018"


def test_a_missing_sysfs_path_returns_empty_and_never_raises() -> None:
    with tempfile.TemporaryDirectory() as d:
        assert sysfs_can_serial("can9", root=Path(d)) == ""


def test_by_id_gives_serial_model_and_the_opencv_index_directly() -> None:
    cams = {c.serial or c.model: c for c in parse_v4l_by_id(BY_ID_FIXTURE)}
    assert set(cams) == {"255323071773", "260323072846", "046d_HD_Pro_Webcam_C920"}, \
        f"expected two serialled D405s and the serial-less C920, got {sorted(cams)}"
    assert cams["260323072846"].index == 2 and cams["260323072846"].device == "/dev/video2"
    assert cams["255323071773"].index == 8
    assert "RealSense" in cams["255323071773"].model
    # ⭐ This is the whole Linux advantage: two IDENTICAL cameras separated by serial, with
    # their OpenCV index read from a symlink — no hint file, no mode probe, no lens covering.
    assert cams["255323071773"].index != cams["260323072846"].index


def test_the_c920s_absent_serial_stays_absent() -> None:
    c920 = [c for c in parse_v4l_by_id(BY_ID_FIXTURE) if "C920" in c.model][0]
    assert c920.serial == "", "the C920 reports no USB serial (FINDINGS §70.6) — do not invent one"
    assert c920.index == 0


def test_metadata_nodes_are_never_chosen_when_udev_can_say() -> None:
    """⛔ MEASURED on the station (FINDINGS §75.6): a D405's six nodes are three capture
    streams each paired with a METADATA node, and a metadata node opens successfully and
    delivers nothing. udev knows which is which, with no root and no `video` group, so the
    chosen index must come from that set."""
    capture = {0, 2, 4, 6}
    cams = {c.serial or "c920": c for c in parse_v4l_by_id(BY_ID_FIXTURE, capture)}
    d405 = cams["260323072846"]
    assert d405.capture_nodes == (2, 4, 6), "three capture streams: depth, infrared, colour"
    assert d405.index == 2 and d405.index in d405.capture_nodes
    assert cams["c920"].capture_nodes == (0,) and cams["c920"].index == 0
    # Without udev the fallback is the first node, and capture_nodes stays EMPTY so a caller
    # can tell "udev said this is capture" from "nobody asked".
    blind = {c.serial or "c920": c for c in parse_v4l_by_id(BY_ID_FIXTURE)}
    assert blind["260323072846"].capture_nodes == ()
    assert blind["260323072846"].index == 2


def test_a_camera_whose_only_capture_node_is_not_the_first_is_still_found() -> None:
    # The guard that matters: if udev marks only node 4 as capture, node 2 must NOT be
    # chosen, however first it comes.
    cams = {c.serial or "c920": c for c in parse_v4l_by_id(BY_ID_FIXTURE, {4})}
    assert cams["260323072846"].index == 4, \
        "the chosen index must come from the capture set, not from the node order"


def test_one_camera_per_device_with_every_node_it_publishes() -> None:
    """⛔ The capture corrected this: a D405 publishes SIX nodes, not one. Keeping the whole
    list is what makes "which node is colour?" an askable question rather than a silent
    assumption (FINDINGS §75.5)."""
    cams = {c.serial or "c920": c for c in parse_v4l_by_id(BY_ID_FIXTURE)}
    assert len(cams) == 3, f"14 by-id entries, 3 physical cameras: {sorted(cams)}"
    assert cams["c920"].nodes == (0, 1), "the C920 publishes two nodes"
    assert cams["260323072846"].nodes == (2, 3, 4, 5, 6, 7), \
        "a D405 publishes SIX — colour, depth, infrared and metadata under one device"
    assert all(c.index == c.nodes[0] for c in cams.values()), \
        "index is the FIRST node, and it is a choice: nobody has confirmed which node " \
        "carries colour on Linux"
    assert all(c.by_id.endswith("index0") for c in cams.values())


def test_read_v4l_cameras_walks_a_real_directory() -> None:  # noqa: D103
    # Proves the live reader against the same shape, using a real filesystem rather than a
    # mock — a reader that only works against a dict would not have been exercised at all.
    with tempfile.TemporaryDirectory() as d:
        root = Path(d)
        (root / "dev").mkdir()
        for name, target in BY_ID_FIXTURE.items():
            node = root / "dev" / Path(target).name
            node.write_text("")
            (root / name).symlink_to(node)
        cams = read_v4l_cameras(root=root)
        assert len(cams) == 3 and isinstance(cams[0], V4lCamera)


#: ✅ REAL format lists, measured on the station 2026-08-19 with BOTH `v4l2-ctl --list-formats`
#: and this module's own ioctl, which agreed exactly (FINDINGS §75.7). This is the map that
#: turns "the first capture node" into "the colour one".
REAL_FORMATS = {
    0: ("YUYV", "MJPG"),                            # C920, colour
    2: ("Z16 ",),                                   # D405 depth
    4: ("GREY", "UYVY", "Y8I ", "Y12I"),            # D405 infrared
    6: ("YUYV",),                                   # D405 colour
}


def test_the_measured_format_lists_classify_correctly() -> None:
    assert classify_v4l_node(REAL_FORMATS[0]) == "colour"
    assert classify_v4l_node(REAL_FORMATS[2]) == "depth"
    assert classify_v4l_node(REAL_FORMATS[6]) == "colour"
    # ⛔ THE ORDERING TRAP: the infrared node ALSO offers UYVY, which is a colour format. If
    # colour were tested first this node would classify as colour and a dataset would carry
    # greyscale infrared as if it were photographs (FINDINGS §75.7).
    assert classify_v4l_node(REAL_FORMATS[4]) == "infrared", \
        "the infrared node offers UYVY too — greyscale formats beside it are what identify it"
    assert classify_v4l_node(()) == "unknown", "no formats read means unknown, never a guess"


def test_the_colour_node_is_chosen_over_the_first_capture_node() -> None:
    """⛔ The defect this prevents: a D405's first capture node is DEPTH. Opening it and
    storing the frames as photographs is the silent wrong answer (FINDINGS §75.7)."""
    cams = parse_v4l_by_id(BY_ID_FIXTURE, {0, 2, 4, 6})
    before = {c.serial or "c920": c for c in cams}
    assert before["260323072846"].index == 2, "the first capture node is the depth stream"

    after = {c.serial or "c920": c for c in with_colour_nodes(cams, REAL_FORMATS)}
    d405 = after["260323072846"]
    assert d405.index == 6 and d405.colour_node == 6
    assert d405.index_reason == "colour-format", "the report must be able to say WHY"
    assert after["c920"].index == 0 and after["c920"].colour_node == 0


def test_unreadable_formats_leave_the_index_alone_and_say_so() -> None:
    # No `video` group means every format list is empty. The index must NOT silently become
    # something else, and `index_reason` must not claim a measurement that did not happen.
    cams = parse_v4l_by_id(BY_ID_FIXTURE, {0, 2, 4, 6})
    after = {c.serial or "c920": c for c in with_colour_nodes(cams, {})}
    d405 = after["260323072846"]
    assert d405.index == 2 and d405.colour_node is None
    assert d405.index_reason == "first-capture-node"


#: ✅ REAL values read from the station's sysfs, 2026-08-19: both CAN adapters (whose serials
#: libusb could NOT read there, because the kernel driver owns them), the D405, and the C920
#: with its genuinely empty serial. Interface directories are included on purpose — the parser
#: must skip them or every device would appear several times.
SYSFS_USB = {
    "1-6": {"idVendor": "1d50", "idProduct": "606f", "serial": "20593383594E5018",
            "product": "canable gs_usb", "busnum": "1", "devnum": "6"},
    "1-6:1.0": {"bInterfaceNumber": "00"},
    "3-5": {"idVendor": "1d50", "idProduct": "606f", "serial": "2081337C594E5018",
            "product": "canable gs_usb", "busnum": "3", "devnum": "5"},
    "2-2": {"idVendor": "8086", "idProduct": "0b5b", "serial": "260323072846",
            "product": "Intel(R) RealSense(TM) Depth Camera 405", "busnum": "2",
            "devnum": "2"},
    "4-1": {"idVendor": "046d", "idProduct": "08e5", "serial": "",
            "product": "HD Pro Webcam C920", "busnum": "4", "devnum": "1"},
    "not-a-device": {"product": "a hub port with no idVendor"},
}


def test_sysfs_gives_the_serials_libusb_cannot_read_on_linux() -> None:
    """⛔ THE DEFECT THIS FIXES (FINDINGS §75.8): libusb must OPEN a device to read a string
    descriptor, and the kernel's gs_usb driver owns the CAN adapters on Linux — so every
    serial came back "?" and `check_rig` announced BOTH adapters missing while the motors
    were answering through them. sysfs publishes the descriptors as files."""
    devices = {d["serial"]: d for d in parse_sysfs_usb(SYSFS_USB)}
    assert "20593383594E5018" in devices and "2081337C594E5018" in devices, \
        "both adapter serials must be readable, since that is what an arm is resolved by"
    g = devices["20593383594E5018"]
    assert (g["vid"], g["pid"]) == (0x1D50, 0x606F), "vid/pid parse from hex"
    assert (g["bus"], g["addr"]) == (1, 6)
    assert devices["260323072846"]["product"].startswith("Intel")
    assert "" in devices, "the C920 reports an empty serial and that must survive as empty"
    assert len(parse_sysfs_usb(SYSFS_USB)) == 4, \
        "4 devices from 6 entries: the interface dir and the attribute-less one are skipped"


def test_the_platform_answers_are_specific_and_honest() -> None:
    assert platform_name() in {"macOS", "Linux"} or platform_name().startswith(sys.platform)
    note = camera_permission_note()
    assert ("PER APP" in note) if platform_name() == "macOS" else True
    assert ("video" in note) if platform_name() == "Linux" else True


def main() -> int:
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failed = 0
    for fn in tests:
        try:
            fn()
        except AssertionError as e:  # noqa: PERF203
            failed += 1
            print(f"✗ {fn.__name__}: {e}")
        else:
            print(f"✓ {fn.__name__}")
    print(f"\n{len(tests) - failed}/{len(tests)} passed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
