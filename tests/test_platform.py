#!/usr/bin/env python3
"""Tests for the platform layer — the Linux device-naming parsers.

    uv run tests/test_platform.py

⛔⭐ READ THIS BEFORE TRUSTING A GREEN RUN. Every fixture below is **hand-written from the documented `ip`, sysfs and `/dev/v4l/by-id` formats. None of it was captured from Julien's Linux PC**, because no Linux machine has been reached yet. So these tests prove the parsers do what their author intended; they do NOT prove the real machine prints this. `checks/check_platform.py` prints the RAW text beside its parse for exactly that reason: the first run on the real PC either confirms these formats or shows precisely how they differ ([FINDINGS §74.0](../docs/FINDINGS.md)).

⭐ The macOS-side fixtures ARE real captures and live in `tests/test_camera_identity.py`. Keeping the two apart is deliberate: a reader must never have to guess which fixtures are measurements.
"""

from __future__ import annotations

import sys
import tempfile
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent

from yam.platform import (  # noqa: E402
    CAN_BITRATE,
    V4lCamera,
    camera_permission_note,
    parse_can_links,
    parse_v4l_by_id,
    platform_name,
    read_v4l_cameras,
    sysfs_can_serial,
)

#: ⚠️ HAND-WRITTEN, not captured. Two candleLight adapters as `ip -details link show type can`
#: is documented to print them: one brought up at 1 Mbit/s, one still down.
IP_FIXTURE = """\
3: can0: <NOARP,UP,LOWER_UP,ECHO> mtu 16 qdisc pfifo_fast state UP mode DEFAULT group default qlen 10
    link/can  promiscuity 0 minmtu 0 maxmtu 0
    can state ERROR-ACTIVE (berr-counter tx 0 rx 0) restart-ms 0
          bitrate 1000000 sample-point 0.750
          tq 12 prop-seg 29 phase-seg1 30 phase-seg2 20 sjw 1
4: can1: <NOARP,ECHO> mtu 16 qdisc pfifo_fast state DOWN mode DEFAULT group default qlen 10
    link/can  promiscuity 0 minmtu 0 maxmtu 0
    can <BERR-REPORTING> state STOPPED restart-ms 0
"""

#: ⚠️ HAND-WRITTEN. This rig's three cameras as `/dev/v4l/by-id` is documented to name them:
#: two D405s with their REAL serials (from FINDINGS §70.6, which is a real capture) and the
#: C920, whose USB serial is genuinely empty — so its entry carries no serial field at all.
BY_ID_FIXTURE = {
    "usb-Intel_R__RealSense_TM__Depth_Camera_405_255323071773-video-index0": "../../video2",
    "usb-Intel_R__RealSense_TM__Depth_Camera_405_255323071773-video-index1": "../../video3",
    "usb-Intel_R__RealSense_TM__Depth_Camera_405_260323072846-video-index0": "../../video4",
    "usb-046d_HD_Pro_Webcam_C920-video-index0": "../../video0",
    "usb-046d_HD_Pro_Webcam_C920-video-index1": "../../video1",
}


def test_can_links_carry_state_and_bitrate_apart() -> None:
    links = parse_can_links(IP_FIXTURE, {"can0": "2081337C594E5018"})
    assert [l.interface for l in links] == ["can0", "can1"]
    up, down = links
    assert up.state == "UP" and up.bitrate == CAN_BITRATE
    assert up.serial == "2081337C594E5018", "the serial is what an arm is resolved by"
    assert down.state == "DOWN" and down.bitrate is None, \
        "an interface that exists but is DOWN cannot carry a frame, and must read that way"
    assert down.serial == "", "an unknown serial stays empty — never guessed from position"


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
    assert cams["255323071773"].index == 2 and cams["255323071773"].device == "/dev/video2"
    assert cams["260323072846"].index == 4
    assert "RealSense" in cams["255323071773"].model
    # ⭐ This is the whole Linux advantage: two IDENTICAL cameras separated by serial, with
    # their OpenCV index read from a symlink — no hint file, no mode probe, no lens covering.
    assert cams["255323071773"].index != cams["260323072846"].index


def test_the_c920s_absent_serial_stays_absent() -> None:
    c920 = [c for c in parse_v4l_by_id(BY_ID_FIXTURE) if "C920" in c.model][0]
    assert c920.serial == "", "the C920 reports no USB serial (FINDINGS §70.6) — do not invent one"
    assert c920.index == 0


def test_only_the_capture_node_is_returned() -> None:
    cams = parse_v4l_by_id(BY_ID_FIXTURE)
    assert len(cams) == 3, f"5 by-id entries, 3 cameras — index1 nodes are metadata: {cams}"
    assert all(c.by_id.endswith("index0") for c in cams)


def test_read_v4l_cameras_walks_a_real_directory() -> None:
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
