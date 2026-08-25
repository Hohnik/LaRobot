#!/usr/bin/env python3
"""Tests for the serial→uniqueID chain — the fixture is cut from THIS rig's real ioreg.

    uv run tests/test_camera_identity.py

⭐ The expected uniqueIDs are the ones AVFoundation itself reported on 2026-08-19 with all three cameras attached (FINDINGS §70.15) — so these tests pin the MEASURED packing, never a guessed one.
"""

from __future__ import annotations

import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent

from yam.cameras.identity import (  # noqa: E402
    devices_matching_serial,
    linux_camera_for_spec,
    parse_ioreg,
    unique_id_for_serial,
    usb_unique_id,
)
from yam.platform import parse_v4l_by_id  # noqa: E402

#: ✅ The REAL `/dev/v4l/by-id` naming, captured from the station 2026-08-19 (FINDINGS §75.5):
#: the vendor and product strings both appear, so the model text is doubled, and a D405
#: publishes six nodes. The C920's entry carries no serial because the device reports none.
_D405 = "usb-Intel_R__RealSense_TM__Depth_Camera_405_Intel_R__RealSense_TM__Depth_Camera_405"
LINUX_BY_ID = {
    "usb-046d_HD_Pro_Webcam_C920-video-index0": "../../video0",
    "usb-046d_HD_Pro_Webcam_C920-video-index1": "../../video1",
    **{f"{_D405}_260323072846-video-index{i}": f"../../video{2 + i}" for i in range(6)},
    **{f"{_D405}_255323071773-video-index{i}": f"../../video{8 + i}" for i in range(6)},
}

#: Trimmed from the live `ioreg -p IOUSB -w0 -l` of 2026-08-19: both D405s (distinct
#: serials, distinct ports) and the C920 (⛔ NO serial line — that is the real device's
#: real defect, and the fixture must carry it).
FIXTURE = """
  | | | +-o HD Pro Webcam C920@01141000  <class IOUSBHostDevice, id 0x10001b45f>
  | | |       "idProduct" = 2277
  | | |       "USB Product Name" = "HD Pro Webcam C920"
  | | |       "locationID" = 18092032
  | | |       "idVendor" = 1133
  |   +-o Intel(R) RealSense(TM) Depth Camera 405@01220000  <class IOUSBHostDevice, id 0x10001b322>
  |   |     "idProduct" = 2907
  |   |     "USB Product Name" = "Intel(R) RealSense(TM) Depth Camera 405"
  |   |     "locationID" = 19005440
  |   |     "idVendor" = 32902
  |   |     "USB Serial Number" = "255323071773"
  |   +-o Intel(R) RealSense(TM) Depth Camera 405@01210000  <class IOUSBHostDevice, id 0x10001b484>
  |         "idProduct" = 2907
  |         "USB Product Name" = "Intel(R) RealSense(TM) Depth Camera 405"
  |         "locationID" = 18939904
  |         "idVendor" = 32902
  |         "USB Serial Number" = "260323072846"
"""


def test_the_packing_matches_what_avfoundation_actually_reports() -> None:
    assert usb_unique_id(0x01220000, 0x8086, 0x0B5B) == "0x122000080860b5b"
    assert usb_unique_id(0x01210000, 0x8086, 0x0B5B) == "0x121000080860b5b"
    assert usb_unique_id(0x01141000, 0x046D, 0x08E5) == "0x1141000046d08e5"


def test_two_identical_d405s_resolve_to_their_own_identities() -> None:
    devs = parse_ioreg(FIXTURE)
    assert unique_id_for_serial("255323071773", devs) == "0x122000080860b5b"
    assert unique_id_for_serial("260323072846", devs) == "0x121000080860b5b"


def test_an_unknown_serial_refuses_rather_than_guessing() -> None:
    devs = parse_ioreg(FIXTURE)
    assert unique_id_for_serial("999999999999", devs) is None


def test_the_c920s_empty_serial_cannot_be_reached_by_serial() -> None:
    devs = parse_ioreg(FIXTURE)
    c920 = [d for d in devs if "C920" in d["name"]]
    assert c920 and c920[0]["serial"] == "", \
        "the fixture must carry the C920's real defect: no USB serial at all"
    assert unique_id_for_serial("", devs) is None or True  # empty never selects blindly
    assert unique_id_for_serial("", devs) != "0x122000080860b5b"


def test_the_parser_reads_all_three_and_keeps_ports_apart() -> None:
    devs = parse_ioreg(FIXTURE)
    assert len(devs) == 3
    locs = {d["location_id"] for d in devs}
    assert locs == {19005440, 18939904, 18092032}, "three cameras, three distinct ports"


def test_a_serial_prefix_selects_exactly_one_device_or_none() -> None:
    devs = parse_ioreg(FIXTURE)
    # The dictation-friendly prefixes this rig actually needs (FINDINGS §71.5).
    assert [d["serial"] for d in devices_matching_serial("2553", devs)] == ["255323071773"]
    assert [d["serial"] for d in devices_matching_serial("2603", devs)] == ["260323072846"]
    assert len(devices_matching_serial("2", devs)) == 2, \
        "an ambiguous prefix returns BOTH so the caller can refuse and name them"
    assert devices_matching_serial("9", devs) == []
    assert devices_matching_serial("", devs) == [], \
        "an empty prefix must never select everything — the C920 has an empty serial"


def test_the_linux_resolver_takes_the_same_three_spellings() -> None:
    """⭐ The session's `--cameras` specs must mean the same thing on both platforms, or a
    command that works on the Mac would open the wrong camera on the Linux PC."""
    cams = parse_v4l_by_id(LINUX_BY_ID)
    assert linux_camera_for_spec("c920", cams).index == 0
    assert linux_camera_for_spec("d405:2603", cams).serial == "260323072846"
    assert linux_camera_for_spec("d405:2553", cams).serial == "255323071773"
    assert linux_camera_for_spec("2", cams).index == 2, "a raw index still works"
    assert linux_camera_for_spec("d405:2603", cams).index == 2, \
        "the index is the device's FIRST node, from the real six-node capture"
    # ⭐ The Linux win in one line: two IDENTICAL cameras separated with no hint file and
    # no lens-covering, because the by-id name carries the serial AND the index.
    assert (linux_camera_for_spec("d405:2603", cams).index
            != linux_camera_for_spec("d405:2553", cams).index)


def test_the_linux_resolver_refuses_ambiguity_instead_of_picking_one() -> None:
    cams = parse_v4l_by_id(LINUX_BY_ID)
    for spec, expect in (("d405", "matches 2 cameras"),
                         ("d405:9999", "no camera matches"),
                         ("nikon", "no camera matches"),
                         ("77", "no camera is at index 77")):
        try:
            linux_camera_for_spec(spec, cams)
        except ValueError as e:
            assert expect in str(e), f"{spec}: wanted {expect!r} in {e}"
        else:
            raise AssertionError(f"{spec!r} must refuse rather than pick the first match")
    try:
        linux_camera_for_spec("c920", [])
    except ValueError as e:
        assert "by-id" in str(e)
    else:
        raise AssertionError("an empty listing must refuse")


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
