#!/usr/bin/env python3
"""Tests for `pick_device_by_wiggle`'s exclusion. No hardware, no HID device.

    uv run tests/test_puck_assignment.py

⛔ WHY THIS IS WORTH TESTING. Without `exclude`, calling the picker once per arm can
hand **the same puck to both arms**, and both routes to that are silent: the
single-device shortcut returns unconditionally, and with two attached nothing stops
the operator moving the one they already assigned. The symptom — two arms following
one hand — reads as a control bug, so it would be debugged in the control loop,
which is the wrong file entirely. That is exactly the CAN-adapter-by-index failure
(FINDINGS §0 #5) in a new place.

The device list is monkeypatched, so this exercises the selection logic without
touching hidapi or seizing Julien's SpaceMouse.
"""

from __future__ import annotations

import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent

import yam.inputs.spacemouse as spacemouse  # noqa: E402

PUCK_A = {"path": b"DevSrvsID:1111", "vendor_id": 0x256F, "product_id": 0xC635}
PUCK_B = {"path": b"DevSrvsID:2222", "vendor_id": 0x256F, "product_id": 0xC635}


class FakeDevices:
    """Swap out find_all_devices() for the duration of a test."""

    def __init__(self, devices: list[dict]):
        self.devices = devices

    def __enter__(self):  # noqa: ANN204
        self._real = spacemouse.find_all_devices
        spacemouse.find_all_devices = lambda: list(self.devices)
        return self

    def __exit__(self, *exc: object) -> None:
        spacemouse.find_all_devices = self._real


def test_one_puck_no_exclusion_is_returned() -> None:
    with FakeDevices([PUCK_A]):
        assert spacemouse.pick_device_by_wiggle() == PUCK_A


def test_the_only_puck_is_NOT_handed_out_twice() -> None:
    """⛔ The core regression: one puck, two arms, must not silently serve both."""
    with FakeDevices([PUCK_A]):
        first = spacemouse.pick_device_by_wiggle(label="B")
        assert first == PUCK_A
        second = spacemouse.pick_device_by_wiggle(label="G", exclude=[first["path"]])
        assert second is None, "the same puck was assigned to both arms"


def test_second_call_gets_the_other_puck_without_asking() -> None:
    """Two pucks, one already taken -> the remaining one needs no wiggle at all."""
    with FakeDevices([PUCK_A, PUCK_B]):
        second = spacemouse.pick_device_by_wiggle(label="G", exclude=[PUCK_A["path"]])
        assert second == PUCK_B


def test_exclusion_accepts_bytes_or_str_paths() -> None:
    with FakeDevices([PUCK_A, PUCK_B]):
        assert spacemouse.pick_device_by_wiggle(exclude=[bytearray(PUCK_A["path"])]) == PUCK_B


def test_no_devices_at_all_returns_none() -> None:
    with FakeDevices([]):
        assert spacemouse.pick_device_by_wiggle() is None


def test_excluding_everything_returns_none() -> None:
    with FakeDevices([PUCK_A, PUCK_B]):
        got = spacemouse.pick_device_by_wiggle(exclude=[PUCK_A["path"], PUCK_B["path"]])
        assert got is None


def test_a_puckless_arm_still_joins_the_session() -> None:
    """⭐⭐ Item 47 (FINDINGS §68.5): with one SpaceMouse and two arms the session used to
    refuse outright, which killed MIRROR and two-arm playback although a follower never
    needs a hand. The fallback lives in the session script and this file cannot attach
    hardware, so it pins the source: the fallback exists, uses the zero-deflection
    StillPuck, and fires ONLY when some puck is already assigned AND none is free —
    an attached-but-unmoved puck must still abort, never silently lose its TELEOP."""
    src = (REPO / "apps" / "teleop_session.py").read_text()
    at = src.find('pucks[name] = {"path": f"none:{name}"')
    assert at != -1, "the puckless-arm fallback is gone"
    assert "StillPuck()" in src[at:at + 120], "the fallback no longer uses StillPuck"
    gate = src.rfind("if pucks and not [d for d in find_all_devices()", 0, at)
    assert gate != -1 and at - gate < 400, \
        "the fallback lost its gate — it must fire only when no unassigned device exists"


def test_a_single_shared_puck_follows_the_selection() -> None:
    """⭐⭐ His design (FINDINGS §68.8): with ONE real puck in a multi-arm session, `a`
    aims the puck as well as the mode keys — B, G, or BOTH driving both arms at once.
    Source-pinned: the shared reader is read ONCE per cycle (two arms draining one HID
    queue would split the event stream), routing uses the live selection, and unaimed
    arms read a centred puck."""
    src = (REPO / "apps" / "teleop_session.py").read_text()
    assert "shared_axes = shared_puck.read()" in src, \
        "the shared puck is no longer read once at session level"
    assert "aimed_now = one.name in selection.names()" in src, \
        "the shared puck no longer follows the selection"
    assert 'one.raw_axes = list(shared_axes) if aimed_now else [0.0] * 6' in src, \
        "unaimed arms no longer read as centred"
    gate = src.find("if len(arm_names) > 1 and len(_real_pucks) == 1 else None")
    assert gate != -1, "the shared-puck mode lost its exactly-one-real-puck gate"


#: ✅ REAL captures. The Linux entry is exactly what `hid.enumerate()` returned on the station
#: 2026-08-19 for the one attached puck; the Logitech mouse and the C920 are also really on
#: that machine. ⚠️ The macOS entry is the documented macOS shape (usage_page 0x01, usage 0x08
#: are this module's own measured constants) — the puck had already moved to the PC by the time
#: this test was written, so it could not be re-captured on the Mac at that moment.
LINUX_PUCK = {"vendor_id": 0x256F, "product_id": 0xC635, "product_string": "",
              "usage_page": 0, "usage": 0, "path": b"9-2:1.0"}
MAC_PUCK = {"vendor_id": 0x256F, "product_id": 0xC635,
            "product_string": "SpaceMouse Compact", "usage_page": 0x01, "usage": 0x08}
LINUX_MOUSE = {"vendor_id": 0x046D, "product_id": 0xC077,
               "product_string": "USB Optical Mouse", "usage_page": 0, "usage": 0}
LEGACY_PUCK = {"vendor_id": 0x046D, "product_id": 0xC626,
               "product_string": "SpaceNavigator", "usage_page": 0x01, "usage": 0x08}
C920 = {"vendor_id": 0x046D, "product_id": 0x08E5, "product_string": "HD Pro Webcam C920",
        "usage_page": 0x01, "usage": 0x02}


def test_a_puck_is_recognised_on_both_platforms() -> None:
    """⛔ THE DEFECT THIS FIXES (FINDINGS §75.9): on macOS hidapi fills usage_page/usage, on
    Linux it leaves them 0. Code that REQUIRED them rejected the only puck attached to the
    station and reported "No SpaceMouse found", which is a true statement about the wrong
    question."""
    from yam.inputs.spacemouse import looks_like_a_puck, usage_fields_readable

    assert usage_fields_readable(MAC_PUCK) and not usage_fields_readable(LINUX_PUCK)
    assert looks_like_a_puck(MAC_PUCK), "the macOS shape must still be accepted"
    assert looks_like_a_puck(LINUX_PUCK), "the Linux shape has no usage fields to offer"
    assert looks_like_a_puck(LEGACY_PUCK), \
        "a legacy Logitech-branded unit that DOES declare multi-axis is still a puck"


def test_a_logitech_device_is_never_accepted_blind() -> None:
    """⛔ 0x046D covers the legacy pucks AND the C920 AND the plain optical mouse sitting on
    the station. Accepting that vendor without evidence would open the mouse: it enumerates,
    opens, reports a plausible name, and never sends a motion report — indistinguishable
    from a decode bug (FINDINGS §0)."""
    from yam.inputs.spacemouse import looks_like_a_puck

    assert not looks_like_a_puck(LINUX_MOUSE), \
        "a Logitech mouse with no usage fields must be refused, not guessed at"
    assert not looks_like_a_puck(C920), "the webcam declares usage 0x02, which is not a puck"
    assert not looks_like_a_puck({"vendor_id": 0x1234, "usage_page": 0x01, "usage": 0x08}), \
        "an unknown vendor is not a puck however it describes itself"


def main() -> int:
    tests = [(n, f) for n, f in sorted(globals().items()) if n.startswith("test_") and callable(f)]
    failed = []
    for name, fn in tests:
        try:
            fn()
            print(f"  ✓ {name}")
        except Exception as exc:  # noqa: BLE001
            failed.append(name)
            print(f"  ✗ {name}\n      {type(exc).__name__}: {exc}")
    print(f"\n{len(tests) - len(failed)}/{len(tests)} passed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
