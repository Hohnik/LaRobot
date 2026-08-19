#!/usr/bin/env python3
"""What does THIS machine provide, and what is still missing? Run this first on a new machine.

    uv run checks/check_platform.py
    uv run checks/check_platform.py --raw     # also dump every raw listing it parsed

⭐⭐ WHY THIS IS THE FIRST COMMAND ON THE LINUX PC (ROADMAP §8.2 item 49, [FINDINGS §74.0](../docs/FINDINGS.md)). The Linux device-naming code in `yam/platform.py` was written from the DOCUMENTED sysfs and by-id formats, not from a capture of Julien's machine, because no Linux machine had been reached when it was written. This report prints the RAW text beside the parse, so its first run either confirms the format or shows exactly how it differs — which is the difference between a port that is verified and a port that merely runs.

⛔ It transmits nothing, opens no camera and energises nothing. Every fix it suggests that needs `sudo` or a physical action is named as the operator's.
"""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent

from yam.can import ARM_SERIALS  # noqa: E402
from yam.platform import (  # noqa: E402
    CAN_BITRATE,
    IS_LINUX,
    IS_MACOS,
    camera_permission_note,
    parse_can_links,
    platform_name,
    platform_note,
    read_can_links,
    read_v4l_cameras,
    sysfs_can_serial,
)

#: Command-line tools each platform needs, with what breaks without them.
TOOLS = {
    "ffmpeg":  "encoding training videos (apps/export_dataset.py)",
    "ffprobe": "verifying the encoding numerically (checks/check_dataset.py)",
}
LINUX_TOOLS = {
    "ip":        "listing and configuring SocketCAN interfaces",
    "v4l2-ctl":  "optional: camera modes and controls (apt install v4l-utils)",
}
MAC_TOOLS = {"ioreg": "reading USB serials without root"}


def run(cmd: list[str]) -> tuple[bool, str]:
    """Run a read-only command; `(ok, output)`. Never raises — a missing tool is data."""
    try:
        out = subprocess.run(cmd, capture_output=True, text=True, check=False, timeout=20)
    except (OSError, subprocess.SubprocessError) as exc:
        return False, f"{type(exc).__name__}: {exc}"
    return out.returncode == 0, (out.stdout or "") + (out.stderr or "")


def section(title: str) -> None:
    print(f"\n=== {title} " + "=" * max(0, 66 - len(title)))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--raw", action="store_true",
                    help="also print every raw listing, so the parse can be audited by eye")
    args = ap.parse_args()

    blockers: list[str] = []
    notes: list[str] = []

    section("this machine")
    print(f"  platform    : {platform_name()}  (sys.platform={sys.platform!r})")
    print(f"  python      : {sys.version.split()[0]}")
    print(f"  device world: {platform_note()}")
    print(f"  cameras     : {camera_permission_note()}")
    if not (IS_MACOS or IS_LINUX):
        blockers.append(f"{sys.platform} has no device implementation at all.")

    section("tools")
    wanted = dict(TOOLS, **(LINUX_TOOLS if IS_LINUX else MAC_TOOLS if IS_MACOS else {}))
    for tool, why in wanted.items():
        where = shutil.which(tool)
        mark = "✓" if where else ("·" if "optional" in why else "⛔")
        print(f"  {mark} {tool:<10} {where or 'NOT FOUND'}   {why}")
        if not where and "optional" not in why:
            blockers.append(f"{tool} is missing, needed for: {why}")

    if IS_LINUX:
        section("group membership (Linux access control)")
        ok, groups = run(["id"])
        print(f"  {groups.strip() or '(id failed)'}")
        for group, why in (("video", "opening /dev/video* cameras"),
                           ("dialout", "some USB serial devices"),
                           ("plugdev", "raw USB access for HID/SpaceMouse")):
            has = group in groups
            print(f"  {'✓' if has else '⚠️'} {group:<9} {why}")
            if group == "video" and not has:
                blockers.append("the user is not in the `video` group, so no camera will "
                                "open. Fix (operator, needs sudo + a re-login): "
                                "sudo usermod -aG video $USER")

    section("clocks — the frame/joint alignment depends on this")
    # ⛔⭐ WHY THIS IS CHECKED AT ALL. A recording stamps joint samples from
    # `time.perf_counter()` and camera frames from `time.monotonic_ns()`, and
    # `yam/episode.py` joins the two by SUBTRACTING one from the other. That is only valid
    # if both clocks share an epoch. On this Mac they are the same clock
    # (mach_absolute_time, measured 2026-08-19, FINDINGS §71.2) — on Linux they are both
    # documented as CLOCK_MONOTONIC, which would also share an epoch, but "documented" is
    # not "measured" and this repo does not treat those as equal. So: measure it here, on
    # whatever machine this runs on, and let the number speak.
    import time

    perf, mono = time.get_clock_info("perf_counter"), time.get_clock_info("monotonic")
    offsets = [abs(time.perf_counter() - time.monotonic_ns() / 1e9) for _ in range(5)]
    drift = max(offsets) - min(offsets)
    print(f"  perf_counter: {perf.implementation}  monotonic: {mono.implementation}")
    print(f"  |perf_counter - monotonic| = {min(offsets):.9f}s, "
          f"varying by {drift:.9f}s over 5 reads")
    same_epoch = min(offsets) < 0.001
    if same_epoch:
        print("  ✓ the two clocks share an epoch, so camera frames and joint samples land on "
              "ONE\n     time axis and the episode join is exact.")
    else:
        print(f"  ⛔ the clocks are {min(offsets):.6f}s apart. yam/episode.py's frame join "
              "subtracts one\n     from the other, so every exported episode would pair "
              "images with the wrong joints.")
        blockers.append(
            f"perf_counter and monotonic differ by {min(offsets):.6f}s on this machine. "
            "The recorder stamps joints with the first and frames with the second, and "
            "yam/episode.py joins them by subtraction — fix that before recording anything "
            "you intend to keep (FINDINGS §71.2 has the alignment argument).")

    section("CAN — the arms")
    if IS_LINUX:
        ok, raw = run(["ip", "-details", "link", "show", "type", "can"])
        if args.raw:
            print("  --- raw `ip -details link show type can` ---")
            for line in (raw or "(empty)").splitlines():
                print(f"  | {line}")
            print("  --- end raw ---")
        names = [l.split(":")[1].strip() for l in raw.splitlines()
                 if l and l[0].isdigit() and ":" in l]
        serials = {n: sysfs_can_serial(n) for n in names}
        if args.raw and names:
            for n in names:
                print(f"  | sysfs serial for {n}: {serials[n]!r} "
                      f"(from /sys/class/net/{n}/device/../serial)")
        links = parse_can_links(raw, serials)
        if not links:
            print("  ⛔ no SocketCAN interface is present.")
            blockers.append("No CAN interface. Plug the adapters in; if they are plugged in, "
                            "the gs_usb kernel module may not have bound them (docs/LINUX.md).")
        known = {v: k for k, v in ARM_SERIALS.items()}
        for link in links:
            arm = known.get(link.serial)
            who = f"arm {arm}" if arm else "⚠️ UNKNOWN adapter"
            rate = f"{link.bitrate} bit/s" if link.bitrate else "no bitrate set"
            mark = "✓" if (arm and link.state == "UP" and link.bitrate == CAN_BITRATE) else "⚠️"
            print(f"  {mark} {link.interface:<6} {who:<18} serial "
                  f"{link.serial or '(unreadable)':<20} {link.state}, {rate}")
            if link.state != "UP":
                blockers.append(
                    f"{link.interface} is {link.state}. Operator, needs sudo: "
                    f"sudo ip link set {link.interface} up type can bitrate {CAN_BITRATE}")
            elif link.bitrate not in (None, CAN_BITRATE):
                blockers.append(f"{link.interface} runs at {link.bitrate}, motors need "
                                f"{CAN_BITRATE}.")
            if not link.serial:
                notes.append(f"{link.interface} exposes no USB serial in sysfs, so it cannot "
                             "be matched to an arm. Run with --raw and send the output.")
        for arm, serial in sorted(ARM_SERIALS.items()):
            if not any(l.serial == serial for l in links):
                print(f"  ⚠️ arm {arm} (serial {serial}) is not on any CAN interface")
    else:
        ok, _ = run(["ioreg", "-p", "IOUSB", "-w0", "-l"])
        print(f"  macOS path: gs_usb over libusb. ioreg readable: {'✓' if ok else '⛔'}")
        print("  Arm→adapter resolution is `uv run checks/check_rig.py` on this platform.")

    section("cameras")
    if IS_LINUX:
        by_id = Path("/dev/v4l/by-id")
        if args.raw:
            print(f"  --- raw listing of {by_id} ---")
            try:
                for p in sorted(by_id.iterdir()):
                    print(f"  | {p.name} -> {p.readlink() if p.is_symlink() else '(not a link)'}")
            except OSError as exc:
                print(f"  | unreadable: {exc}")
            print("  --- end raw ---")
        cams = read_v4l_cameras()
        if not cams:
            print(f"  ⛔ nothing in {by_id}.")
            blockers.append(f"No camera in {by_id}. Plug them in; if they are in, run with "
                            "--raw and send the output — the by-id name format may differ "
                            "from what yam/platform.py expects (FINDINGS §74.0).")
        for cam in cams:
            print(f"  ✓ index {cam.index:<3} {cam.model[:40]:<40} "
                  f"serial {cam.serial or '(none reported)'}")
        if cams:
            print("\n  ⭐ On Linux these indices come from the by-id symlinks, so they ARE "
                  "OpenCV's\n     indices. No hint file and no lens-covering "
                  "(the macOS workaround of FINDINGS §70.15).")
        try:
            import pyrealsense2  # noqa: F401, PLC0415
            print("  ✓ pyrealsense2 present — D405 DEPTH is available on this machine")
        except ImportError:
            notes.append("pyrealsense2 is not installed, so the D405s are colour-only here "
                         "(same as the Mac). Depth needs librealsense + pyrealsense2, which "
                         "the rebuild plan wants on Linux (docs/PLAN.md Phase B).")
    else:
        print("  macOS path: AVFoundation + the confirmed index hint. "
              "`uv run apps/camera_view.py --list` is the report here.")

    section("SpaceMouse")
    try:
        import hid  # noqa: PLC0415

        devices = hid.enumerate()
        pucks = [d for d in devices
                 if "spacemouse" in str(d.get("product_string", "")).lower()
                 or d.get("vendor_id") in (0x256F, 0x046D) and "space" in
                 str(d.get("product_string", "")).lower()]
        print(f"  hidapi sees {len(devices)} HID device(s), {len(pucks)} SpaceMouse-like")
        for d in pucks:
            print(f"  ✓ {d.get('product_string')}  vid:pid "
                  f"{d.get('vendor_id'):04x}:{d.get('product_id'):04x}  "
                  f"serial {d.get('serial_number') or '(empty — assigned by wiggle)'}")
        if not pucks and IS_LINUX:
            notes.append("No SpaceMouse seen. On Linux a puck needs a udev rule before a "
                         "non-root user may open it — docs/LINUX.md has the rule.")
    except Exception as exc:  # noqa: BLE001
        print(f"  ⚠️ hidapi unavailable: {type(exc).__name__}: {exc}")

    section("verdict")
    if blockers:
        print(f"⛔ {len(blockers)} thing(s) stand between this machine and a session:\n")
        for i, item in enumerate(blockers, 1):
            print(f"  {i}. {item}")
    else:
        print("✓ nothing is blocking a session on this machine, as far as this check can see.")
    if notes:
        print(f"\n⚠️ {len(notes)} note(s), none blocking:\n")
        for item in notes:
            print(f"  · {item}")
    print("\n⚠️ This reports DEVICE and TOOL state only. It never transmits, so it cannot say "
          "\n   whether a motor is healthy — that is `uv run apps/ping_motors.py --arm B --yes`.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
