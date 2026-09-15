"""Assemble recording camera readers, preserving ownership through partial startup.

Linux uses by-id identity and measured frame delivery. macOS resolves serials,
checks hinted models where a distinguishing mode exists, and retains full serial
names in recordings. Readers transfer to the returned CaptureSet on success.
"""
from __future__ import annotations

from yam.cameras.capture import CaptureSet
from yam.cameras.grabber import FrameGrabber
from yam.cameras.open import SIZE_LADDER, TARGET_FPS, open_camera, open_measured
from yam.cameras.specs import camera_dir_name, flatten_tokens
from yam.cameras.startup import CameraStartup

def _open_session_cameras_linux(specs_text: str) -> tuple[CaptureSet, list[str]]:
    """Open Linux capture nodes with measured frames and partial-startup ownership."""
    with CameraStartup() as startup:
        import cv2  # noqa: PLC0415

        from yam.cameras.identity import linux_camera_for_spec  # noqa: PLC0415
        from yam.platform import dynamic_framerate_allowed, read_v4l_cameras  # noqa: PLC0415

        listed = read_v4l_cameras()
        grabbers: dict[str, FrameGrabber] = {}
        for spec in flatten_tokens([specs_text]):
            try:
                cam = linux_camera_for_spec(spec, listed)
            except ValueError as e:
                raise SystemExit(f"⛔ {spec}: {e}") from e
            model_word = spec.partition(":")[0]
            name = camera_dir_name(f"{model_word}:{cam.serial}" if cam.serial else model_word)
            if name in grabbers:
                raise SystemExit(f"⛔ --cameras names {name!r} twice.")
            # ⛔⭐ A metadata node opens fine and delivers nothing (FINDINGS §75.6), so refuse
            # one outright rather than recording a camera that produces no frames. `cam.index`
            # is already a capture node when udev could be asked; this guards the explicit
            # `--cameras <N>` spelling, where the operator picks the number.
            if cam.capture_nodes and cam.index not in cam.capture_nodes:
                raise SystemExit(
                    f"⛔ {spec}: /dev/video{cam.index} is not a CAPTURE node on this camera "
                    f"(its capture nodes are {list(cam.capture_nodes)}).\n"
                    "  A metadata node opens successfully and delivers no frames, which is why "
                    "this refuses instead of recording nothing."
                )
            # ⭐ Say which stream is being opened and how that was decided. A D405's FIRST
            # capture node is DEPTH (Z16), so "opened the first one" would be a silent wrong
            # answer — the formats are read and the COLOUR node chosen (FINDINGS §75.7).
            if cam.index_reason == "colour-format":
                print(f"  ✓ {name}: /dev/video{cam.index} is the COLOUR stream, identified from "
                      "its pixel formats")
            elif len(cam.capture_nodes) > 1:
                raise SystemExit(
                    f"⛔ {spec}: this camera has {len(cam.capture_nodes)} capture streams "
                    f"{list(cam.capture_nodes)} and their pixel formats could not be read, so "
                    "the COLOUR one cannot be identified.\n"
                    "  On a D405 the first stream is DEPTH, and recording it as if it were a "
                    "photograph is exactly the silent-wrong-answer this refuses to make.\n"
                    "  Fix: join the `video` group (`sudo usermod -aG video $USER`, then log out "
                    "and back in), or pass the node directly, e.g. --cameras "
                    f"{cam.capture_nodes[-1]}."
                )
            cap = startup.own(cv2.VideoCapture(cam.index))
            if not cap.isOpened():
                startup.release(cap)
                raise SystemExit(
                    f"⛔ {spec} resolved to {cam.device} (index {cam.index}) and would not open.\n"
                    "  On Linux this is almost always group membership: the user must be in "
                    "`video`.\n  Check with `id`, and see docs/LINUX.md."
                )
            # ⛔⭐⭐ THE WORD "delivering" USED TO BE A CLAIM. This block used to set the size,
            # read the size back with `cap.get`, and print it as what the camera was delivering.
            # On 2026-08-19 that printed "delivering 1280x720" for a D405 that delivered ZERO
            # frames for the whole session, and for a C920 running at 10 fps instead of 30
            # because no MJPG was requested. Both are in FINDINGS §76. `open_measured` reads
            # real frames, counts them, steps the size down when nothing arrives, and every
            # number below comes out of a frame that actually existed.
            opened = open_measured(cap)
            if opened is None:
                startup.release(cap)
                raise SystemExit(
                    f"⛔ {spec}: {cam.device} (index {cam.index}) opened and delivered NO FRAMES "
                    f"at any of {SIZE_LADDER}.\n"
                    "  It accepted the settings and produced nothing, which is why this refuses "
                    "instead of recording an empty camera.\n"
                    f"  Check the device on its own:  v4l2-ctl -d {cam.device} "
                    "--stream-mmap --stream-count=5 --stream-to=/dev/null\n"
                    "  If that also hangs, the camera or its cable is the problem, not this "
                    "session. See docs/LINUX.md."
                )
            grabbers[name] = startup.start_reader(cap, FrameGrabber)
            print(f"  📷 {name} open on {cam.device} (index {cam.index}), "
                  f"measured {opened.line()}.")
            if opened.stepped_down:
                print(f"     ⚠️ stepped down from {opened.asked[0]}x{opened.asked[1]}: this "
                      "camera accepted that size and delivered no frames at it.")
            # ⛔⭐⭐ THE ROOM CAN HALVE THE FRAME RATE, AND ONLY THIS LINE SAYS SO. A C920 with
            # `exposure_dynamic_framerate` on drops from 29.92 fps to 14.98 in a dim room, at the
            # same size and format, while the driver keeps reporting 30 (FINDINGS §76.16). So the
            # same rig yields 30 fps by day and 15 by night with nothing on screen to show it.
            # ⭐ Reported, never changed: turning it off buys a steady rate and pays in darker
            # pictures, and that trade is the operator's, like which arm stands on the left.
            dyn = dynamic_framerate_allowed(cam.device)
            if dyn:
                print("     ⚠️ this camera may HALVE its own frame rate to lengthen exposure in a "
                      "dim room, and the driver still reports 30.")
                print("        Steady rate instead of brighter pictures:  v4l2-ctl -d "
                      f"{cam.device} --set-ctrl=exposure_dynamic_framerate=0")
            if opened.slow:
                print(f"     ⛔ {opened.fps:.1f} fps is well under {TARGET_FPS:.0f}. The episode "
                      "exporter fills 30 ticks a second, so a camera this slow makes every")
                print("        tick repeat frames.")
                if dyn:
                    print("        ⭐ MOST LIKELY THE LINE ABOVE, because the control is on and "
                          "this rate is about half of 30.")
                else:
                    print("        Usually the format: check that MJPG is offered with  "
                          f"v4l2-ctl --list-formats-ext -d {cam.device}")
        return CaptureSet(grabbers), list(grabbers)


def open_session_cameras(specs_text: str) -> tuple[CaptureSet, list[str]]:
    """Open recording readers with stable names and platform-specific verification."""
    with CameraStartup() as startup:
        from yam.platform import IS_LINUX  # noqa: PLC0415

        if IS_LINUX:
            return _open_session_cameras_linux(specs_text)

        from yam.cameras.discovery import (  # noqa: PLC0415 — OpenCV + AVFoundation load only when cameras are asked for
            CameraLookupError,
            hinted_index,
            mac_cameras,
            model_discriminating_mode,
            resolve_camera,
        )
        from yam.cameras.identity import (  # noqa: PLC0415
            devices_matching_serial,
            read_ioreg,
            usb_unique_id,
        )

        grabbers: dict[str, FrameGrabber] = {}
        for spec in flatten_tokens([specs_text]):
            expect_uid = None
            if spec.isdigit():
                name, idx = camera_dir_name(spec), int(spec)
            elif ":" in spec:
                model, serial = spec.split(":", 1)
                matches = devices_matching_serial(serial, read_ioreg())
                if not matches:
                    raise SystemExit(
                        f"⛔ {spec}: no attached USB device's serial starts with "
                        f"{serial.strip()!r} — `uv run checks/check_rig.py` shows what is there."
                    )
                if len(matches) > 1:
                    listing = ", ".join(d["serial"] for d in matches)
                    raise SystemExit(f"⛔ {spec}: {serial.strip()!r} matches more than one "
                                     f"device ({listing}) — type more of the serial.")
                dev = matches[0]
                expect_uid = usb_unique_id(dev["location_id"], dev["vid"], dev["pid"])
                name = camera_dir_name(f"{model}:{dev['serial']}")
                idx = hinted_index(expect_uid)
                if idx is None:
                    raise SystemExit(
                        f"⛔ {spec}: the serial resolves (uniqueID {expect_uid}) but no "
                        "confirmed OpenCV index is on file for it. Two identical D405s cannot "
                        "be told apart by any measurement (FINDINGS §67.12), so the mapping "
                        "needs one physical confirmation per port arrangement: run `uv run "
                        "apps/capture_probe.py --indices 1 2 --seconds 3 --save`, look at the "
                        "two saved pictures, and say which index shows which view — "
                        "config/camera_index_hint.json then pins it (FINDINGS §71.5)."
                    )
            else:
                name = camera_dir_name(spec)
                try:
                    idx, _, found_cap = resolve_camera(spec)
                except CameraLookupError as e:
                    raise SystemExit(f"⛔ {e}") from e
                # The resolver may hand the device back already open, unconfigured. Release it and reopen below with the recording mode, so every camera goes through ONE configuration path.
                if found_cap is not None:
                    startup.release(startup.own(found_cap))
            if name in grabbers:
                raise SystemExit(f"⛔ --cameras names {name!r} twice.")
            cap = startup.own(open_camera(idx, 1280, 720, 30))
            if cap is None:
                raise SystemExit(f"⛔ {spec} resolved to index {idx} and would not open. "
                                 "`uv run apps/camera_view.py --list` shows what is there.")
            checked = ""
            if expect_uid is not None:
                cams_listed = mac_cameras()
                target = next((c for c in cams_listed if c.unique_id == expect_uid), None)
                if target is None:
                    startup.release(cap)
                    raise SystemExit(f"⛔ {spec}: ioreg sees the device but AVFoundation lists "
                                     f"no camera with uniqueID {expect_uid} — replug it, then "
                                     "`uv run apps/camera_view.py --list`.")
                probe = model_discriminating_mode(target, cams_listed)
                if probe is None:
                    print(f"  ⚠️ {name}: every mode is shared with another model, so the "
                          "hinted index cannot be model-checked.")
                else:
                    import cv2  # noqa: PLC0415

                    cap.set(cv2.CAP_PROP_FRAME_WIDTH, probe[0])
                    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, probe[1])
                    got = (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
                           int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
                    startup.release(cap)
                    if got != probe:
                        raise SystemExit(
                            f"⛔ {spec}: index {idx} did not answer {probe[0]}x{probe[1]}, a "
                            f"mode only a {target.short} offers — the hint in "
                            "config/camera_index_hint.json is STALE and would have recorded "
                            "the wrong camera under this name (FINDINGS §71.5). Re-establish "
                            "it: `uv run apps/capture_probe.py --indices 1 2 --seconds 3 "
                            "--save`, look at the pictures, say which is which."
                        )
                    # Reopened rather than reconfigured: the probe changed the mode, and one configuration path (open_camera) beats trusting set() to restore fps as well as size.
                    cap = startup.own(open_camera(idx, 1280, 720, 30))
                    if cap is None:
                        raise SystemExit(f"⛔ {spec}: index {idx} passed the model check and "
                                         "then refused to reopen — replug it and retry.")
                    checked = f", model-checked at {probe[0]}x{probe[1]}"
            grabbers[name] = startup.start_reader(cap, FrameGrabber)
            print(f"  📷 {name} open on index {idx} (asked for 1280x720@30{checked}).")
        return CaptureSet(grabbers), list(grabbers)


