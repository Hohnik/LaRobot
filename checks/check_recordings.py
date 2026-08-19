#!/usr/bin/env python3
"""Report what is in `recordings/`, and whether any of it carries the §30.1 padding.

    uv run checks/check_recordings.py
    uv run checks/check_recordings.py --dir recordings --still 0.05

⛔ WHY THIS SCRIPT EXISTS, and it is a documentation failure rather than a code one.
[FINDINGS §30.1](../docs/FINDINGS.md) found that `w` did not stop a recording where it
should have, so each file carried the seconds the save prompt spent waiting. The handoff
then said, in prose, *"slots 1, 3, 4, 5, 6 are all padded, discard them"*. ⛔ **Three of
those five were recorded after the fix and are clean**, and the sentence had no way to
know: it was written once and never re-derived against the files. This is the same defect
this repo keeps finding in its own guards ([HANDOFF §4](../docs/HANDOFF.md) rule 7), and
the defence is to measure it instead of asserting it.

⭐ It reads the files only. No hardware, no motion, nothing is written.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent

from yam.cameras.specs import camera_dir_name  # noqa: E402
from yam.files import listing, sidecars  # noqa: E402
from yam.recording import Layout, Trajectory  # noqa: E402

#: Above this, a tail is the §30.1 defect: it produced 1.8 to 4.4 s. Below it, a tail is
#: the arm coming to rest before the key was pressed, which is not a fault and not
#: something to re-record for. ⚠️ The gap between the two cases is wide in the measured
#: data, so this threshold is not delicate.
PADDING_S = 1.0

#: A recording labelled HOLD should barely move. The measured wobble floor of a held
#: arm is 0.032-0.038 rad/s ([FINDINGS §33.1](../docs/FINDINGS.md)) and hand-guiding
#: reaches 2.4-3.7. 0.5 sits far above the first and far below the second, so this is
#: not a delicate threshold.
HOLD_SPEED_S = 0.5


def label_verdict(method: str, modes: list[str] | None, peak_speed: float) -> tuple[str, str | None]:
    """Judge a recording's provenance label against what the recording actually did.

    Returns ``(text_to_show, fault)`` where ``fault`` is ``None``, ``"mismatch"`` or
    ``"implausible"``.

    ⛔ Two different defects are being caught, and they are not the same one.

    **mismatch** — the file carries several ``modes`` while ``method`` names only one.
    That is the [FINDINGS §35.4](../docs/FINDINGS.md) fix having failed to apply.

    **implausible** — ``method`` says HOLD only, and the arm moved at hand-guiding
    speed. HOLD commands the arm to stay where it is, against a position gain of 80
    on the shoulder, so the label describes something that did not happen. ⚠️ Called
    implausible rather than impossible: a hard enough shove does move a held arm.

    ⭐ This is a pure function so it can be tested. The rule lived inline first, and
    inline safety logic is what this repo keeps having to re-derive.
    """
    text = method
    if modes and len(modes) > 1 and "+" not in method:
        return f"{text}  ⛔ but modes={modes}", "mismatch"
    # ⛔⭐⭐ PARSED, NOT COMPARED, and this is a regression fixed the same day it appeared.
    #
    # This rule was `method == "live:hold"`, an exact match on a label format. On 2026-08-14
    # the recorder became able to hold several arms, so the label gained an arm prefix:
    # `live:B:hold` for one arm, `live:B:guide+G:mirror` for two. **The exact match stopped
    # matching, so the check that caught `3.json` would never have fired again** — a format
    # change silently disarming a guard, which is the [FINDINGS §0](../docs/FINDINGS.md)
    # defect class arriving through a rename rather than through a bug.
    #
    # ⭐ So: strip `live:`, split on `+`, drop any `ARM:` prefix, and ask whether ANY arm was
    # in a mode that moves. It reads both formats, and a new one only has to keep the mode
    # word last.
    words = {part.rsplit(":", 1)[-1]
             for part in method.removeprefix("live:").split("+") if part}
    if words and words <= {"hold"} and peak_speed > HOLD_SPEED_S:
        return f"{text}  ⛔ implausible", "implausible"
    return text, None


def jpeg_size(path: Path) -> tuple[int, int] | None:
    """`(width, height)` from a JPEG's own header, or None if it cannot be read.

    ⭐ Reads the header only, so it costs nothing even on a directory of 500 pictures, and it needs no image library. A JPEG is a chain of markers: `0xFF <code> <2-byte length> <payload>`, and the frame's dimensions live in whichever "start of frame" marker the encoder used (`0xC0`-`0xCF`, skipping `0xC4`, `0xC8` and `0xCC`, which mean other things).
    """
    try:
        data = path.read_bytes()
    except OSError:
        return None
    i = 2                                     # past the 0xFFD8 start-of-image marker
    while i + 9 < len(data):
        if data[i] != 0xFF:
            i += 1
            continue
        code = data[i + 1]
        if 0xC0 <= code <= 0xCF and code not in (0xC4, 0xC8, 0xCC):
            height = int.from_bytes(data[i + 5:i + 7], "big")
            width = int.from_bytes(data[i + 7:i + 9], "big")
            return (width, height) if width and height else None
        i += 2 + int.from_bytes(data[i + 2:i + 4], "big")
    return None


#: A camera's frames per second, implied by its count and the recording's length, below which
#: the recording is thin. Two thirds of the 30 the episode exporter fills its ticks with.
#:
#: ⛔⭐⭐ WHY THIS CHECK EXISTS AT ALL. On 2026-08-19 a recording on the Linux station saved
#: **26 frames across 2.43 s**, which is 10.7 fps against a nominal 30, because the camera was
#: running in an uncompressed format capped at 10 fps by its own firmware ([FINDINGS §76.1](../docs/FINDINGS.md)).
#: The recording's meta and its directory agreed perfectly, so **this checker passed it**, and
#: the whole defect was found by reading a session log rather than by any tool. ⭐ The number
#: was in the saved file the entire time: frames ÷ duration. Now it is on screen.
THIN_FPS = 20.0


def frames_verdict(meta_cameras: dict, recording_dir: Path,
                   duration_s: float = 0.0) -> tuple[str, list[str]]:
    """Measure a recording's camera frames against what its meta claims (item 48 ③).

    Returns ``(one display line, fault lines)``. The counting is the point: the meta's numbers were written at save time and the JPEGs sit on disk, so agreement is evidence the frames belong to this file and disagreement means the directory was moved, half-copied or edited — the same measure-don't-assert rule as the padding column above.

    ⭐ `duration_s` turns the count into a RATE, which is the question the count alone cannot answer. Agreement between meta and disk says the frames belong to the file. It says nothing about whether there are enough of them.
    """
    faults: list[str] = []
    parts: list[str] = []
    base = recording_dir / str(meta_cameras.get("dir", ""))
    for name, counts in meta_cameras.get("per_camera", {}).items():
        cam_dir = base / camera_dir_name(name)
        jpgs = listing(cam_dir, "*.jpg")
        on_disk = len(jpgs)
        want = int(counts.get("written", 0))
        # ⭐ The recorded SIZE, read from the first picture rather than from any claim.
        # It became a question worth answering on 2026-08-19: the D405 colour stream
        # delivers nothing at 1280x720 on Linux and works at 848x480, so a session can
        # now legitimately record two cameras at two different sizes (FINDINGS §76).
        # Nothing is wrong with that, and nobody should have to guess which happened.
        piece = f"{name}:{on_disk}"
        size = jpeg_size(jpgs[0]) if jpgs else None
        if size:
            piece += f"@{size[0]}x{size[1]}"
        if on_disk != want:
            piece += "⛔"
            faults.append(f"{name}: meta says {want} frame(s) were written and "
                          f"{cam_dir.relative_to(recording_dir)} holds {on_disk} — "
                          "these frames are not the recording's own.")
        # ⛔ ZERO FRAMES IS A FAULT even when the meta agrees, and this is the case that
        # started FINDINGS §76: a camera was named on the command line, opened, reported
        # "delivering 1280x720", and wrote nothing. Meta said 0, disk held 0, they agreed,
        # and this checker said nothing at all. Agreement is not health.
        if want == 0 and on_disk == 0:
            piece += "⛔"
            faults.append(f"{name}: recorded ZERO frames. The camera was named for this "
                          "recording and contributed nothing, so any episode exported from "
                          "it has an empty view. See FINDINGS §76.0.")
        elif duration_s > 0:
            rate = on_disk / duration_s
            piece += f" {rate:.1f}fps"
            if rate < THIN_FPS:
                piece += "⚠️"
                faults.append(f"{name}: {on_disk} frames over {duration_s:.2f}s is "
                              f"{rate:.1f} fps, well under the 30 the episode exporter fills "
                              "its ticks with, so every tick repeats pictures. The known "
                              "cause is a camera left in an uncompressed format (FINDINGS "
                              "§76.1). Not corrupt data; thin data.")
        if counts.get("dropped", 0):
            piece += f" ({counts['dropped']} dropped)"
        if counts.get("write_errors", 0):
            piece += f" ({counts['write_errors']} write error(s))"
        parts.append(piece)
    return "  📷 " + " · ".join(parts), faults


def orphaned_frames(recording_dir: Path, files: list[Path]) -> list[str]:
    """Frame directories no recording accounts for — dead sessions and stale slots.

    A `pending_*` directory is a take whose session died before the save digit; a slot directory whose `.json` is missing or frameless is debris from an overwritten or deleted recording. Both would read as belonging to something, which is why the checker names them instead of leaving them plausible.
    """
    frames_root = recording_dir / "frames"
    if not frames_root.is_dir():
        return []
    claimed = set()
    for path in files:
        try:
            info = Trajectory.load(path).meta.get("cameras") or {}
        except Exception:  # noqa: BLE001 — an unreadable file is already reported above
            continue
        if info.get("dir"):
            claimed.add((recording_dir / info["dir"]).resolve())
    out = []
    for child in sorted(frames_root.iterdir()):
        if child.is_dir() and child.resolve() not in claimed:
            kind = ("a session died before the save digit"
                    if child.name.startswith("pending_")
                    else "no recording claims it")
            out.append(f"{child.relative_to(recording_dir)} — {kind}; safe to delete.")
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--dir", default="recordings", help="where the .json files are")
    ap.add_argument("--still", type=float, default=0.05,
                    help="rad/s below which a joint counts as not moving (default 0.05, "
                         "chosen above the 0.032-0.038 wobble floor of a held arm)")
    args = ap.parse_args()

    folder = REPO / args.dir if not Path(args.dir).is_absolute() else Path(args.dir)
    files = listing(folder, "*.json")
    litter = sidecars(folder)
    if not files:
        print(f"no recordings in {folder}")
        return 0
    if litter:
        # ⭐ Reported rather than silently skipped: an operator whose copy brought 813
        # sidecar files should know, because those files also break other tools and they
        # made this checker report a false fault about real data (FINDINGS §76).
        print(f"⚠️ {len(litter)} macOS sidecar file(s) (`._*`) in {folder}, ignored here.")
        print("   They appear when a Mac copies files to a filesystem that cannot hold its")
        print("   extended attributes, for example a USB stick. They are not data.")
        print("   Remove them with:  find recordings -name '._*' -delete")
        print()

    # ⭐ `arms` is a column since 2026-08-14, when a recording became able to hold more
    # than one arm. A two-arm file and a one-arm file look identical in every other column,
    # and playing the wrong one into the wrong session is the mistake worth making visible.
    print(f"{'file':>9} {'arms':>6} {'commit':>9} {'recorded':>17} {'dur':>7} "
          f"{'padding':>9} {'share':>6} {'peak p99':>9}  {'how it was made':<22}")
    padded: list[tuple[str, bool]] = []   # (file name, its modes include a park)
    # ⛔ TWO LISTS, and they used to be one. On 2026-08-19 the Linux station printed
    # "⛔ 3 file(s) are labelled `live:hold` yet moved faster than 0.5 rad/s: 5.json,
    # 5.json, 5.json" for a file whose own row in the table above showed 0.43 rad/s and
    # whose label was fine. The three entries were FRAME-COUNT faults appended to the
    # label-fault list, so the summary named the wrong defect, the wrong count and the
    # same file three times (FINDINGS §76). A checker that misreports its own finding is
    # worse than one that says nothing, because somebody acts on it.
    label_faults: list[str] = []
    frame_faults: list[str] = []
    for path in files:
        try:
            traj = Trajectory.load(path)
        except Exception as exc:  # noqa: BLE001
            print(f"{path.name:>9}  ⛔ unreadable: {type(exc).__name__}: {exc}")
            continue
        pad = traj.trailing_still_seconds(args.still)
        share = 100.0 * pad / traj.duration if traj.duration else 0.0
        flag = "  ⛔" if pad > PADDING_S else ""
        if pad > PADDING_S:
            # ⭐ FINDINGS §73.1: a recording of an AUTOMATED run (its modes include a park) usually carries a real trailing pause — the seconds between the run finishing and the operator pressing w. Same measurement, different likely cause, different advice; the split keeps the §30.1 defect loud without telling anyone to re-record a healthy run.
            is_run = any(":park" in m for m in (traj.meta.get("modes") or []))
            padded.append((path.name, is_run))
        # ⭐ `method` is shown because the defect in FINDINGS §35.4 was found by
        # reading a saved file rather than the screen: a movement hand-guided in
        # GUIDE was stamped `live:hold`, because the label was written at the
        # keypress and the mode changed afterwards. The fix collects every mode
        # the recording passed through, so a run started in HOLD and guided reads
        # `live:hold+guide`. **This column is how you check that fix**, and reading
        # it from the file is the same route that caught the original.
        method, fault = label_verdict(
            str(traj.meta.get("method", "?")), traj.meta.get("modes"), traj.joint_speed(99)
        )
        # ⭐ item 8: a labelled recording says so here, because the dataset export will
        # act on these spans and a label nobody can see is a label nobody can check.
        if traj.bad_seconds() > 0:
            method += f" · ✎{traj.bad_seconds():.1f}s bad"
        if fault:
            label_faults.append(path.name)
        # ⚠️ Read through `Layout.from_meta`, which also understands files written before
        # the layout existed: they carry a single `arm` field and nothing else.
        layout = Layout.from_meta(traj.meta, traj.n_joints)
        arms_col = ",".join(layout.arms)
        print(f"{path.name:>9} {arms_col:>6} {str(traj.meta.get('commit', '?')):>9} "
              f"{str(traj.meta.get('recorded_at', '?'))[:16]:>17} "
              f"{traj.duration:6.2f}s {pad:8.2f}s {share:5.1f}% "
              f"{traj.joint_speed(99):8.2f}{flag}  {method:<22}")
        # ⭐ item 48: a recording that carries camera frames says so on its own second line, counted from disk rather than trusted from meta.
        if traj.meta.get("cameras"):
            line, cam_faults = frames_verdict(traj.meta["cameras"], folder, traj.duration)
            print(line)
            for fault in cam_faults:
                if path.name not in frame_faults:
                    frame_faults.append(path.name)
                print(f"      ⛔ {fault}")

    orphans = orphaned_frames(folder, files)
    if orphans:
        print()
        print(f"⚠️ {len(orphans)} frame director(ies) no recording accounts for:")
        for line in orphans:
            print(f"   {line}")

    print()
    if label_faults:
        print(f"⛔ {len(label_faults)} file(s) carry a provenance label that disagrees with what "
              f"the recording did: {', '.join(label_faults)}.")
        print("   Either `method` names one mode while `modes` lists several, or `method` says")
        print("   HOLD only and the arm moved faster than "
              f"{HOLD_SPEED_S} rad/s. HOLD commands the arm to")
        print("   stay put, so that label and that speed cannot both be true.")
        print("   The known cause is the FINDINGS §35.4 defect: `method` was written when the")
        print("   recording started and the mode was changed afterwards, so a movement guided")
        print("   by hand came out stamped as HOLD. The data is fine; the label is not.")
        print()
    if frame_faults:
        print(f"⛔ {len(frame_faults)} file(s) have a camera problem: {', '.join(frame_faults)}.")
        print("   Three different things land here, and the per-camera lines above say which:")
        print("   a count that disagrees with the directory · a camera that recorded ZERO")
        print("   frames · a camera whose frames-per-second is far under 30.")
        print("   ⭐ FIRST THING TO RULE OUT, because it caused every instance of this so far:")
        print("   macOS sidecar files. If the folder was hand-copied from the Mac, every picture")
        print("   arrived with a `._`-prefixed twin and the count came out at exactly double.")
        print("   This checker now skips them, so a doubled count here means something else.")
        print()

    # ⛔ This pass RE-LOADS every file and had no error guard, while the table above did.
    # So `._5.json` printed politely as "⛔ unreadable" in the table and then crashed the
    # whole script forty lines later with UnicodeDecodeError (FINDINGS §76). The sidecar
    # is filtered out at the source now; the guard stays, because "one loop is careful
    # and the other is not" is the actual defect and any unreadable file re-creates it.
    pre_fix = []
    for path in files:
        try:
            if not Trajectory.load(path).meta.get("modes"):
                pre_fix.append(path.name)
        except Exception:  # noqa: BLE001 — already reported as unreadable in the table
            continue
    if pre_fix:
        print(f"⚠️ {len(pre_fix)} file(s) carry no `modes` field: {', '.join(pre_fix)}.")
        print("   Those were recorded before the FINDINGS §35.4 provenance fix, so their")
        print("   `method` names only the mode the recording STARTED in. Not a fault in the")
        print("   data, and it does mean the label cannot be trusted for those files.")
        print()
    defect = [name for name, is_run in padded if not is_run]
    run_tails = [name for name, is_run in padded if is_run]
    if defect:
        print(f"⛔ {len(defect)} of {len(files)} carry more than {PADDING_S:.1f}s of "
              f"padding: {', '.join(defect)}")
        print("   That is the FINDINGS §30.1 defect. Re-record those; it takes seconds.")
    if run_tails:
        print(f"⚠️ {len(run_tails)} run-recording(s) carry more than {PADDING_S:.1f}s of "
              f"trailing still time: {', '.join(run_tails)}")
        print("   These recorded an automated run (their modes include a park), and the")
        print("   tail is usually the gap between the run finishing and w being pressed —")
        print("   not the §30.1 defect. Wasted ticks in an episode, not broken data. The")
        print("   technique that avoids it: press w DURING the run's last leg (FINDINGS §73.1).")
    if not padded:
        print(f"✓ none of the {len(files)} files carry more than {PADDING_S:.1f}s of "
              f"trailing still time.")
    print("\n⚠️ Padding is measured at the END only, so a pause in the middle is invisible,")
    print("   and a deliberate pause at the end reads the same as padding. Tenths of a")
    print("   second are the arm coming to rest, not the defect.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
