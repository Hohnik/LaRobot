#!/usr/bin/env python3
"""Report what is in `recordings/`, and whether any of it carries the §30.1 padding.

    uv run scripts/check_recordings.py
    uv run scripts/check_recordings.py --dir recordings --still 0.05

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
sys.path.insert(0, str(REPO / "src"))

from recording import Layout, Trajectory  # noqa: E402

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
    if method == "live:hold" and peak_speed > HOLD_SPEED_S:
        return f"{text}  ⛔ implausible", "implausible"
    return text, None


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--dir", default="recordings", help="where the .json files are")
    ap.add_argument("--still", type=float, default=0.05,
                    help="rad/s below which a joint counts as not moving (default 0.05, "
                         "chosen above the 0.032-0.038 wobble floor of a held arm)")
    args = ap.parse_args()

    folder = REPO / args.dir if not Path(args.dir).is_absolute() else Path(args.dir)
    files = sorted(folder.glob("*.json"))
    if not files:
        print(f"no recordings in {folder}")
        return 0

    # ⭐ `arms` is a column since 2026-08-14, when a recording became able to hold more
    # than one arm. A two-arm file and a one-arm file look identical in every other column,
    # and playing the wrong one into the wrong session is the mistake worth making visible.
    print(f"{'file':>9} {'arms':>6} {'commit':>9} {'recorded':>17} {'dur':>7} "
          f"{'padding':>9} {'share':>6} {'peak p99':>9}  {'how it was made':<22}")
    padded: list[str] = []
    contradictory: list[str] = []
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
            padded.append(path.name)
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
        if fault:
            contradictory.append(path.name)
        # ⚠️ Read through `Layout.from_meta`, which also understands files written before
        # the layout existed: they carry a single `arm` field and nothing else.
        layout = Layout.from_meta(traj.meta, traj.n_joints)
        arms_col = ",".join(layout.arms)
        print(f"{path.name:>9} {arms_col:>6} {str(traj.meta.get('commit', '?')):>9} "
              f"{str(traj.meta.get('recorded_at', '?'))[:16]:>17} "
              f"{traj.duration:6.2f}s {pad:8.2f}s {share:5.1f}% "
              f"{traj.joint_speed(99):8.2f}{flag}  {method:<22}")

    print()
    if contradictory:
        print(f"⛔ {len(contradictory)} file(s) are labelled `live:hold` yet moved faster than "
              f"{HOLD_SPEED_S} rad/s: {', '.join(contradictory)}.")
        print("   HOLD commands the arm to stay put, so that label and that speed disagree.")
        print("   The known cause is the FINDINGS §35.4 defect: `method` was written when the")
        print("   recording started and the mode was changed afterwards, so a movement guided")
        print("   by hand came out stamped as HOLD. The data is fine; the label is not.")
        print()

    pre_fix = [p.name for p in files
               if not (Trajectory.load(p).meta.get("modes"))]
    if pre_fix:
        print(f"⚠️ {len(pre_fix)} file(s) carry no `modes` field: {', '.join(pre_fix)}.")
        print("   Those were recorded before the FINDINGS §35.4 provenance fix, so their")
        print("   `method` names only the mode the recording STARTED in. Not a fault in the")
        print("   data, and it does mean the label cannot be trusted for those files.")
        print()
    if padded:
        print(f"⛔ {len(padded)} of {len(files)} carry more than {PADDING_S:.1f}s of "
              f"padding: {', '.join(padded)}")
        print("   That is the FINDINGS §30.1 defect. Re-record those; it takes seconds.")
    else:
        print(f"✓ none of the {len(files)} files carry more than {PADDING_S:.1f}s of "
              f"trailing still time.")
    print("\n⚠️ Padding is measured at the END only, so a pause in the middle is invisible,")
    print("   and a deliberate pause at the end reads the same as padding. Tenths of a")
    print("   second are the arm coming to rest, not the defect.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
