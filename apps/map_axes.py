#!/usr/bin/env python3
"""⭐ Decide which SpaceMouse direction drives which robot motion. NO HARDWARE.

    uv run apps/map_axes.py

⛔ THIS CANNOT MOVE THE ARM. It opens no CAN bus, builds no robot, enables no
motor and imports nothing from `yam_can` or `yam_robot`. It reads the SpaceMouse
and writes `config/spacemouse_map.json`. Run it with the arms unplugged, at a
café, anywhere.

WHY THIS IS A SEPARATE TOOL AND NOT JUST A MODE IN THE SESSION
--------------------------------------------------------------
Julien asked for the ability to choose *"which spacemouse control direction
controls what part of the arm, and then testing it in realtime"*. The realtime
part is the reason this is standalone: the mapping is a property of the **input
device**, so it can be dialled in and verified with the arm switched off, and only
then carried to the bench. Dialling it while a 4.3 kg arm is energised means every
experiment costs attention that should be on the arm.

`teleop_session.py` still has a MAP mode for touch-ups mid-drive. This is for the
first, deliberate pass.

WHAT YOU SEE
------------
Every cycle it prints what the puck reports **and how the arm would interpret it**
— the same metres per second the session will command, from the same constants. So
pushing the puck and reading "UP +0.074 m/s" is a complete test of the mapping,
with nothing at risk.

⚠️ It seizes the SpaceMouse from macOS while it runs (`src/yam/inputs/spacemouse.py` explains
why, and why the hands-off countdown is not optional). Your cursor will stop
responding to the puck. That is intended, and it is restored on exit.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parent.parent

from yam.inputs.axis_map import (  # noqa: E402
    DEFAULT_ANGULAR_SCALE,
    DEFAULT_LINEAR_SCALE,
    N,
    PUCK_AXES,
    ROBOT_MOTIONS,
    UNBOUND,
    AxisMap,
    GestureDetector,
    ambiguity_note,
    axes_readout,
)
from yam.inputs.keyboard import KeyReader  # noqa: E402
from yam.inputs.spacemouse import (  # noqa: E402
    TwistReader,
    countdown_hands_off,
    open_device,
    pick_device_by_wiggle,
)

MAP_FILE = REPO / "config" / "spacemouse_map.json"
POLL_HZ = 60.0

HELP = """
  SELECT    1 2 3 4 5 6   pick the robot motion to bind (1=X 2=Y 3=UP 4=ROLL 5=PITCH 6=YAW)
  BIND      move the puck the way you want the SELECTED motion to go POSITIVE
  EDIT      f  flip the selected motion      u  unbind it       n  next motion
  UNDO      0  revert the whole map to how it was when this started
  FINISH    q  SAVE and quit                 d  DISCARD and quit        ?  this help
"""


def reference_table() -> str:
    """What the six robot motions physically are — measured, not assumed.

    The numbers behind these labels were produced by integrating a unit twist in
    simulation (see `src/yam/inputs/axis_map.py`). "Forward" and "left" are deliberately not
    claimed: they depend on how the arm is turned on the desk, which no file here
    records.
    """
    lines = [
        "  the six robot motions, in the WORLD frame (so they do not change when the wrist turns):",
        "",
    ]
    for i, m in enumerate(ROBOT_MOTIONS):
        lines.append(f"    {i + 1}  {m['short']:<5} {m['world']:<10}  {m['note']}")
    lines += [
        "",
        "  ⚠️ +X and +Y are horizontal. Which one points away from YOU depends on how the",
        "     arm is turned on the desk — that is not recorded anywhere, so bind them by",
        "     watching the arm later, or just pick and flip if it feels wrong.",
    ]
    return "\n".join(lines)


def interpretation(m: AxisMap, axes: list[float], linear: float, angular: float) -> str:
    """What the arm would actually do with this puck reading, right now."""
    mapped = m.apply(axes)
    parts = []
    for i, v in enumerate(mapped):
        if abs(v) < 1e-9:
            continue
        mo = ROBOT_MOTIONS[i]
        if i < 3:
            parts.append(f"{mo['short']} {v * linear:+.3f} m/s")
        else:
            parts.append(f"{mo['short']} {np.degrees(v * angular):+.1f}°/s")
    if not parts:
        return "arm would not move"
    return "   ".join(parts)


def main() -> int:
    ap = argparse.ArgumentParser(description="Dial in the SpaceMouse→robot axis map. No hardware.")
    ap.add_argument("--linear-scale", type=float, default=DEFAULT_LINEAR_SCALE,
                    help="m/s at full deflection, only used to report speeds")
    ap.add_argument("--angular-scale", type=float, default=DEFAULT_ANGULAR_SCALE,
                    help="rad/s at full deflection, only used to report speeds")
    args = ap.parse_args()

    original = AxisMap.load(MAP_FILE)
    m = original.copy()

    print("=== SpaceMouse → robot axis map ===\n")
    print(reference_table())
    print("\n  current map:")
    print(m.describe())
    print(HELP)
    print("  ⛔ This tool cannot move the arm. No CAN bus is opened and no motor is enabled.\n")

    info = pick_device_by_wiggle(label="the axis map")
    if info is None:
        print("No SpaceMouse found (or none was moved). Nothing changed.")
        return 1
    countdown_hands_off(3)
    handle = open_device(info)
    handle.set_nonblocking(True)
    reader = TwistReader(handle)

    detector = GestureDetector()
    selected = 0
    saved = False
    dt = 1.0 / POLL_HZ
    last_note = ""
    next_note_at = 0.0

    print(f"\n⭐ SELECTED: {ROBOT_MOTIONS[selected]['short']} "
          f"— move the puck the way you want it to go POSITIVE "
          f"({ROBOT_MOTIONS[selected]['note']})\n")

    try:
        with KeyReader() as keys:
            if not keys.enabled:
                print("⚠️  stdin is not a terminal, so keys will not work. Ctrl-C to stop.\n")
            while True:
                now = time.perf_counter()
                axes = reader.read()

                for k in keys.drain():
                    if k == "q":
                        m.save(MAP_FILE)
                        saved = True
                        raise KeyboardInterrupt
                    if k == "d":
                        raise KeyboardInterrupt
                    if k in "123456":
                        selected = int(k) - 1
                        detector.reset()
                        mo = ROBOT_MOTIONS[selected]
                        print(f"\n⭐ SELECTED: {mo['short']} ({mo['world']}) — {mo['note']}")
                        print(f"   move the puck the way you want {mo['short']} to go POSITIVE\n")
                    elif k == "f":
                        m.flip(selected)
                        print(f"\n  flipped {ROBOT_MOTIONS[selected]['short']} → {m.row(selected).strip()}\n")
                    elif k == "u":
                        m.unbind(selected)
                        print(f"\n  unbound {ROBOT_MOTIONS[selected]['short']} — it will not move\n")
                    elif k == "n":
                        selected = (selected + 1) % N
                        detector.reset()
                        mo = ROBOT_MOTIONS[selected]
                        print(f"\n⭐ SELECTED: {mo['short']} ({mo['world']}) — {mo['note']}\n")
                    elif k == "0":
                        m = original.copy()
                        print("\n  reverted to the map this session started with:")
                        print(m.describe() + "\n")
                    elif k == "?":
                        print(reference_table())
                        print(HELP)
                        print(m.describe() + "\n")
                    elif k.isprintable() and k.strip():
                        print(f"\n  (key {k!r} does nothing — press ? for the list)\n")

                # ---- binding by gesture ----------------------------------
                hit = detector.feed(axes, now)
                if hit is not None:
                    puck_axis, value = hit
                    displaced = m.bind(selected, puck_axis, value)
                    mo = ROBOT_MOTIONS[selected]
                    print(f"\n  ✓ {mo['short']} ← puck {PUCK_AXES[puck_axis]} "
                          f"{'+' if value > 0 else '−'}   "
                          f"(you moved {PUCK_AXES[puck_axis]} to {value:+.2f})")
                    if displaced is not None:
                        dm = ROBOT_MOTIONS[displaced]
                        print(f"  ⚠️  puck {PUCK_AXES[puck_axis]} was driving {dm['short']}, so "
                              f"{dm['short']} is now UNBOUND.")
                        print(f"      Press {displaced + 1} and bind it to something else, or leave it dead.")
                    selected = (selected + 1) % N
                    nxt = ROBOT_MOTIONS[selected]
                    print(f"\n⭐ SELECTED: {nxt['short']} ({nxt['world']}) — {nxt['note']}\n")
                    detector.reset()
                elif now >= next_note_at:
                    note = ambiguity_note(axes)
                    if note and note != last_note:
                        print(f"\n  … {note}\n")
                        last_note = note
                        next_note_at = now + 1.5

                # ---- live readout ----------------------------------------
                sel = ROBOT_MOTIONS[selected]["short"]
                print(f"\r[{sel:>5}] {axes_readout(axes)}  →  "
                      f"{interpretation(m, axes, args.linear_scale, args.angular_scale)}"
                      f"{' ' * 12}", end="", flush=True)

                time.sleep(max(0.0, dt - (time.perf_counter() - now)))
    except KeyboardInterrupt:
        pass
    finally:
        try:
            handle.close()
        except Exception:  # noqa: BLE001, S110
            pass

    print("\n\n=== final map ===")
    print(m.describe())
    if m == original:
        print("\n  unchanged.")
    elif saved:
        print(f"\n  ✅ saved → {MAP_FILE.relative_to(REPO)}")
        print(f"     was:  {original.one_line()}")
        print(f"     now:  {m.one_line()}")
    else:
        print("\n  ⚠️  DISCARDED — nothing was written. The file still holds:")
        print(f"     {original.one_line()}")

    if m.unbound():
        names = ", ".join(ROBOT_MOTIONS[i]["short"] for i in m.unbound())
        print(f"\n  ⚠️  These motions are UNBOUND and the arm will not perform them: {names}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
