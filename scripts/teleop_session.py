#!/usr/bin/env python3
"""⭐ Interactive arm session: switch between hand-guiding and SpaceMouse at will.

    uv run scripts/teleop_session.py            # dry run: explains the keys
    uv run scripts/teleop_session.py --yes      # LIVE

⛔ MOVES THE WHOLE ARM. Desk clear, hand near the power.

    KEY   MODE
    g     GUIDE   — zero gravity. The arm is weightless; push it where you like.
    t     TELEOP  — the SpaceMouse drives the end effector.
    h     HOLD    — the arm holds its current pose. The safe idle.
    p     PARK    — slowly drive back to the saved park pose.
    m     CONTROLS — set up the mouse. The arm MOVES, one isolated axis, half speed.
    s     save the current pose as the park pose.
    x/y/z flip that motion of the SpaceMouse mapping (saved on exit).
    +/-   faster / slower.
    ?     print this again.
    q     QUIT — goes to HOLD first and asks; it never just releases the arm.
    ^C    the SAME as q — stops and asks. Press it twice to force a real quit.

⛔ CONTROLS mode is where the axis map gets set up, ON THE ARM, and that is not a
convenience. An earlier version held the arm still and this docstring recommended
`scripts/map_axes.py` for the "first dial-in… with the arms unplugged". Julien
showed that was wrong: **you cannot decide a direction is wrong until you have
watched the arm go that way.** The map is not a property of the input device — it
is a property of the device *and* how the arm is turned on the desk, and only one
of those is in a file. `map_axes.py` remains useful for sign tweaks away from the
bench; it cannot tell you what a direction *is*.

WHY IT IS ONE SESSION AND NOT A SEQUENCE OF SCRIPTS
---------------------------------------------------
Julien, after the first run: *"it shouldn't be in phases. It should be more like
going forward and backward. I should be able to control when the weightless or
SpaceMouse-controlled things are happening."* He is right, and there is a safety
argument for it beyond convenience: **zero gravity cancels the arm's weight, so
the instant any process stops commanding, the weight is back.** Every gap between
scripts is a moment when a raised arm sags. One long-lived session with mode
switches has no gaps.

FOUR REAL FAILURES FROM THE PREVIOUS RUN, ALL FIXED HERE
--------------------------------------------------------
1. **The control thread died and the loop carried on regardless.** Motor 7 hit an
   over-temperature fault at t≈24 s; I2RT's control thread raised and exited. The
   teleop loop kept solving IK and kept calling `command_joint_pos` into a dead
   robot for another **64 seconds**, printing plausible EE numbers the whole time
   while the arm did nothing. Julien saw exactly that. **Now every cycle checks
   the chain is alive and stops the instant it is not.** A loop that cannot tell
   whether its commands are arriving is worse than one that crashes.
2. **The arm drooped when the thread died.** With no commands arriving, the
   motors' own 400 ms timeout damps them — so the arm sank slowly under gravity
   rather than holding. Now the session detects the death immediately and says so
   loudly, so a human can catch it while it is still a slow sag.
3. **Motor over-temperature came as a surprise.** Temperatures are now read every
   cycle from the chain, shown live, warned at 55 °C and stopped at 65 °C — below
   the firmware's own trip, so the session ends in a controlled way instead of
   the thread dying underneath it.
4. **Quitting released the arm on a timer.** A 5 s countdown is not consent. Now
   `q` moves to HOLD and waits for an explicit second key. ⚠️ **And that fix had a
   hole in it for two days: Ctrl-C went around the consent flow entirely** and
   disabled the motors from `finally`. Fixed 2026-08-12 — the first Ctrl-C is now
   the same request `q` is, the second forces the quit.

ON RECOVERING A DROOPED ARM — yes, and nothing is lost
------------------------------------------------------
The encoders report true joint positions at all times; that is why a hand-twist
of the gripper this morning read back exactly. When the arm drooped, the system
did **not** lose track of where it was — it lost the loop that was commanding it.
So PARK is safe: it reads the true current pose, then interpolates slowly to the
saved one. It is not dead reckoning and it cannot be "miscalibrated" by a droop.
⚠️ What it cannot know is what is now in the way, so it moves slowly and can be
stopped with any key.
"""

from __future__ import annotations

import argparse
import json
import signal
import subprocess
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "src"))
sys.path.insert(0, str(REPO / "third_party" / "i2rt"))
from axis_map import (  # noqa: E402
    DEFAULT_ANGULAR_SCALE,
    DEFAULT_LINEAR_SCALE,
    PUCK_AXES,
    ROBOT_MOTIONS,
    motions_for,
    AxisMap,
    axes_readout,
    isolate,
    isolated_axes,
)
from axis_map import AxisMapStore  # noqa: E402
from axis_map import N as N_AXES  # noqa: E402
from keyboard import KeyReader  # noqa: E402
from spacemouse import (  # noqa: E402
    TwistReader,
    countdown_hands_off,
    open_device,
    pick_device_by_wiggle,
)
from motion import EASINGS, JointPath, easing_factor  # noqa: E402
from recording import TrackingLog, Trajectory, replay_step, safe_time_scale  # noqa: E402
from screen import StatusLine  # noqa: E402
from teleop import FRAMES, CartesianTeleop  # noqa: E402
from yam_can import ARM_SERIALS, DEFAULT_ARM, YAM_JOINTS  # noqa: E402
from yam_robot import (  # noqa: E402
    ThermalGuard,
    advance_park_command,
    build_robot,
    motor_temperatures,
    park_slots,
    park_target_from,
    park_verdict,
    resolve_park_legs,
    shutdown_robot,
    with_park_slot,
)

CONTROL_HZ = 100.0
N_ARM = 6

# Faster than the first run, which Julien found "very slow". Still well short of
# what the hardware can do — this is a human-in-the-loop speed, not a limit.
# ⭐ Defined in src/axis_map.py so `scripts/map_axes.py` reports the exact speeds
# this session commands. Dialling a mapping against speeds the arm does not use
# would teach the wrong feel.
LINEAR_SCALE = DEFAULT_LINEAR_SCALE     # m/s at full deflection  (was 0.04)
ANGULAR_SCALE = DEFAULT_ANGULAR_SCALE   # rad/s at full deflection (was 0.25)

WORKSPACE_BOX = 0.30
MAX_JOINT_STEP = 0.015     # rad/cycle ≈ 1.5 rad/s at 100 Hz
JOINT_LIMIT_MARGIN = 0.08
TEMP_WARN = 55.0
TEMP_STOP = 65.0
PARK_SPEED = 0.40          # rad/s per joint when driving to the park pose
# ⭐ Slot "0" in the UI. The one pose Ctrl-C returns to before releasing the motors,
# and the only one `s 0` may overwrite — see where it is loaded for why that matters.
BASE_SLOT = "default"
# ⭐ Ease in and out over this much joint travel. A constant-rate park starts and
# stops with a jerk; with sequences that jerk lands at every waypoint. 0.20 rad is
# ~half a second of ramp at the default 0.4 rad/s, and a move shorter than twice it
# simply never reaches full speed. `--no-smooth` sets it to 0.
PARK_RAMP = 0.20
# ⭐ How much the path may cut a corner, in radians of the fastest joint. `sharp`
# reproduces the old stop-at-every-waypoint behaviour exactly. Julien's words for what
# the others are for: *"instead of moving and then jittering ninety degrees to the next
# side, in a smooth curve it would go to the next point."*
BLEND_MODES = [("sharp", 0.0), ("smooth", 0.15), ("flowing", 0.35)]
# ⭐⭐ THE SAME PAIR OF KEYS, REACHABLE ON A GERMAN KEYBOARD. Julien, 2026-08-12:
# *"I don't like the fact that the brackets are used because I have a German
# keyboard, and they're awkward to reach. Maybe ä and ö could be used."*
#
# He is right, and it is worse than awkward: on a German QWERTZ layout `[` and `]` are
# **AltGr + 8** and **AltGr + 9** — a three-finger chord, on a rig whose whole input
# design rule is "no shift keys", for a knob he adjusts while 4.3 kg is moving.
#
# ⭐ `ö` and `ä` are single unshifted keys, adjacent, on the home row immediately right
# of `L` — the physical position US layouts give to `;` and `'`. They are the best pair
# available: `ü` and `+` are the other unshifted candidates and `+` already means
# "faster", while `^` and `´` are dead keys that emit nothing until a second press.
#
# ⚠️ ALIASES, NOT REPLACEMENTS. `[` and `]` keep working — they are in every doc, and
# a US keyboard (a colleague's, or a clone of this repo) must not lose the feature. Two
# spellings of one key is cheap; a key that exists only on one person's laptop is not.
#
# ⛔ These are non-ASCII, so they only arrive at all because `KeyReader` now decodes
# UTF-8 across reads — `ö` is two bytes and the old one-byte reader turned it into two
# replacement characters. See `src/keyboard.py::_refill`.
KEY_STEP_DOWN = ("[", "ö")          # shorter ease ramp · smaller gripper step
KEY_STEP_UP = ("]", "ä")            # longer ease ramp  · bigger gripper step
# ⚠️ `m` is absent on purpose: CONTROLS owns the keyboard while it is active, so `m`
# pressed inside it is handled by that branch and never reaches the check that uses this.
MODE_KEYS = {"g": "GUIDE", "t": "TELEOP", "h": "HOLD"}
# ⛔ Do not let the commanded cursor run further than this ahead of the arm. The
# trajectory is a SHAPE now, and a command that races ahead while the arm cuts its own
# corner is not the shape anyone chose. SafeRobot's 0.25 rad lag limit is the backstop
# below this; this keeps the path faithful rather than merely safe.
MAX_CURSOR_LAG = 0.15
# ⭐ Two DIFFERENT patiences, and separating them removed a four-second dead wait at
# the end of every park. "Has the controller finished settling?" is answered in a
# fraction of a second; "is something blocking the arm?" deserves four. They used to
# share PARK_STALL_SECONDS, so every park that finished outside the 0.02 tolerance —
# which is most of them — sat apparently doing nothing before admitting it had
# arrived. Julien: *"it moves millimetre by millimetre really slowly."* He was
# watching a wait, not a crawl.
PARK_SETTLE_SECONDS = 0.5
# Judged on the MEASURED pose, so it must allow for a position controller's
# steady-state error. 0.02 rad is 1.1°, which is "arrived" for parking.
PARK_TOLERANCE = 0.02
# ⭐ "Close enough that what is left is the controller, not an obstruction."
# MEASURED on hardware 2026-08-12: the same arm parking to the same pose reported
# 0.020 rad off (pass) and 0.021 rad off (stall) in consecutive sessions, because a
# position-controlled arm settles a fraction of a degree short under its own weight.
# 0.02 rad sits ON that noise floor rather than above it. See yam_robot.park_verdict:
# the answer is not a bigger tolerance — it is a second threshold, so that stopping
# CLOSE is success while stopping FAR is still an obstruction.
PARK_SETTLED = 0.06        # rad, 3.4°
# If the measured error stops improving for this long, PARK says so and holds,
# rather than printing a number that is not changing. That silence is precisely how
# the treadmill bug survived two sessions.
PARK_STALL_SECONDS = 4.0
PARK_PROGRESS_EPS = 0.003  # rad of improvement that counts as "still making progress"

# CONTROLS mode drives the arm at this fraction of the teleop speed. It is the mode
# you experiment in, with a mapping you have not yet confirmed, so a wrong direction
# should be a slow wrong direction.
CONTROLS_SCALE = 0.5

# ⛔ NEVER command the gripper to 0.0 or 1.0. Those are the mechanical stops, and
# holding a position AT a stop is stall torque: full current, no motion, no
# cooling. That is what cooked motor 7 twice on 2026-08-10 -- the arm was simply
# told to "hold where you are" while the jaws happened to be resting on a stop.
# Keeping the command inside this band means the jaws are always free to move,
# so a hold command costs almost no torque.
# ⛔ These are only applied to values the OPERATOR asks for. The gripper is never
# forced away from where it already is. The earlier [0.15, 0.85] clamp was applied
# on entering TELEOP, which meant that if the jaws happened to sit outside the band
# the session COMMANDED THEM TO MOVE the moment teleop began -- a motion nobody
# asked for, into a mechanical stop when the limits were also mis-framed.
GRIPPER_MIN = 0.02
GRIPPER_MAX = 0.98
GRIPPER_STEP = 0.02        # per keypress
# Hold-to-move rate for the puck buttons. A gripper wants squeeze-and-hold, not a
# staircase of keypresses. 0.6/s crosses the whole normalised stroke in ~1.6 s,
# which is deliberate and slow: the jaws close on real objects, and the stall guard
# should be a backstop rather than the thing that routinely stops you.
GRIPPER_BUTTON_RATE = 0.6  # normalised units per second while a button is held

# Gripper stall guard. Catches the CAUSE (jaws pushing against something they
# cannot move) rather than the symptom (temperature). Torque high while velocity
# is ~0 is the definition of a stall, and stall is the worst thermal case there
# is: full current, no motion, no cooling.
GRIPPER_STALL_TORQUE = 1.0   # Nm
GRIPPER_STALL_VEL = 0.05     # rad/s
GRIPPER_STALL_SECONDS = 0.4

MAP_FILE = REPO / "config" / "spacemouse_map.json"
BACKUP_FILE = REPO / "config" / "spacemouse_map.prev.json"
PARK_FILE = REPO / "config" / "park_pose.json"
# ⭐ Hand-taught movements live OUTSIDE config/, and the distinction is deliberate.
# `config/` holds measured calibration that the code refuses to start without, so it is
# tracked in git. A recording is *data*: a five-minute one is megabytes, there will be
# hundreds, and losing one costs a minute of re-teaching rather than a session of
# re-calibrating. Gitignored, and the dataset itself will live somewhere else again
# (ROADMAP step 5).
TAKES_DIR = REPO / "recordings"
# ⭐ One file per playback, named with a timestamp so nothing is ever overwritten. The
# recordings themselves are saved by slot digit and DO overwrite, which lost the two files
# an earlier measurement was taken from ([FINDINGS §33.2](../docs/FINDINGS.md)).
TRACKING_DIR = TAKES_DIR / "tracking"
# ⭐⭐ ONE CEILING FOR EVERY PLANNED MOTION, in radians per second for a single joint.
# Julien asked for this on 2026-08-13: *"max speed would just be limited by the actual
# safety things we have or the motors. Maybe we need an extra system for max speeds in
# general."* He is right, and the number already existed in the file twice over: TELEOP
# clamps the commanded joint change to MAX_JOINT_STEP per cycle, which at CONTROL_HZ is
# exactly this speed. Deriving it keeps the two from drifting apart, and it means a change
# to the teleop clamp automatically applies to playback.
#
# ⚠️ WHAT THIS IS NOT: a measurement of how fast the arm can actually track a command.
# Nobody has measured that on this rig. The evidence so far says it is LOWER — at 0.26x of
# a 2.67 rad/s recording, so ~0.7 rad/s commanded, the arm was already 0.105 rad behind
# against a 0.15 limit. So treat this as "the fastest we allow ourselves to ask for", and
# see ROADMAP §6.6 for the measurement that would replace it.
MAX_PLANNED_JOINT_SPEED = MAX_JOINT_STEP * CONTROL_HZ
# ⚠️ How long one recording may run before it stops itself. ~16 minutes at 100 Hz, which
# is well past the ~4.5 minutes of context a long-horizon policy wants (ROADMAP §9.3).
# It exists because nothing else would ever stop a recording, and an unbounded list in a
# process that is driving an arm is a memory problem waiting for the worst moment.
MAX_TAKE_SAMPLES = 100_000

HELP = """
  MODES     g GUIDE (weightless)   t TELEOP   h HOLD
  POSES     s then 0-9  SAVE here (0 = the BASE pose Ctrl-C returns to, 1-9 waypoints)
            p then Enter          drive to the base pose
            p then 1 then Enter   drive to waypoint 1
            p then 1 2 3 Enter    ONE smooth motion through all three, Enter again to go
            while choosing OR moving:  - / + speed   , / . corners   ö / ä  ease length
  TAKES     w  record a movement (any mode; GUIDE is the point). w again stops, then 0-9 saves
            l then 0-9   play a recording back — shows the plan, Enter runs it
  EASE      e  profile   ö / ä  how long   — shapes p runs and Ctrl-C, nothing else
            (gripper step is --gripper-step now, not a live key)
  DIRECTION x y z  flip translation axis      1 2 3  flip rotation axis (roll/pitch/yaw)
  CONTROLS  m  set up the mouse — the arm MOVES, one isolated axis, half speed
  SPEED     - / +  linear             , / .  rotation
  GRIPPER   o open   c close          b  assign the PUCK BUTTONS (hold to move jaws)
  FRAME     v  world / tool / camera — what "forward" means (tool = follows the wrist)
  OTHER     r  wrist rotation on/off   ?  help    q  QUIT → then p park, g guide, d disable
"""

MAP_HELP = """
  ⭐ CONTROLS MODE — the arm DOES move, but only along the ONE axis you push hardest,
     at half speed. Moving the puck NEVER changes the map; only the keys below do.
  DRIVE     push the puck — the strongest direction wins, so the motion is unambiguous
  REVERSE   f   flip the direction of the control you just used   ← the main one
  SWAP      1 2 3 4 5 6   EXCHANGE the control you just used with that motion's
                          (1=X 2=Y 3=UP 4=ROLL 5=PITCH 6=YAW). Both move, so nothing
                          is left unbound — and the same key again swaps back
  UNBIND    u   the control you just used drives nothing
  BUTTONS   b   assign the two puck buttons to gripper OPEN / CLOSE (press them)
                then f swaps them, same as it reverses an axis. Hold to move
  SPEED     - / +  linear          , / .  rotation          r  rotation on/off
  UNDO      0   revert the whole map to how it was when this session started
  LEAVE     t TELEOP   g GUIDE   h HOLD   m HOLD        ?  this help
"""


def map_reference(frame: str = "world") -> str:
    """What the six motions physically are. Measured in simulation, not assumed —
    see `src/axis_map.py` for the numbers and for why "forward" is not claimed."""
    lines = ["  the six motions, in the WORLD frame (they do not change when the wrist turns):"]
    for i, m in enumerate(motions_for(frame)):
        lines.append(f"    {i + 1}  {m['short']:<5} {m['world']:<10}  {m['note']}")
    return "\n".join(lines)


def load_json(path: Path, default):  # noqa: ANN001, ANN201
    try:
        return json.loads(path.read_text()) if path.exists() else default
    except Exception:  # noqa: BLE001
        return default


def save_json(path: Path, data) -> None:  # noqa: ANN001
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2) + "\n")


def ease_note(profile: str, ramp: float) -> str:
    """The one-line answer to *"what does easing even do here?"*

    ⭐ It names where the effect lives, because that is the question Julien actually asked
    on the arm: *"the easing outside of parking, I don't really know what that means. Does
    it work for recording, or does it work for teleoperating?"* Neither. Easing shapes how
    a **planned** move starts and stops, which means `p` runs and the Ctrl-C park, and
    nothing else. Driving by hand has no plan to shape, and a playback follows the timing
    it was taught rather than an eased ramp.
    """
    tail = "off" if ramp <= 0 else f"over {ramp:.2f} rad"
    return f"ease {profile} {tail} · affects p runs and Ctrl-C only · ö/ä = how long"


def git_commit() -> str:
    """Short hash of the code that is running, or `"unknown"`.

    ⭐ Written into every recording. Julien's requirement, 2026-08-12: *"being able to
    reproduce everything and connect it to other research papers."* Which version of the
    code produced a demonstration is free to record now and unrecoverable later.
    ⚠️ Never raises: a missing git is not a reason to lose a recording.
    """
    try:
        out = subprocess.run(["git", "-C", str(REPO), "rev-parse", "--short", "HEAD"],
                             capture_output=True, text=True, timeout=5, check=False)
        return out.stdout.strip() or "unknown"
    except Exception:  # noqa: BLE001
        return "unknown"


def dt_now() -> str:
    """Wall-clock time as text, for the record. Local time, because a human reads it."""
    return datetime.now().astimezone().isoformat(timespec="seconds")


def chain_alive(robot) -> bool:  # noqa: ANN001
    """Is the robot still actually being commanded?

    ⛔ The single most important check in this file. I2RT's control thread raises
    and exits on a motor fault; nothing tells the caller. Without this, the loop
    keeps issuing commands into a corpse and reporting healthy-looking numbers,
    which is what happened for 64 s on 2026-08-10.
    """
    chain = getattr(robot, "motor_chain", None)
    if chain is None:
        return False
    return bool(getattr(chain, "running", False))


_SHUTTING_DOWN = {"yes": False}


def _quiet_expected_server_exit(args) -> None:  # noqa: ANN001
    """Silence ONE known, expected traceback — and only while we are shutting down.

    ⛔ The noise this removes. Every clean exit printed:

        Exception in thread robot_server:
        RuntimeError: … motor_chain_robot's motor chain is not running, exiting the
        robot server

    …immediately before `motors confirmed disabled: [1, 2, 3, 4, 5, 6, 7]`. It is the
    I2RT SDK's background server thread noticing the chain has stopped — **because we
    stopped it**. Nothing is wrong, and the shutdown it appears to indict has in fact
    succeeded.

    ⚠️ Why bother, when it is harmless? Because a scary traceback printed on every
    successful exit is a training exercise in ignoring tracebacks, and this project
    depends on people reading the ones that matter. FINDINGS §0 is a catalogue of
    failures that looked calm; the inverse — a success that looks like a failure — has
    the same cost, paid in attention.

    ⛔ Deliberately narrow, because blanket exception-swallowing is the other half of
    that catalogue: it fires only during our own shutdown, only for that thread, only
    for `RuntimeError`, and only for that message. Anything else goes to the real hook
    and prints in full.
    """
    if (_SHUTTING_DOWN["yes"]
            and getattr(args, "thread", None) is not None
            and args.thread.name == "robot_server"
            and args.exc_type is RuntimeError
            and "motor chain is not running" in str(args.exc_value)):
        print("  (the SDK's robot_server thread exited because we stopped the chain — expected)")
        return
    threading.__excepthook__(args)


def park_and_wait(robot, keys, park, clamp_gripper, ramp: float = PARK_RAMP,
                  speed: float = PARK_SPEED, easing=EASINGS[2]) -> str:  # noqa: ANN001
    """Drive to the park pose, **blocking**, and say how it ended.

    Returns `"arrived"` · `"stalled"` · `"stopped"` (a key was pressed) · `"dead"`.

    ⛔ THE DUPLICATION THIS REMOVES. The quit path used to carry its own copy of this
    loop, and the code's own comment admitted the risk: *"This is a SECOND park loop,
    and duplication is what has bitten this repo four times."* It was bounded
    carefully — same tested `advance_park_command`, same SafeRobot limits — but it
    was still a second place for a fix to miss. Now both callers share this one.

    ⚠️ The *interleaved* park (mode == "park") is deliberately NOT folded in here.
    That one advances a single step per control cycle so the operator can still press
    keys and the temperature guard still runs; this one blocks because the session is
    already ending. Same trajectory maths, different scheduling — collapsing them
    would mean a blocking call inside the 100 Hz loop.
    """
    tgt, warn = park_target_from(robot.get_joint_pos(), park,
                                 gripper_index=N_ARM, clamp=clamp_gripper)
    if warn:
        print(f"\n  ⚠️  {warn}.")
    cmd = np.asarray(robot.get_joint_pos(), dtype=float)
    start = cmd.copy()
    best = float(np.max(np.abs(tgt - cmd)))
    last_progress = time.perf_counter()
    # ⛔ DISCARD ANYTHING TYPED BEFORE THIS MOVE EXISTED. "Any key stops it" must mean
    # a key pressed *at* the moving arm, not one left over from teleop or from the
    # menu that led here. Julien saw a park announce itself and stop in the same
    # breath — the stale keystroke that cancelled it had been typed seconds earlier,
    # and with the KeyReader buffering bug it could have been typed long before that.
    keys.drain()
    print(f"\n⭐ PARKING to {np.round(np.asarray(tgt)[:N_ARM], 2)} — any key stops it.\n")
    while True:
        now = time.perf_counter()
        if not chain_alive(robot):
            print("\n⚠️  the chain died while parking.")
            return "dead"
        meas = np.asarray(robot.get_joint_pos(), dtype=float)
        err = float(np.max(np.abs(tgt - meas)))
        verdict = park_verdict(err, now - last_progress > PARK_STALL_SECONDS,
                               PARK_TOLERANCE, PARK_SETTLED,
                               stopped_briefly=now - last_progress > PARK_SETTLE_SECONDS)
        if verdict == "arrived":
            print(f"\n⭐ PARKED ({err:.3f} rad off).")
            return "arrived"
        if verdict == "settled":
            print(f"\n⭐ PARKED ({err:.3f} rad off — as close as the arm holds itself; "
                  f"the last fraction of a degree is the controller settling under load).")
            return "arrived"
        if verdict == "blocked":
            print(f"\n⛔ PARK BLOCKED — {err:.3f} rad still to go and no progress for "
                  f"{PARK_STALL_SECONDS:.0f}s. Something is in the way, or the pose "
                  "is unreachable.")
            return "stalled"
        if keys.get() is not None:
            print("\n  park stopped.")
            return "stopped"
        if err < best - PARK_PROGRESS_EPS:
            best, last_progress = err, now
        # Same ease-in/ease-out as the interleaved park — Ctrl-C should not shut down
        # with a jerk at both ends when a mid-session park glides.
        # ⭐ EASINGS[2] is "out": full speed from the first step, soft landing.
        # Julien on Ctrl-C: *"I don't want to have to wait until the movement has been
        # smoothed out — I want it to move into its parking position quickly and
        # swiftly, without the excessive starting and pausing."* A shutdown move should
        # leave at once; only the arrival needs to be gentle.
        factor = easing_factor(easing, float(np.max(np.abs(cmd - start))),
                               float(np.max(np.abs(tgt - cmd))), ramp)
        cmd = advance_park_command(cmd, tgt, speed * factor / CONTROL_HZ)
        robot.command_joint_pos(cmd)
        time.sleep(1.0 / CONTROL_HZ)


def main() -> int:  # noqa: PLR0915
    ap = argparse.ArgumentParser(description="Interactive YAM session: guide, teleop, park.")
    ap.add_argument("--yes", action="store_true", help="actually energise the arm")
    ap.add_argument("--arm", default=DEFAULT_ARM, choices=sorted(ARM_SERIALS))
    ap.add_argument("--start-mode", default="guide", choices=["guide", "hold", "teleop"])
    ap.add_argument("--no-gripper", action="store_true",
                    help="run the 6 arm joints only and leave motor 7 free — the escape hatch if the "
                         "gripper misbehaves again")
    ap.add_argument("--no-smooth", action="store_true",
                    help="drive to park poses at a constant rate, as before. By default "
                         "each move eases in and out over 0.2 rad, which matters most "
                         "when running a SEQUENCE of poses — every waypoint is otherwise "
                         "a stop and a start")
    ap.add_argument("--no-rotation", action="store_true",
                    help="start with wrist rotation disabled (toggle live with r)")
    ap.add_argument("--frame", default="world", choices=sorted(FRAMES),
                    help="which frame the puck's directions mean. world = fixed to the desk "
                         "(default, and what was tuned on hardware); tool = attached to the "
                         "gripper, for driving while watching a wrist camera; camera = the "
                         "MODELLED D405 mount, wrong for a hand-mounted webcam. Toggle live with v")
    ap.add_argument("--linear-scale", type=float, default=LINEAR_SCALE)
    ap.add_argument("--gripper-step", type=float, default=GRIPPER_STEP,
                    help="how far o/c move the jaws per press, 0-1 of their travel. "
                         "⭐ A FLAG rather than a live key since 2026-08-13: it used to be "
                         "ö/ä, which now always mean the ease ramp. Julien on the arm: "
                         "changing the gripper step live is 'not necessary currently and "
                         "all the time', and sharing the keys with the ease ramp meant a "
                         "message told him to press keys that did something else")
    ap.add_argument("--box", type=float, default=WORKSPACE_BOX)
    ap.add_argument("--fork-map", action="store_true",
                    help="give THIS arm its own axis map, copied from the one it uses now. "
                         "Without this, both arms share one map and editing changes both")
    ap.add_argument("--share-map", action="store_true",
                    help="drop this arm's own axis map and go back to the shared one")
    args = ap.parse_args()
    if args.fork_map and args.share_map:
        ap.error("--fork-map and --share-map are opposites; pass at most one")

    # Rotation is ON by default now. Julien: "the gripper cannot be tilted
    # currently and cannot be twisted". It was off for the first hardware run
    # because a wrong rotation sign swings the wrist while a wrong translation
    # sign only nudges — that caution has served its purpose.
    # ⭐⭐ ALL OUTPUT IN THIS SESSION GOES THROUGH ONE STATUS LINE.
    #
    # ⛔ `print` is deliberately shadowed for the whole of main(). Julien changed the
    # park speed six times while choosing a run and got six copies of a two-line plan
    # interleaved with six status lines — *"that seems to be more of a bug."* It was:
    # a `\r` status with no newline and ordinary prints with newlines were fighting
    # over the same row.
    #
    # Shadowing rather than converting ~60 call sites is a deliberate trade. It makes
    # the output policy ONE thing in ONE place, and — the part that matters — a print
    # added later cannot forget to follow it. The rule: `end=""` means "this is the
    # live line, repaint it in place"; anything else scrolls above it. See src/screen.py.
    screen = StatusLine()

    def hint(text: str = "") -> None:
        """A value the operator just changed, on its own live row above the status.

        ⭐ `linear speed → 0.188 m/s` printed as a MESSAGE six times is six rows of
        scrollback saying the same word. As a hint it is one row whose number changes
        — and, crucially, it no longer loses a race with the once-a-second status,
        which is what made a knob change flash up and vanish.
        """
        screen.hint(text)

    def print(*args, sep=" ", end="\n", flush=False):  # noqa: A001, ARG001
        text = sep.join(str(a) for a in args)
        if end == "":
            screen.set(text.lstrip("\r"))
        else:
            screen.say(text)

    threading.excepthook = _quiet_expected_server_exit
    rotation = not args.no_rotation
    control_frame = args.frame
    # ⛔ The store decides WHICH map this arm uses — its own override if it has one,
    # otherwise the shared one. Editing a shared map changes both arms, so the scope
    # is printed in the plan and again at exit. Never leave that implicit.
    map_store = AxisMapStore.load(MAP_FILE)
    if args.fork_map:
        map_store.fork(args.arm)
    elif args.share_map:
        map_store.unfork(args.arm)
    axis_map = map_store.for_arm(args.arm, control_frame)
    map_store_at_start = map_store.copy()
    axis_map_at_start = axis_map.copy()
    # ⭐⭐ THE BASE POSE AND THE WAYPOINTS ARE DIFFERENT THINGS. Julien's ruling,
    # 2026-08-12: *"the control-c park to disable needs to always go back to the stable
    # parking save. If I save a new parking option it shouldn't go back to that and then
    # disable. It should always go back to the base parking option."*
    #
    # ⛔ That is a safety requirement, not a preference. Ctrl-C is the "get me out of
    # here" key: it parks and then RELEASES the motors, so the pose it chooses must be
    # one that is safe to be let go in. A waypoint saved mid-task — arm extended over
    # the desk, gripper holding something — is exactly what that must never be.
    #
    # So `park` (slot 0, the base) is only ever changed deliberately with `s 0`, while
    # `s 1`…`s 9` fill waypoints that Ctrl-C ignores completely.
    slots = park_slots(load_json(PARK_FILE, {}), args.arm)
    park = slots.get(BASE_SLOT)
    park_speed = PARK_SPEED
    # The blended path being followed, the cursor along it, and where each waypoint
    # falls so the readout can say which one it is heading for.
    park_path: JointPath | None = None
    park_s = 0.0
    park_marks: list[tuple[str, float]] = []
    blend_idx = 1                       # "smooth" — the sensible default
    ease_idx = 3                        # "both" — see motion.EASINGS
    # ⛔⭐ TWO CLOCKS, AND CONFLATING THEM PRINTED A WRONG NUMBER FOR A DAY.
    # `park_leg_t` is reset every time the cursor passes a waypoint, because Julien asked
    # for each leg's own duration. `park_start_t` is set once and never reset, because the
    # arrival message wants the whole park. Using `park_leg_t` for both reported
    # **"PARK reached in 0.0s"** on a park that had just taken 4.4 seconds: the last leg's
    # mark is passed at the end of the path, so the reset happened moments before arrival.
    # See [FINDINGS §34.3](../docs/FINDINGS.md).
    park_leg_t = 0.0                    # when the CURRENT LEG started
    park_start_t = 0.0                  # when the whole park started — never reset
    park_ramp = PARK_RAMP               # how much of the move is eased
    # ⭐⭐ HAND-TAUGHT MOVEMENTS. Julien, 2026-08-12: *"one good idea is definitely
    # recording everything in the guide mode and then replaying it. That's a smart idea,
    # definitely."* `w` records, `l` plays one back. The reasoning for why this may beat
    # saved waypoints is docs/ROADMAP.md §6.6; the movement itself lives in
    # src/recording.py so every decision about it is testable without an arm.
    #
    # ⚠️ Only these few names are held here. That is on purpose: this whole block moves
    # into ArmSession when main() is restructured (HANDOFF task 0c), and the fewer locals
    # it owns the smaller that diff is.
    take: Trajectory | None = None      # being recorded right now, or None
    take_to_save: Trajectory | None = None   # frozen, waiting for its slot digit
    take_t0 = 0.0
    take_modes: list[str] = []          # every mode the current recording passed through
    replay: Trajectory | None = None    # being played back right now, or None
    replay_t0 = 0.0
    replay_s = 0.0                      # seconds into the recording, held back on lag
    replay_speed = 1.0
    replay_progress_t = 0.0             # last cycle in which the clock actually moved
    replay_slot = "?"                   # which saved recording is being played
    replay_held_s = 0.0                 # seconds spent waiting for the arm to catch up
    replay_worst_lag = 0.0              # furthest behind the arm ever got, radians
    replay_prev_target: list[float] | None = None
    tracking: TrackingLog | None = None   # per-joint answer to "how fast can it go?"
    replay_pending: Trajectory | None = None   # parked to its start, waiting to run
    # A pending `s` or `p` waiting for its digit, and the sequence being typed after `p`.
    pending: str | None = None
    park_sequence: list[str] = []
    angular_scale = ANGULAR_SCALE
    gripper_step = args.gripper_step
    # CONTROLS mode remembers the last puck axis that actually moved, with no
    # timeout: f and 1-6 act on "the control you just used", and it must still be
    # remembered after the puck has sprung back to centre and his hand has left it.
    last_active_axis: int | None = None
    last_active_value = 0.0
    # "The control you just used" can be an axis OR a puck button, and `f` reverses
    # whichever it was. One key, one meaning.
    last_input_kind: str | None = None      # None | "axis" | "button"
    learn_button: str | None = None         # None | "open" | "close"
    buttons_prev = 0

    print("=== plan ===")
    print(f"  ARM         : {args.arm}  (serial {ARM_SERIALS[args.arm]})")
    print(f"  gripper     : {'NOT controlled — motor 7 left free' if args.no_gripper else 'controlled (o/c), frame-checked at startup'}")
    if args.no_gripper:
        # This is a safety fact and belongs in front of him BEFORE he runs it, not
        # only in the build note. --no-gripper swaps the gravity model, and on
        # 2026-08-10 that dropped the arm in GUIDE mode.
        print("  ⚠️  gravity   : --no-gripper also swaps the DYNAMICS model, so ee_mass=0.695 kg is")
        print("                passed to keep the arm holding itself. Without it the elbow is 39%")
        print("                short and the arm falls in GUIDE. See FINDINGS §11.")
    print(f"  start mode  : {args.start_mode}")
    print(f"  speed       : {args.linear_scale} m/s linear, "
          f"{ANGULAR_SCALE if rotation else 0} rad/s angular  (rotation {'ON' if rotation else 'OFF'}, toggle with r)")
    print(f"  axis map    : {axis_map.one_line(control_frame)}   (m to change it live)")
    print(f"  map scope   : {map_store.scope_note(args.arm)}")
    print(f"  control fr. : {CartesianTeleop.FRAME_NOTES[control_frame]}  (v cycles it live)")
    if axis_map.unbound():
        names = ", ".join(motions_for(control_frame)[i]["short"] for i in axis_map.unbound())
        print(f"  ⚠️  UNBOUND  : {names} — the arm will NOT perform these until they are bound (m)")
    print(f"  park pose   : {np.round(park, 3).tolist() if park else 'none saved — press s to set one'}")
    print(f"  workspace   : ±{args.box} m box, re-centred whenever TELEOP is entered")
    print(f"  temperature : warn {TEMP_WARN}°C, stop {TEMP_STOP}°C")
    print(HELP)

    if not args.yes:
        print("DRY RUN — nothing transmitted, nothing energised. Re-run with --yes.")
        return 0

    info = pick_device_by_wiggle(label=args.arm)
    if info is None:
        print("No SpaceMouse found (or none was moved).")
        return 1
    countdown_hands_off(3)
    handle = open_device(info)
    handle.set_nonblocking(True)
    reader = TwistReader(handle)

    robot = None
    mode = args.start_mode
    stop_reason: str | None = None
    teleop: CartesianTeleop | None = None
    home_ee = None
    gripper_value = 0.0
    park_target = None
    # ⛔ The thermal guard is an object rather than a pair of floats, because
    # "I cannot read the temperature" is a state that has to be tracked and acted
    # on. It used to be indistinguishable from 0 °C — see ThermalGuard.
    thermal = ThermalGuard(warn_at=TEMP_WARN, stop_at=TEMP_STOP)
    hottest: float | None = None
    jaw_temp = None
    stall_since = None
    next_park_report = 0.0
    park_cmd: np.ndarray | None = None      # the park TRAJECTORY, not the measurement
    park_best_err = float("inf")
    park_progress_t = 0.0
    guide_ref: np.ndarray | None = None

    try:
        n_motors = N_ARM if args.no_gripper else N_ARM + 1
        print(f"building robot — enables {n_motors} motors, starts the control loop …")
        robot, note = build_robot(args.arm, zero_gravity=(mode == "guide"),
                                  with_gripper=not args.no_gripper)
        print(f"  {note}\n")
        chain = robot.motor_chain
        prev_q = np.asarray(robot.get_joint_pos(), dtype=float)[:N_ARM]

        # ⭐ DEFAULT PARK POSE = WHEREVER THE ARM STARTED. Julien: *"if the standard
        # set position for park mode is just the starting position, then I can always
        # just press p and then d, and I don't have to do anything with my hands."*
        #
        # This also removes a real dependency he flagged: the two arms no longer have
        # to be physically placed the same way before a session, because each one
        # parks back to its own measured start rather than to a pose recorded from the
        # other. `s` still overrides it, and a saved pose still wins.
        #
        # ⚠️ It is only as good as the pose you start in. Start with the arm drooped
        # and PARK will faithfully return it to drooped — which is why the plan line
        # prints the actual numbers rather than just saying "default".
        startup_pose = np.asarray(robot.get_joint_pos(), dtype=float)
        if park is None:
            park = startup_pose.tolist()
            print(f"  park pose   : none saved — defaulting to the pose the arm is in NOW, "
                  f"{np.round(startup_pose[:N_ARM], 3).tolist()}")
            print("                (press s to set a different one; q then p then d parks and quits)")

        # ⛔ DELIBERATELY NOT RE-CHECKING THE GRIPPER FRAME HERE. Do not add it back.
        #
        # A stale-limit check used to live at this point, and it was worse than
        # nothing for two reasons. It compared the raw jaw position against the
        # *unshifted* limits from the file, so it re-flagged exactly the cases
        # `frame_correct_gripper_limits()` had legitimately reconciled: at the
        # measured raw −1.380 it printed "STALE GRIPPER LIMITS … re-run
        # calibrate_gripper" while the frame was in fact correct and the jaws
        # normalised to 0.3005. And it then **warned and continued** — in the wording
        # of the one rule this project wrote in blood (FINDINGS §3.5) — so the real
        # message and a false alarm were indistinguishable, and the advice it gave
        # was to run a routine that drives the jaws into both mechanical stops.
        #
        # `build_robot()` already gates this twice, better, and BEFORE any control
        # loop starts: it refuses if no ±2π shift reconciles the saved range, and it
        # reads the normalised jaw position back from the runtime and shuts
        # everything down if it is outside [0,1]. Both raise. The `note` printed
        # above carries the verified value ("jaws normalise to 0.030 ✓"), which is
        # also the baseline to watch during the thermal test.

        def clamp_gripper(v: float) -> float:
            return float(np.clip(v, GRIPPER_MIN, GRIPPER_MAX))

        def resync() -> None:
            """⛔ Re-anchor EVERY cached variable to the measured pose.

            This is the fix for the snap Julien saw going GUIDE → TELEOP. `prev_q`
            was initialised once before the loop and only updated inside teleop,
            so after hand-guiding the arm it still held the pose from minutes
            earlier. The very first teleop cycle then computed
            `clip(q_target - prev_q)` and commanded `prev_q + 0.015` — i.e. it
            aimed the arm at where it USED to be, snapped there, and walked back
            at 1.5 rad/s. Exactly what he described.

            The general rule, and the reason this is its own function called from
            every transition: **a mode change must re-read reality. Never carry
            cached state across one.**
            """
            nonlocal prev_q
            prev_q = np.asarray(robot.get_joint_pos(), dtype=float)[:N_ARM]
            if hasattr(robot, "resync"):
                robot.resync()

        def enter_teleop() -> None:
            nonlocal teleop, home_ee, gripper_value
            resync()
            q = np.asarray(robot.get_joint_pos(), dtype=float)
            robot.command_joint_pos(q)          # leaves zero-gravity mode
            # Take the jaws exactly where they are. Do NOT clamp here: clamping on
            # entry is a command to move, and nobody asked for that.
            gripper_value = float(q[N_ARM]) if len(q) > N_ARM else 0.5
            teleop = CartesianTeleop(frame=control_frame)
            teleop.reset(q[:N_ARM])
            home_ee = teleop.ee_position().copy()

        def enter_hold() -> None:
            resync()
            robot.command_joint_pos(np.asarray(robot.get_joint_pos(), dtype=float))

        def enter_guide() -> None:
            """Return to weightless after PD control.

            ⛔ The method is `enter_gravity_comp_idle()`. My first attempt guessed
            at `enable_gravity_comp` / `set_zero_gravity_mode` / `zero_gravity`,
            none of which exist — so GUIDE silently never worked after the first
            time, while the banner still announced "arm is weightless". Another
            message that lied. Guessing an API name and reporting success on the
            fallback path is exactly the failure mode this codebase specialises in.

            ⛔⭐ AND UNDERSTAND WHAT THIS MODE ACTUALLY RESTS ON. `zero_gravity_mode`
            sets **kp = 0** and commands zero torque, so the computed gravity
            compensation is the ONLY thing holding 4.3 kg up — there is no position
            term to absorb an error. Any shortfall in the model is an unopposed
            torque. That is how the arm fell on 2026-08-10 (FINDINGS §11): with
            `--no-gripper` the model was 0.695 kg light and the elbow was 39% short.
            GUIDE is therefore the mode where a dynamics-model error becomes a
            falling arm rather than a droop, which is why `guide_ref` is recorded
            here and drift is now printed live.
            """
            nonlocal guide_ref
            resync()
            guide_ref = np.asarray(robot.get_joint_pos(), dtype=float)
            fn = getattr(robot, "enter_gravity_comp_idle", None)
            if callable(fn):
                fn()
                return
            print("  ⚠️  enter_gravity_comp_idle() missing — staying in HOLD (NOT weightless)")

        def park_plan_line() -> str:
            """The one line showing what a run will do and how it will feel.

            ⭐ Printed while typing the sequence, on every knob change, and again at the
            confirm step — so speed and corner style are never something discovered
            only after the arm is already moving.
            """
            seq = " → ".join(park_sequence) if park_sequence else "0"
            name, radius = BLEND_MODES[blend_idx]
            # ⭐ ONE line, so changing a knob repaints instead of appending. Six taps
            # on `+` should leave one line showing the final speed, not six blocks.
            return (f"RUN {seq} · speed {park_speed:.2f} (-/+) · corners {name} "
                    f"{radius:.2f} (,/.) · ease {EASINGS[ease_idx].name} over "
                    f"{park_ramp:.2f} (e, ö/ä) · Enter=go")

        def replay_plan_line() -> str:
            """What a playback will do, with the two numbers that decide whether it can.

            ⭐ It shows the taught speed AND the ceiling on a planned move, because the
            interesting case is when the first exceeds the second. Julien met that case on
            his first try: a hand-guided recording moves faster than any planned motion here
            is allowed to, so playing it at 1.00x makes the loop wait for the arm and the
            playback comes out longer than the recording. Better said before he presses
            Enter than discovered afterwards.
            """
            if replay_pending is None:
                return ""
            taught = replay_pending.joint_speed(99)
            note = ""
            if taught > MAX_PLANNED_JOINT_SPEED:
                note = (f" ⚠️ taught {taught:.1f} rad/s exceeds the "
                        f"{MAX_PLANNED_JOINT_SPEED:.1f} allowed, so 1.00x will lag")
            return (f"PLAY {replay_slot} · {replay_pending.duration:.1f}s taught at "
                    f"{taught:.2f} rad/s · speed {replay_speed:.2f}x (-/+){note} · Enter=go")

        def begin_path(legs: list, what: str) -> None:
            """Start ONE continuous motion through every leg — the whole run, blended.

            ⭐ THE CORRECTION THIS IMPLEMENTS. The previous version ran each leg as a
            separate park and stopped dead at every waypoint. Julien: *"instead of
            moving and then jittering ninety degrees to the next side, in a smooth
            curve it would go to the next point … so that we have one smooth motion of
            specific waypoints."* One path, one cursor, corners rounded.

            ⚠️ Every waypoint still goes through `park_target_from`, so the gripper
            clamp and the 6-vs-7-joint reconciliation that once dropped an arm apply
            to each of them, not just the first.
            """
            nonlocal mode, park_target, park_cmd, park_path, park_s, park_marks
            nonlocal park_best_err, park_progress_t, park_leg_t, park_start_t
            targets = []
            for _, pose in legs:
                tgt, warn = park_target_from(robot.get_joint_pos(), pose,
                                             gripper_index=N_ARM, clamp=clamp_gripper)
                if warn:
                    print(f"\n  ⚠️  {warn}.")
                targets.append(tgt)
            start = np.asarray(robot.get_joint_pos(), dtype=float)
            park_path = JointPath([start, *targets], blend=BLEND_MODES[blend_idx][1])
            park_marks = list(zip([n for n, _ in legs], park_path.arrival_lengths()[1:]))
            park_s = 0.0
            park_target = targets[-1]
            park_cmd = start.copy()
            mode = "park"
            enter_hold()
            park_best_err = float(np.max(np.abs(park_target - start)))
            park_progress_t = t
            park_leg_t = t
            park_start_t = t
            # The plan has become the thing happening; the progress readout replaces it.
            hint("")
            print(f"\n⭐ MODE: PARK → {what}, {park_path.length:.2f} rad of travel at "
                  f"{park_speed:.2f} rad/s, corners {BLEND_MODES[blend_idx][0]}. "
                  "Press h or t to stop.\n")

        if mode == "teleop":
            enter_teleop()
        elif mode == "hold":
            enter_hold()
        elif mode == "guide":
            # ⚠️ GUIDE at startup is established by build_robot(zero_gravity=True), not
            # by enter_guide() — so the drift reference has to be taken here too, or the
            # readout silently shows nothing for the whole first GUIDE period. That gap
            # is exactly the 33 seconds in which the arm sank unremarked on 2026-08-10.
            guide_ref = np.asarray(robot.get_joint_pos(), dtype=float)

        dt = 1.0 / CONTROL_HZ
        # ⭐⭐ THE MEASURED LENGTH OF THE LAST CYCLE, next to the nominal one. Found on
        # 2026-08-13 because Julien's playback summary did not add up: a 3.6 s recording
        # reported "3.6 s of movement plus 0.4 s waiting" and finished in 4.6 s. The
        # missing 0.6 s is the loop running below 100 Hz. `dt` is a constant, and the
        # sleep at the bottom is `max(0, dt - elapsed)`, so a cycle that overruns is not
        # compensated: nominal time falls behind the wall clock, roughly 87 Hz against 100.
        #
        # ⛔ Anything that has to match real time must use `real_dt`, not `dt`. A playback
        # is exactly that: the whole point is reproducing the timing a hand taught.
        # ⚠️ Clamped, so one long stall (a slow disk, a thermal read retry) cannot make the
        # cursor jump forward and command a step the arm never asked for.
        real_dt = dt
        prev_t = 0.0
        loop_hz = CONTROL_HZ
        t0 = time.perf_counter()
        next_report = 1.0

        with KeyReader() as keys:
            if not keys.enabled:
                print("⚠️  stdin is not a terminal — keys will not work. Ctrl-C still does.\n")
            print(f"⭐ MODE: {mode.upper()}\n")

            # ⛔⭐ CTRL-C MUST NOT RELEASE THE ARM, and it used to.
            #
            # This whole session exists partly because "quitting released the arm on a
            # timer" and *a 5 s countdown is not consent* — so `q` was changed to go to
            # HOLD and wait for an explicit second key. **Ctrl-C went around all of
            # that.** SIGINT raised past the consent flow, past the `with`, into the
            # outer handler and straight to `finally`, which calls `shutdown_robot()`
            # and disables the motors. On a raised arm that is a sag, and Ctrl-C is
            # what everyone presses when something looks wrong.
            #
            # Found by reading on 2026-08-12, never yet triggered on hardware. It is
            # working contract rule 7 exactly: *what path reaches the hazard without
            # passing through your guard?* — this one, and the guard was the newest
            # code in the file.
            #
            # A handler rather than a try/except so the ~540-line loop body is not
            # re-indented for it. The first Ctrl-C sets a flag and restores the DEFAULT
            # handler, so the loop stops at the top of its next cycle and falls into
            # the same consent flow `q` uses — and a **second** Ctrl-C is a real force
            # quit, which is what someone pressing it twice means.
            interrupted: list[bool] = []

            def on_sigint(signum, frame) -> None:  # noqa: ANN001, ARG001
                signal.signal(signal.SIGINT, signal.SIG_DFL)
                interrupted.append(True)

            signal.signal(signal.SIGINT, on_sigint)

            while True:
                loop_start = time.perf_counter()
                t = loop_start - t0
                real_dt = min(0.1, max(1e-4, t - prev_t))
                prev_t = t
                # A slow exponential average, so the readout is a rate rather than noise.
                loop_hz += 0.02 * (1.0 / real_dt - loop_hz)

                if interrupted:
                    stop_reason = ("Ctrl-C — the loop is stopping, and the arm is NOT "
                                   "released until you choose below (Ctrl-C again forces it)")
                    break

                # ---- 1. is the robot still there? -------------------------
                if not chain_alive(robot):
                    stop_reason = (
                        "the motor chain STOPPED — I2RT's control thread exited, almost "
                        "certainly on a motor fault. Commands are no longer reaching the arm."
                    )
                    break

                # ---- 2. temperatures and the gripper stall guard -----------
                # ⛔⭐ ONLY THE READ IS WRAPPED. The decisions are not, and that is the
                # entire point of this shape. The previous version wrapped the read AND
                # every check that followed in one `try`, whose handler set
                # `hottest = 0.0` — so a failed read silently disarmed the thermal stop
                # and printed a calm "hottest 0°C". A guard with a path around it is
                # the defect this repo keeps paying for (working contract rule 7);
                # here the path was its own exception handler. See ThermalGuard.
                try:
                    states = chain.read_states()
                    read_error = None
                except Exception as exc:  # noqa: BLE001
                    states, read_error = None, f"{type(exc).__name__}: {exc}"

                if states is None:
                    hottest, jaw_temp = None, None
                    stall_since = None                # cannot judge a stall we cannot see
                    verdict = thermal.update(None)
                else:
                    temps, hottest, jaw_temp = motor_temperatures(states, N_ARM)
                    verdict = thermal.update(
                        hottest, jaw_temp,
                        motor=temps.index(hottest) if hottest is not None else None)
                    # ---- gripper stall guard ------------------------------
                    # ⚠️ With --no-gripper the chain has 6 motors, so states[6] would
                    # IndexError. It used to be guarded by raising StopIteration out of
                    # the shared try — which worked, but meant the "no gripper" path and
                    # the "read failed" path were the same code path. Now it is just an
                    # if, because there is nothing left to jump out of.
                    jaw = states[N_ARM] if len(states) > N_ARM else None
                    if jaw is None:
                        stall_since = None
                    elif (abs(getattr(jaw, "eff", 0.0)) > GRIPPER_STALL_TORQUE
                            and abs(getattr(jaw, "vel", 0.0)) < GRIPPER_STALL_VEL):
                        if stall_since is None:
                            stall_since = loop_start
                        elif loop_start - stall_since > GRIPPER_STALL_SECONDS:
                            measured_jaw = float(np.asarray(robot.get_joint_pos(), dtype=float)[N_ARM])
                            print(f"\n⚠️  GRIPPER STALLED ({jaw.eff:+.2f} Nm, not moving) — releasing it to "
                                  f"{measured_jaw:.3f} so it stops pushing.\n")
                            gripper_value = measured_jaw
                            stall_since = None
                    else:
                        stall_since = None

                if verdict.warning:
                    detail = f"  ({read_error})" if read_error else ""
                    print(f"\n⚠️  {verdict.warning}{detail}\n")
                if verdict.stop_reason:
                    stop_reason = verdict.stop_reason
                    break

                # ---- 3. keys ----------------------------------------------
                for k in keys.drain():
                    # ---- a pending s/p consumes the NEXT key as its argument ----
                    # ⭐ Two-key sequences, because a bare digit is already taken: 1-3
                    # flip rotation axes in the drive modes and 1-6 select motions in
                    # CONTROLS. A digit AFTER s or p is an argument, not a command, so
                    # nothing has to be re-bound. This is the shape Julien proposed.
                    #
                    # ⚠️ It is a mode, in a loop that drives 4.3 kg — so it is bounded:
                    # exactly one keypress wide for `s`, echoed at every keystroke for
                    # `p`, and cancelled by anything unexpected rather than guessing.
                    if pending == "save":
                        pending = None
                        if k.isdigit():
                            q = np.asarray(robot.get_joint_pos(), dtype=float)
                            name = BASE_SLOT if k == "0" else k
                            data = with_park_slot(load_json(PARK_FILE, {}), args.arm,
                                                  name, q.tolist())
                            save_json(PARK_FILE, data)
                            slots = park_slots(data, args.arm)
                            if k == "0":
                                park = q.tolist()
                                print(f"\n  ⭐ BASE pose (0) saved — this is where Ctrl-C "
                                      f"parks before disabling:\n     {np.round(q[:N_ARM], 3)}\n")
                            else:
                                print(f"\n  ✓ waypoint {k} saved: {np.round(q[:N_ARM], 3)}"
                                      f"     (p {k} drives back to it; Ctrl-C ignores it)\n")
                        else:
                            print("\n  save cancelled — s then 0-9 (0 = the base pose).\n")
                        continue

                    if pending == "take_save":
                        pending = None
                        take = None          # belt and braces: never resume by accident
                        if k.isdigit() and take_to_save is not None:
                            TAKES_DIR.mkdir(exist_ok=True)
                            path = TAKES_DIR / f"{k}.json"
                            # ⭐ The commit goes in at SAVE time, not at load time. Julien
                            # on provenance: he wants *"being able to reproduce everything
                            # and connect it to other research papers."* Which version of
                            # the code produced a recording is cheap to write now and
                            # impossible to reconstruct later. Same argument as the whole
                            # metadata block in ROADMAP §6.6.
                            take_to_save.meta["commit"] = git_commit()
                            take_to_save.meta["recorded_at"] = dt_now()
                            take_to_save.save(path)
                            print(f"\n  ✓ recording {k} saved: {take_to_save.duration:.1f}s, "
                                  f"{len(take_to_save)} samples → {path.name}")
                            print(f"     (l then {k} plays it back)\n")
                        else:
                            print("\n  recording discarded.\n")
                        take_to_save = None
                        continue

                    if pending == "take_play":
                        pending = None
                        if not k.isdigit():
                            print("\n  play cancelled.\n")
                            continue
                        path = TAKES_DIR / f"{k}.json"
                        if not path.is_file():
                            print(f"\n  ⚠️  nothing saved in recording {k} — "
                                  "press w to record one.\n")
                            continue
                        replay_pending = Trajectory.load(path)
                        replay_slot = k
                        # ⭐⭐ 1.00x IS THE TAUGHT SPEED, AND THE DEFAULT IS WHATEVER THE ARM
                        # CAN ACTUALLY FOLLOW. Reworked 2026-08-13, on Julien's suggestion:
                        # *"maybe one x should just be the original speed, and then you could
                        # go up and down. So max speed would just be limited by the actual
                        # safety things we have or the motors."*
                        #
                        # ⛔ WHY THE OLD VERSION COULD NOT EXPLAIN ITSELF. Hand-guiding a
                        # weightless arm reaches 2.4 to 2.9 rad/s (his own three recordings,
                        # 99th percentile), while MAX_PLANNED_JOINT_SPEED is 1.5. So every
                        # recording came back reading "max 1.00x", he played it at 1.00x, and
                        # it took 2.3 s longer than the recording because the loop kept
                        # holding the clock to let the arm catch up. Nothing on screen said
                        # why. Now the plan states both numbers and starts at a speed that
                        # will actually track.
                        trackable = safe_time_scale(replay_pending.joint_speed(99),
                                                    MAX_PLANNED_JOINT_SPEED)
                        replay_speed = min(1.0, trackable)
                        pending = "take_go"
                        hint(replay_plan_line())
                        continue

                    if pending == "take_go":
                        if k in ("+", "="):
                            # ⚠️ Upwards is capped at 1.00x whenever the recording is
                            # already faster than a planned move may be. Going above the
                            # taught speed there would ask for something the arm cannot do
                            # and this rig has no emergency stop.
                            ceiling = max(1.0, safe_time_scale(
                                replay_pending.joint_speed(99), MAX_PLANNED_JOINT_SPEED))
                            replay_speed = min(ceiling, replay_speed * 1.25)
                            hint(replay_plan_line()); continue
                        if k == "-":
                            replay_speed = max(0.05, replay_speed / 1.25)
                            hint(replay_plan_line()); continue
                        pending = None
                        if k in ("\r", "\n", " ") and replay_pending is not None:
                            # ⛔⭐ PARK TO THE START POSE FIRST, AND THIS IS THE SAFETY
                            # POINT OF THE WHOLE FEATURE. Playback commands poses the arm
                            # is known to reach, because a hand physically put it there.
                            # The dangerous command is the FIRST one: if the arm is
                            # somewhere else right now, commanding the recording's opening
                            # pose is a jump across whatever separates them. So the
                            # existing, tested, interruptible park drives there, and only
                            # when it arrives does playback begin.
                            begin_path([("recording start", list(replay_pending.start_pose()))],
                                       "the recording's start pose")
                            print("     then it plays the recording. Press h or t to stop.\n")
                        else:
                            replay_pending = None
                            hint("")
                            print("\n  play cancelled.\n")
                        continue

                    if pending in ("park", "confirm"):
                        # ⭐ SPEED AND CORNERS ADJUSTABLE WHILE TYPING, not only while
                        # moving. Julien: *"I can change the park speeds whilst it's
                        # parking, but not whilst I'm putting in the numbers, which is
                        # a bit annoying."* Deciding how a move should feel belongs to
                        # the moment you are choosing the move.
                        if k in "+=":
                            park_speed = min(1.5, park_speed * 1.25)
                            hint(park_plan_line()); continue
                        if k == "-":
                            park_speed = max(0.05, park_speed / 1.25)
                            hint(park_plan_line()); continue
                        if k == ".":
                            blend_idx = min(len(BLEND_MODES) - 1, blend_idx + 1)
                            hint(park_plan_line()); continue
                        if k == ",":
                            blend_idx = max(0, blend_idx - 1)
                            hint(park_plan_line()); continue
                        if k in KEY_STEP_UP:
                            # ⭐ How LONG the ease lasts, separately from its shape.
                            # Julien: *"the smoothing should maybe be adjustable at the
                            # beginning of the park, similar to the parking speed."*
                            # ö/ä (or [/]) mean gripper step elsewhere, which is
                            # meaningless while choosing a park — same
                            # context-dependence as +/-.
                            park_ramp = min(1.0, park_ramp * 1.4)
                            hint(park_plan_line()); continue
                        if k in KEY_STEP_DOWN:
                            park_ramp = max(0.0, park_ramp / 1.4 if park_ramp > 0.03 else 0.0)
                            hint(park_plan_line()); continue
                        if k == "e":
                            # ⭐ Start/stop easing is INDEPENDENT of corner blending.
                            # Julien wanted the ends toggleable without giving up the
                            # smooth corners; `none` leaves immediately, which is what
                            # a shutdown move wants.
                            #
                            # ⚠️ Handled HERE as well as in the main dispatch below, and
                            # that duplication is deliberate: an unrecognised key while
                            # typing a sequence CANCELS the run, so a key that is only
                            # bound further down would abort the very move it was meant
                            # to configure. `+/-` and `,/.` are duplicated for the same
                            # reason. The two differ only in what they show — the whole
                            # plan while choosing, just the profile otherwise.
                            ease_idx = (ease_idx + 1) % len(EASINGS)
                            hint(park_plan_line()); continue

                    if pending == "park":
                        if k.isdigit():
                            park_sequence.append(k)
                            # ⛔ A HINT, NOT THE STATUS ROW. This used to `print(…,
                            # end="")`, which the shadowed print routes to `screen.set`
                            # — the heartbeat row. So the echo of what you were typing
                            # replaced the temperature readout and was then wiped by
                            # the next once-a-second repaint: the one piece of feedback
                            # in a modal state that drives 4.3 kg, flickering.
                            hint(f"  park sequence: {' → '.join(park_sequence)}"
                                 f"   (another digit, or Enter)")
                            continue
                        if k in ("\r", "\n", " ", "p"):
                            # ⭐ ONE pose runs immediately, so `p Enter` for the base and
                            # `p 1 Enter` for a waypoint stay two keystrokes — the muscle
                            # memory Ctrl-C also depends on. TWO OR MORE shows the plan
                            # and waits for a second Enter, because a multi-pose run is a
                            # trajectory and how it moves is worth a glance first.
                            if len(park_sequence) >= 2:
                                pending = "confirm"
                                hint(park_plan_line())
                                continue
                            pending = None
                            wanted = park_sequence[:] or ["0"]
                            park_sequence.clear()
                            legs, missing = resolve_park_legs(wanted, park, slots)
                            if missing:
                                print(f"\n  ⚠️  nothing saved in slot {', '.join(missing)}"
                                      " — press s then that digit to record one.\n")
                            if legs:
                                begin_path(legs, f"slot {legs[0][0]}")
                            else:
                                print("\n  nothing to park to.\n")
                            continue
                        pending = None
                        park_sequence.clear()
                        hint("")
                        print("\n  park cancelled.\n")
                        continue

                    if pending == "confirm":
                        pending = None
                        if k in ("\r", "\n", " ", "p"):
                            wanted = park_sequence[:]
                            park_sequence.clear()
                            legs, missing = resolve_park_legs(wanted, park, slots)
                            if missing:
                                print(f"\n  ⚠️  skipping empty slot(s) {', '.join(missing)}.\n")
                            if legs:
                                begin_path(legs, " → ".join(n for n, _ in legs))
                            else:
                                print("\n  nothing to park to.\n")
                        else:
                            park_sequence.clear()
                            # ⛔⭐ THE STALE ROW JULIEN THEN PRESSED KEYS AT. `hint()`
                            # was never cleared anywhere in this file, so the
                            # `RUN 1 → 2 → 3 … ease s-curve over 0.20 (e, ö/ä) ·
                            # Enter=go` row stayed live for the rest of the session —
                            # after the run was cancelled, through mode changes, next
                            # to a `[HOLD]` status. It is a live row, repainted every
                            # cycle, which is why the same block appears twice in his
                            # paste with only the timestamp differing.
                            #
                            # Worse than untidy: it advertises `e` and `ö/ä`, he pressed
                            # `e`, and the reply was `(key 'e' does nothing)`. A hint
                            # that outlives the thing it describes is the `b` defect
                            # again (FINDINGS §17.1) — right text, wrong context.
                            hint("")
                            print("\n  run cancelled.\n")
                        continue

                    # ---- device configuration: works in EVERY mode ------------
                    # ⛔ `b` USED TO LIVE IN THE CONTROLS BRANCH ONLY, while the
                    # "press b to set the gripper buttons" hint printed in TELEOP as
                    # well. So in TELEOP the hint appeared and b fell through to the
                    # catch-all and did nothing. Julien hit exactly that: *"it says
                    # press b to set the gripper, and then b does nothing either."*
                    #
                    # A message that tells you to press a key which does nothing
                    # where you are is the same defect class as the refusal that
                    # named the wrong arm (FINDINGS §16) — the text is right, the
                    # context is wrong, and it costs the user a session to find out.
                    # Button assignment is a property of the DEVICE, not of the
                    # arm's mode, so it belongs above the mode dispatch entirely.
                    if k == "b":
                        learn_button = "open"
                        print("\n⭐ LEARNING THE GRIPPER BUTTONS.")
                        print("   Press the puck button you want for OPEN …")
                        print("   (learned by pressing, never assumed — which physical button")
                        print("    sets which HID bit has never been measured on this unit)\n")
                        continue
                    if k == "v":
                        # ⭐ Cycle which frame the puck's directions mean. Safe to do
                        # live: the twist is a VELOCITY, so a frame change alters the
                        # interpretation from the next cycle onward and leaves no
                        # stale cached state behind — unlike a mode change, which is
                        # why this does not need resync().
                        order = ["world", "tool", "camera"]
                        # ⛔ Save the map for the frame being LEFT before switching.
                        # Each frame owns its own wiring, so carrying one frame's map
                        # into another would silently overwrite it — the same
                        # blast-radius bug as editing a shared map believing it was
                        # per-arm.
                        map_store.set(args.arm, axis_map, control_frame)
                        control_frame = order[(order.index(control_frame) + 1) % len(order)]
                        axis_map = map_store.for_arm(args.arm, control_frame)
                        if teleop is not None:
                            teleop.frame = control_frame
                        print(f"\n  ⭐ CONTROL FRAME → {CartesianTeleop.FRAME_NOTES[control_frame]}")
                        print(f"     controls for this frame: {axis_map.one_line(control_frame)}")
                        print("     press m to edit THESE controls; each frame has its own\n")
                        continue
                    if k == "f" and last_input_kind == "button":
                        # Same key, same meaning everywhere: reverse the control just
                        # used. Axis -> flip its sign; button -> swap open/close.
                        axis_map.swap_buttons()
                        print("\n  ↔ SWAPPED the gripper buttons")
                        print(axis_map.buttons_row() + "\n")
                        continue

                    # ---- MAP mode owns the keyboard while it is active --------
                    # ⚠️ 1-6 mean "select a motion" here and "flip a rotation sign" in
                    # the drive modes. Overloading is a real footgun in a codebase
                    # whose motto is that this stack fails by lying, so it is bounded:
                    # MAP mode is entered explicitly, announces itself loudly, holds
                    # the arm still, and echoes the effect of every key. Nothing it
                    # can do moves a motor.
                    if mode == "map":
                        # ⛔ EVERY EDIT IN THIS BRANCH IS KEY-DRIVEN. Moving the puck
                        # must never change the map — see FINDINGS §11 for what
                        # happened when it did.
                        active = last_active_axis
                        driven = axis_map.motion_driven_by(active) if active is not None else None
                        if k == "q":
                            stop_reason = "quit requested"
                        elif k in "tghm":
                            print("\n  controls now:")
                            print(axis_map.describe(control_frame))
                            if k == "t":
                                mode = "teleop"; enter_teleop()
                                print("\n⭐ MODE: TELEOP — SpaceMouse drives, all axes\n")
                            elif k == "g":
                                mode = "guide"; enter_guide()
                                print("\n⭐ MODE: GUIDE — arm is weightless\n")
                            else:
                                mode = "hold"; enter_hold()
                                print("\n⭐ MODE: HOLD\n")
                        elif k == "f":
                            if active is None:
                                print("\n  push the puck first — f reverses the control you just used.\n")
                            elif driven is None:
                                print(f"\n  puck {PUCK_AXES[active]} drives nothing, so there is no "
                                      f"direction to reverse. Press 1-6 to give it a motion.\n")
                            else:
                                axis_map.flip(driven)
                                print(f"\n  ↔ REVERSED → {axis_map.row(driven, control_frame).strip()}"
                                      f"   (push {PUCK_AXES[active]} again to feel it)\n")
                        elif k in "123456":
                            if active is None:
                                print("\n  push the puck first — 1-6 reassigns the control you just used.\n")
                            else:
                                target = int(k) - 1
                                if driven is not None:
                                    # ⭐ SWAP, not steal. Julien's request after using this
                                    # on the arm: the commonest edit is two controls in
                                    # each other's places, and stealing left an orphan he
                                    # then had to notice and re-bind. A straight exchange
                                    # is also an involution, so pressing the same key
                                    # again undoes it. See AxisMap.swap().
                                    axis_map.swap(driven, target)
                                    print(f"\n  ⇄ SWAPPED {motions_for(control_frame)[driven]['short']} ↔ "
                                          f"{motions_for(control_frame)[target]['short']}")
                                    print(f"      {axis_map.row(target, control_frame).strip()}")
                                    print(f"      {axis_map.row(driven, control_frame).strip()}")
                                    print("      (press the same key again to swap back)\n")
                                else:
                                    # The active control drove nothing, so there is nothing
                                    # to exchange with. The direction he was last pushing
                                    # becomes this motion's positive sense.
                                    displaced = axis_map.bind(target, active, last_active_value)
                                    print(f"\n  ✓ puck {PUCK_AXES[active]} now drives "
                                          f"{motions_for(control_frame)[target]['short']} → "
                                          f"{axis_map.row(target, control_frame).strip()}")
                                    if displaced is not None:
                                        print(f"  ⚠️  {motions_for(control_frame)[displaced]['short']} was using that "
                                              f"control and is now UNBOUND — it will not move.")
                                    print()
                        elif k == "u":
                            if driven is None:
                                print("\n  that control already drives nothing.\n")
                            else:
                                axis_map.unbind(driven)
                                print(f"\n  unbound {motions_for(control_frame)[driven]['short']} — it will not move\n")
                        elif k == "0":
                            axis_map = axis_map_at_start.copy()
                            print("\n  reverted to the controls this session started with:")
                            print(axis_map.describe(control_frame) + "\n")
                        elif k == "?":
                            print(map_reference(control_frame))
                            print(MAP_HELP)
                            print(axis_map.describe(control_frame) + "\n")
                        # ⚠️ The rotation pair was MISSING here while the linear pair was
                        # present, so in CONTROLS mode roll/pitch/yaw could not be sped up
                        # or slowed down at all — Julien found it on the arm. The keys were
                        # copied from the drive-mode handler and the second pair was
                        # dropped. Both scales are also printed in the status line now, so
                        # a key that silently does nothing is visible rather than inferred.
                        elif k in "+=":
                            args.linear_scale *= 1.25
                            hint(f"linear speed {args.linear_scale:.3f} m/s")
                        elif k == "-":
                            args.linear_scale /= 1.25
                            hint(f"linear speed {args.linear_scale:.3f} m/s")
                        elif k == ".":
                            angular_scale *= 1.25
                            print(f"\n  rotation speed → {angular_scale:.2f} rad/s "
                                  f"({np.degrees(angular_scale):.0f}°/s)\n")
                        elif k == ",":
                            angular_scale /= 1.25
                            print(f"\n  rotation speed → {angular_scale:.2f} rad/s "
                                  f"({np.degrees(angular_scale):.0f}°/s)\n")
                        elif k == "r":
                            rotation = not rotation
                            print(f"\n  wrist rotation {'ON' if rotation else 'OFF'}"
                                  f"{'' if rotation else ' — ROLL/PITCH/YAW will not move'}\n")
                        elif k.isprintable() and k.strip():
                            print(f"\n  (key {k!r} does nothing in CONTROLS mode — press ? for the list)\n")
                        continue

                    # ⛔ Unrecognised keys are IGNORED. They used to fall through to a
                    # catch-all that cancelled PARK, so pressing Enter out of habit
                    # right after `p` killed the move in the same keyboard batch --
                    # which looked exactly like "park just went to hold". A control
                    # character must never be an action.
                    if k == "q":
                        stop_reason = "quit requested"
                    elif k == "m" and mode != "map":
                        # ⭐ CONTROLS mode DRIVES the arm — that is the whole point, and it
                        # is why this calls enter_teleop() rather than enter_hold(). The
                        # previous version held the arm still, which made it useless for
                        # the actual task: you cannot decide that a direction is wrong
                        # until you have watched the arm go that way. Julien:
                        # *"the actual mapping has to happen while the arm is moving so I
                        # can see what the different directions are doing."*
                        mode = "map"; enter_teleop()
                        last_active_axis = None
                        print("\n⭐ MODE: CONTROLS — the arm MOVES, one isolated axis, half speed.\n")
                        print(map_reference(control_frame))
                        print(MAP_HELP)
                        print(axis_map.explain(control_frame))
                        print("\n  Push the puck one way at a time and watch the arm. If a direction is")
                        print("  wrong, press f. If a control should do something else, press 1-6.\n")
                        if not rotation:
                            print("  ⚠️  wrist rotation is OFF (r toggles) — ROLL/PITCH/YAW will not move.\n")
                    elif k == "g" and mode != "guide":
                        mode = "guide"; hint(""); enter_guide()
                        print("\n⭐ MODE: GUIDE — arm is weightless\n")
                    elif k == "t" and mode != "teleop":
                        mode = "teleop"; hint(""); enter_teleop()
                        print("\n⭐ MODE: TELEOP — SpaceMouse drives\n")
                    elif k == "h" and mode != "hold":
                        mode = "hold"; hint(""); enter_hold()
                        print("\n⭐ MODE: HOLD\n")
                    elif k in MODE_KEYS:
                        # ⛔ ALREADY IN THAT MODE — say so, do not call it unrecognised.
                        # Julien's paste has `⭐ MODE: GUIDE` immediately followed by
                        # `(key 'g' does nothing — press ? for the list)`, which reads
                        # as the program having lost track of its own state. It had not:
                        # the mode branches above are guarded by `mode != …`, so a second
                        # press fell past them to the catch-all for unknown keys.
                        #
                        # ⚠️ The mode is deliberately NOT re-entered. `enter_guide()`
                        # re-arms gravity compensation and re-takes the drift reference,
                        # and this cannot be tested on the arm from here — so this fixes
                        # the message and changes nothing the motors see.
                        hint(f"already in {MODE_KEYS[k]}")
                    elif k == "w":
                        # ⭐ START OR STOP RECORDING. Deliberately allowed in EVERY mode,
                        # not only GUIDE. Hand-guiding is the intended use and the reason
                        # the feature exists, but a teleop run is also a demonstration and
                        # refusing to record one would be an arbitrary restriction. The
                        # mode in force is written into the metadata instead.
                        #
                        # ⚠️ Recording moves nothing, so a mis-press is harmless. That is
                        # why `w` needs no confirmation while `l` does.
                        if take is None:
                            take = Trajectory(meta={
                                "arm": args.arm,
                                "method": f"live:{mode}",
                                "nominal_hz": CONTROL_HZ,
                                "frame": control_frame,
                            })
                            take_t0 = t
                            take_modes = [mode]
                            print(f"\n⏺  RECORDING — {mode.upper()} mode. Press w again to stop.\n")
                        else:
                            # ⛔⭐ STOP MEANS STOP, AND THIS WAS A REAL BUG FOUND ON THE ARM
                            # ON 2026-08-13. `take` was left in place while the "which slot?"
                            # prompt waited for a digit, and the per-cycle sampler keys off
                            # `take is not None` — so the recording kept growing for as long
                            # as Julien took to answer. Measured from his own three files:
                            # 1.8 s, 4.4 s and 3.3 s of extra samples appended AFTER the stop
                            # keypress, with 0.1 to 0.7 rad of movement in the tail. He
                            # described it exactly: *"the recordings played for like two
                            # seconds longer than I actually recorded, it was just standing
                            # still for that time."*
                            #
                            # Moving it to a second name is the whole fix: the sampler stops
                            # on this line, and the prompt then saves something frozen.
                            take_to_save, take = take, None
                            # ⭐ Stamp what the recording actually WAS, now that it is over.
                            # `method` was written at the keypress and can only name the mode
                            # it started in; a hand-guided movement begun from HOLD came out
                            # labelled `live:hold`. Both fields are kept: `method` stays for
                            # anything already reading it, and `modes` is the truth.
                            take_to_save.meta["modes"] = list(take_modes)
                            if len(take_modes) > 1:
                                take_to_save.meta["method"] = \
                                    "live:" + "+".join(take_modes)
                            n, secs = len(take_to_save), take_to_save.duration
                            if n < 2:
                                take_to_save = None
                                print("\n  nothing recorded (too short) — discarded.\n")
                            else:
                                pending = "take_save"
                                print(f"\n⏹  RECORDED {secs:.1f}s, {n} samples, "
                                      f"typical joint speed "
                                      f"{take_to_save.joint_speed(99):.2f} rad/s "
                                      f"(peak {take_to_save.max_joint_speed():.2f}).")
                                print("     SAVE to which slot? 0-9, any other key discards.\n")
                    elif k == "l":
                        # ⛔ PLAY A RECORDING, AND IT ASKS TWICE ON PURPOSE. `l` sits next
                        # to `ö` and `ä` on a German keyboard, which now adjust the ease
                        # ramp — so a slip lands on a key that would otherwise start 4.3 kg
                        # moving. Showing the plan and waiting for Enter means a stray `l`
                        # can never move the arm. Same shape as `p 1 2 3 Enter`.
                        pending = "take_play"
                        have = ", ".join(sorted(p.stem for p in TAKES_DIR.glob("*.json"))) \
                            if TAKES_DIR.is_dir() else ""
                        print("\n  PLAY which recording?  0-9, any other key cancels.")
                        print(f"     saved: {have or 'none'}\n")
                    elif k == "s":
                        pending = "save"
                        saved_now = ", ".join(sorted(n for n in slots if n != BASE_SLOT))
                        print(f"\n  SAVE this pose to which slot?  0 = the BASE pose "
                              f"(where Ctrl-C parks), 1-9 = a waypoint.")
                        print(f"     waypoints already saved: {saved_now or 'none'}"
                              f"        any other key cancels\n")
                    elif k == "p":
                        pending = "park"
                        park_sequence.clear()
                        have = ", ".join(sorted(n for n in slots if n != BASE_SLOT))
                        print(f"\n  PARK to which?  0 = base, 1-9 = a waypoint, "
                              f"Enter = base.")
                        print(f"     Type several digits for a SEQUENCE, then Enter."
                              f"   waypoints: {have or 'none'}\n")
                    elif k == "o" and mode == "teleop":
                        gripper_value = clamp_gripper(gripper_value + gripper_step)
                    elif k == "c" and mode == "teleop":
                        gripper_value = clamp_gripper(gripper_value - gripper_step)
                    elif k in KEY_STEP_UP:
                        # ⭐⭐ ö AND ä MEAN THE EASE RAMP EVERYWHERE NOW. Changed
                        # 2026-08-13, after Julien hit the confusion on the arm.
                        #
                        # ⛔ WHAT WENT WRONG. `e` cycles the ease profile in any mode, and
                        # its own message read *"(ö/ä adjusts how long)"*. But outside a
                        # park prompt those keys were bound to the GRIPPER STEP, so the
                        # message told him to press keys that did something else. He
                        # pressed them and pushed the gripper step to its 0.200 ceiling by
                        # accident, which makes every later `o` or `c` move the jaws a
                        # fifth of their travel. His words: *"the German characters ö and ä
                        # don't quite work as I think they should… they change the gripper
                        # step speed, which in itself might be cool, but not necessary
                        # currently and all the time."*
                        #
                        # ⭐ The fix is one meaning per key, not a cleverer message. A
                        # message that has to explain which of two things a key does today
                        # is a design admitting it is wrong. The gripper step moves to
                        # `--gripper-step`: it is a preference set once, and he said
                        # outright that it does not need a live key.
                        park_ramp = min(1.0, park_ramp * 1.4)
                        hint(ease_note(EASINGS[ease_idx].name, park_ramp))
                    elif k in KEY_STEP_DOWN:
                        park_ramp = max(0.0, park_ramp / 1.4 if park_ramp > 0.03 else 0.0)
                        hint(ease_note(EASINGS[ease_idx].name, park_ramp))
                    elif k == "e":
                        # ⭐ WORKS EVERYWHERE, and the message now says WHAT IT AFFECTS.
                        # Julien, on the arm: *"the easing outside of parking, I don't
                        # really know what that means. Does it work for recording, or does
                        # it work for teleoperating? … what's the point of that?"* A fair
                        # question, and the answer is neither. Easing shapes only PLANNED
                        # moves: `p` runs and the Ctrl-C park. It does nothing for driving
                        # by hand or for a playback, which follows recorded timing instead.
                        # Pressing `e` elsewhere sets up the next planned move. A knob whose
                        # effect you cannot see has to say where its effect lives, every
                        # time it is touched.
                        ease_idx = (ease_idx + 1) % len(EASINGS)
                        hint(ease_note(EASINGS[ease_idx].name, park_ramp))
                    elif k == "r":
                        rotation = not rotation
                        hint(f"wrist rotation {'ON' if rotation else 'OFF'}")
                    elif k == ".":
                        angular_scale *= 1.25
                        hint(f"rotation speed {angular_scale:.2f} rad/s")
                    elif k == ",":
                        angular_scale /= 1.25
                        hint(f"rotation speed {angular_scale:.2f} rad/s")
                    elif k in "xyz":
                        # ⚠️ These now flip a ROBOT MOTION, not a puck axis. Under the
                        # identity map that is the same arithmetic, which is why the
                        # hand-dialled file still means what it meant. Under a
                        # permutation it is the only reading that stays useful: when
                        # Julien presses x he means "the gripper goes the wrong way",
                        # which is a statement about the arm, not about the device.
                        idx = "xyz".index(k)
                        axis_map.flip(idx)
                        print(f"\n  {motions_for(control_frame)[idx]['short']} flipped → "
                              f"{axis_map.row(idx, control_frame).strip()}\n")
                    elif k in "123":
                        # Rotation motions: 1 roll, 2 pitch, 3 yaw. Digits because every
                        # sensible letter was taken, and because they read as an
                        # ordered triple the way x/y/z do.
                        idx = 3 + "123".index(k)
                        axis_map.flip(idx)
                        print(f"\n  {motions_for(control_frame)[idx]['short']} flipped → "
                              f"{axis_map.row(idx, control_frame).strip()}\n")
                    elif k == "+" or k == "=":
                        # ⭐ In PARK these mean the park speed. The teleop linear scale
                        # is meaningless while the puck is not driving, and a key that
                        # does nothing where you are is the defect class that made `b`
                        # look broken (FINDINGS §17.1).
                        if mode == "park":
                            park_speed = min(1.5, park_speed * 1.25)
                            hint(f"park speed {park_speed:.2f} rad/s")
                        else:
                            args.linear_scale *= 1.25
                            hint(f"linear speed {args.linear_scale:.3f} m/s")
                    elif k == "-":
                        if mode == "park":
                            park_speed = max(0.05, park_speed / 1.25)
                            hint(f"park speed {park_speed:.2f} rad/s")
                        else:
                            args.linear_scale /= 1.25
                            hint(f"linear speed {args.linear_scale:.3f} m/s")
                    elif k == "?":
                        print(HELP)
                    elif k.isprintable() and k.strip():
                        print(f"\n  (key {k!r} does nothing — press ? for the list)\n")
                # ⛔ Leaving PARK for ANY reason abandons the rest of the run. One
                # place rather than a clear() in each of g/t/h/m/blocked, because the
                # one that gets forgotten is the one that matters: an arm resuming a
                # planned trajectory after the operator pressed HOLD is doing something
                # nobody asked for.
                if mode != "park" and park_path is not None:
                    left = park_path.length - park_s
                    if left > PARK_TOLERANCE:
                        print(f"\n  ⚠️  run abandoned with {left:.2f} rad of path left — "
                              "leaving PARK cancels the rest.\n")
                    park_path, park_marks = None, []
                    # ⛔ A park that was interrupted must not hand over to a playback. The
                    # handover lives in the arrival branch, but this is the second gate:
                    # pressing h or t while driving to the start pose cancels the whole
                    # thing, rather than leaving a recording queued to fire later.
                    if replay_pending is not None:
                        replay_pending = None
                        hint("")
                        print("  ⚠️  playback cancelled — it never reached the start pose.\n")
                # ⛔ Same rule for a playback in progress: leaving the mode abandons it.
                # An arm resuming a recorded movement after the operator pressed HOLD is
                # doing something nobody asked for.
                if mode != "replay" and replay is not None:
                    left = replay.duration - replay_s
                    if left > 0.05:
                        print(f"\n  ⚠️  playback abandoned with {left:.1f}s left.\n")
                    replay = None
                    hint("")
                if stop_reason:
                    break

                # ---- 3.4 sample the recording, if one is running ---------------
                # ⭐ EVERY CYCLE, IN EVERY MODE, and before the mode acts. Recording is a
                # property of the session rather than of a mode, so putting it in a branch
                # would silently stop capturing the moment the operator switched modes —
                # which is the same defect shape as the puck being read only inside the
                # teleop branch, fixed just below.
                #
                # ⚠️ It records the MEASURED position. For a hand-guided demonstration that
                # is the only thing that means anything: in GUIDE the position gain is zero,
                # so there is no command to record, and the arm is wherever the hand put it.
                if take is not None:
                    # ⛔⭐ A RECORDING PROBLEM MUST NEVER TAKE DOWN THE SESSION, and this
                    # wrapper is the whole reason the block exists. `append` raises on a
                    # non-monotonic timestamp or a changed joint count. Unwrapped, that
                    # exception would leave the control loop, skip the "the arm is HOLDING,
                    # press g or d" consent flow, and fall into `finally`, which disables
                    # the motors. On a raised arm that is a sag. It is the exact path that
                    # dropped 4.3 kg once already (FINDINGS §11, park_target_from), and
                    # recording is a convenience feature: it has no business being able to
                    # release the arm.
                    try:
                        take.append(t - take_t0, robot.get_joint_pos())
                        # ⛔⭐ RECORD EVERY MODE THE RECORDING PASSED THROUGH, not only the
                        # one it started in. Julien's recording of 2026-08-13 17:21 was
                        # stamped `method: live:hold` because he pressed `w` while in HOLD
                        # and then switched to GUIDE to hand-guide it. **The stamp described
                        # the keypress rather than the demonstration**, and provenance is the
                        # thing ROADMAP §6.6 says matters most about a recording. A dataset
                        # that mislabels how a demonstration was produced is worse than one
                        # that omits it. FINDINGS §35.4.
                        if mode not in take_modes:
                            take_modes.append(mode)
                    except Exception as exc:  # noqa: BLE001
                        print(f"\n⚠️  recording stopped: {type(exc).__name__}: {exc}")
                        print("     The arm is unaffected. Press w to start a new one.\n")
                        take = None
                    else:
                        # ⚠️ A bound, because this grows in memory for as long as it runs and
                        # nothing else would ever stop it. 100 000 samples is ~16 minutes at
                        # 100 Hz, comfortably past the ~4.5 minutes a long-context policy
                        # wants (ROADMAP §9.3). Stopping and saying so beats running out of
                        # memory in a process that is driving an arm.
                        if len(take) >= MAX_TAKE_SAMPLES:
                            # Freeze it the same way `w` does, or the limit would not be one.
                            take_to_save, take = take, None
                            pending = "take_save"
                            print(f"\n⏹  RECORDING STOPPED at the {MAX_TAKE_SAMPLES} sample "
                                  f"limit ({take_to_save.duration:.0f}s).")
                            print("     SAVE to which slot? 0-9, any other key discards.\n")

                # ---- 4. act on the mode -----------------------------------
                # ---- 3.5 the puck, read EVERY cycle in EVERY mode -------------
                # ⛔ This used to sit inside the teleop/map branch, which had two
                # consequences: the buttons were dead in GUIDE and HOLD, and the HID
                # reports queued up while in those modes and then arrived in a burst
                # on the next mode switch. Reading unconditionally costs nothing —
                # TwistReader.read() is non-blocking by construction — and it is what
                # makes button assignment work from wherever Julien happens to be.
                raw_axes = reader.read()
                buttons = getattr(reader, "buttons", 0)
                pressed = buttons & ~buttons_prev              # rising edge only
                buttons_prev = buttons

                if learn_button is not None and pressed:
                    warn = axis_map.learn_button(learn_button, pressed)
                    if warn:
                        print(f"\n  ⚠️  {warn}\n")
                    elif learn_button == "open":
                        learn_button = "close"
                        print(f"  ✓ OPEN  ← button 0x{pressed:02x}")
                        print("   Now press the button you want for CLOSE …\n")
                    else:
                        learn_button = None
                        print(f"  ✓ CLOSE ← button 0x{pressed:02x}")
                        print(axis_map.buttons_row())
                        print("   (f swaps them if they are the wrong way round)\n")
                elif pressed:
                    # A press counts as "the control you just used", so f reverses it.
                    # ⛔ But ONLY keys edit the map, exactly as for the axes: pressing
                    # a button never rebinds anything.
                    last_input_kind = "button"
                    if axis_map.button_action(pressed) is None:
                        print(f"\n  button 0x{pressed:02x} is not assigned — press b to set the "
                              f"gripper buttons (works in any mode)\n")
                    elif mode not in ("teleop", "map"):
                        print(f"\n  gripper buttons move the jaws in TELEOP (t) and CONTROLS (m); "
                              f"you are in {mode.upper()}\n")

                if learn_button is None and robot.num_dofs() > N_ARM and mode in ("teleop", "map"):
                    action = axis_map.button_action(buttons)
                    if action == "open":
                        gripper_value = clamp_gripper(gripper_value + GRIPPER_BUTTON_RATE * dt)
                    elif action == "close":
                        gripper_value = clamp_gripper(gripper_value - GRIPPER_BUTTON_RATE * dt)

                if mode in ("teleop", "map") and teleop is not None:

                    if mode == "map":
                        # ⭐ AXIS ISOLATION — Julien's design: only the strongest puck
                        # direction is applied, so the arm performs exactly one motion and
                        # it is obvious which gesture caused it. Half speed, because this
                        # is the mode you experiment in.
                        #
                        # ⛔ Note what is NOT here: any call that edits the map. Deflection
                        # observes; keys edit. The mode this replaced bound on deflection
                        # and destroyed the hand-dialled map (FINDINGS §11).
                        keep, value = isolate(raw_axes, last_active_axis)
                        if keep is not None:
                            last_active_axis, last_active_value = keep, value
                            last_input_kind = "axis"
                        drive_axes = isolated_axes(raw_axes, keep)
                        scale_l = args.linear_scale * CONTROLS_SCALE
                        scale_a = angular_scale * CONTROLS_SCALE
                    else:
                        drive_axes = raw_axes
                        scale_l, scale_a = args.linear_scale, angular_scale

                    axes = axis_map.apply(drive_axes)
                    twist = np.array([
                        axes[0] * scale_l, axes[1] * scale_l, axes[2] * scale_l,
                        axes[3] * scale_a if rotation else 0.0,
                        axes[4] * scale_a if rotation else 0.0,
                        axes[5] * scale_a if rotation else 0.0,
                    ])
                    q_target = teleop.step(twist, dt)

                    ee = teleop.ee_position()
                    if np.any(np.abs(ee - home_ee) > args.box):
                        import mink  # noqa: PLC0415
                        teleop.target = mink.SE3.from_rotation_and_translation(
                            rotation=teleop.target.rotation(),
                            translation=np.clip(ee, home_ee - args.box, home_ee + args.box),
                        )

                    step = q_target - prev_q
                    q_target = prev_q + np.clip(step, -MAX_JOINT_STEP, MAX_JOINT_STEP)

                    lo = np.array([YAM_JOINTS[i][1] for i in range(1, N_ARM + 1)]) + JOINT_LIMIT_MARGIN
                    hi = np.array([YAM_JOINTS[i][2] for i in range(1, N_ARM + 1)]) - JOINT_LIMIT_MARGIN
                    q_target = np.clip(q_target, lo, hi)

                    full = np.zeros(robot.num_dofs())
                    full[:N_ARM] = q_target
                    if robot.num_dofs() > N_ARM:
                        full[N_ARM] = clamp_gripper(gripper_value)
                    robot.command_joint_pos(full)
                    prev_q = q_target.copy()

                elif mode == "replay" and replay is not None:
                    # ⭐⭐ FOLLOW THE RECORDING IN TIME, not along its length. This is the
                    # whole reason the feature exists rather than reusing the waypoint
                    # runner: a park traverses a *shape* at a constant joint speed, which
                    # throws away exactly the thing hand-guiding provides. Human timing and
                    # hesitation are the signal (docs/ROADMAP.md §6.6), so the cursor here
                    # is a clock.
                    q = np.asarray(robot.get_joint_pos(), dtype=float)
                    # ⭐ The decision is made by `replay_step` in src/recording.py, which
                    # has its own tests. This branch only carries it out and narrates it —
                    # same split as ArmSession, and the reason is that this code path
                    # commands the arm and could not otherwise be exercised without one.
                    #
                    # ⛔ `n_compare=N_ARM` leaves the gripper out of the "is the arm
                    # keeping up" check. The jaws legitimately sit far from their commanded
                    # value while closing on an object, and counting that as falling behind
                    # would stall every playback that grips anything.
                    rs = replay_step(replay, replay_s, q, real_dt, speed=replay_speed,
                                     max_lag=MAX_CURSOR_LAG, n_compare=N_ARM)
                    replay_s = rs.cursor
                    full = q.copy()
                    full[:N_ARM] = rs.target[:N_ARM]
                    if robot.num_dofs() > N_ARM and len(rs.target) > N_ARM:
                        # ⛔ Through the clamp, never straight from the file. A recording
                        # made while the jaws rested on a stop would otherwise drive them
                        # back onto it and HOLD there. That is stall torque, and it is how
                        # motor 7 was cooked three times (FINDINGS §4).
                        full[N_ARM] = clamp_gripper(float(rs.target[N_ARM]))
                    robot.command_joint_pos(full)

                    replay_worst_lag = max(replay_worst_lag, rs.lag)
                    # ⭐ THE PER-JOINT ANSWER TO "HOW FAST CAN THE ARMS MOVE", collected from
                    # motion Julien is already running rather than from a speed sweep that
                    # would command the arm faster than any existing code allows. Reasoning
                    # in src/recording.py::TrackingLog and ROADMAP §7.5.
                    if tracking is not None and replay_prev_target is not None:
                        tracking.observe(rs.target, replay_prev_target, q, real_dt)
                    replay_prev_target = list(rs.target)
                    if rs.held:
                        replay_held_s += real_dt
                    else:
                        replay_progress_t = t
                    if rs.finished:
                        mode = "hold"; enter_hold(); hint("")
                        # ⭐⭐ SAY WHERE THE EXTRA TIME WENT. Julien's first playbacks ran
                        # 2.3 s longer than the recording and the old message reported only
                        # the total, so it read as a bug with no explanation. The whole
                        # difference is the loop holding the clock while the arm catches up,
                        # which is a decision this code makes on purpose. A readout has to
                        # show what can go wrong, not only what looks tidy — the same lesson
                        # as showing the jaw temperature separately (FINDINGS §11).
                        planned = replay.duration / replay_speed
                        elapsed = t - replay_t0
                        print(f"⭐ PLAYBACK finished in {elapsed:.1f}s → HOLD")
                        print(f"     {planned:.1f}s of movement at {replay_speed:.2f}x, "
                              f"plus {replay_held_s:.1f}s waiting for the arm to catch up.")
                        # ⛔ THE TWO NUMBERS MUST RECONCILE, and on 2026-08-13 they did not:
                        # a 3.6 s recording reported 3.6 + 0.4 and finished in 4.6. The gap
                        # was the loop running below 100 Hz while the cursor advanced in
                        # nominal time. That is fixed, and this check stays so a future
                        # version cannot reintroduce it silently.
                        unaccounted = elapsed - planned - replay_held_s
                        if abs(unaccounted) > 0.15 + 0.05 * elapsed:
                            print(f"     ⚠️  {unaccounted:+.1f}s is unaccounted for. The loop "
                                  f"averaged {loop_hz:.0f} Hz against {CONTROL_HZ:.0f}.")
                        print(f"     worst it fell behind: {replay_worst_lag:.3f} rad "
                              f"(the loop holds the clock past {MAX_CURSOR_LAG:.2f}).")
                        if replay_held_s > 0.15 * planned:
                            print(f"     ⚠️  it spent {100 * replay_held_s / (planned + replay_held_s):.0f}% "
                                  f"of the run waiting. Try a lower speed for a faithful replay.\n")
                        else:
                            print()
                        if tracking is not None and tracking.cycles > 20:
                            # ⚠️ MEASURED, so read it as such. The playback holds its clock
                            # once the arm falls behind, so the speeds here are not an even
                            # sweep, and load changes with the arm's pose. It is the cheap
                            # first answer; ROADMAP §7.5 has the active sweep if this is
                            # ambiguous.
                            print("     how well each joint kept up "
                                  f"(the loop holds past {MAX_CURSOR_LAG:.2f} rad):")
                            for i, worst, at_speed, top, lag_top in tracking.rows():
                                if top < 0.01:
                                    continue
                                name = YAM_JOINTS.get(i + 1, ("joint",))[0]
                                print(f"       {name:<14} worst lag {worst:.3f} rad at "
                                      f"{at_speed:5.2f} rad/s · top speed {top:5.2f} rad/s "
                                      f"with {lag_top:.3f} rad of lag")
                            # ⭐⭐ AND KEEP IT. This table is the only measurement anyone has
                            # of what the arm can physically follow, and on 2026-08-13 the
                            # only copy of it was a paste into a chat window. Saved per
                            # playback under a timestamp, so nothing overwrites anything.
                            # ⚠️ Never lets a failed write end a session: the arm is in HOLD
                            # at this point and a missing diagnostic file is not worth a
                            # traceback. FINDINGS §34.4.
                            try:
                                names = [YAM_JOINTS.get(i + 1, ("joint",))[0]
                                         for i in range(tracking.n_joints)]
                                rec = tracking.to_dict(names)
                                rec["meta"] = {
                                    "arm": args.arm,
                                    "slot": replay_slot,
                                    "played_at": dt_now(),
                                    "commit": git_commit(),
                                    "speed": round(replay_speed, 3),
                                    "taught_speed_p99": round(replay.joint_speed(99), 4),
                                    "recording_duration_s": round(replay.duration, 3),
                                    "recording_meta": dict(replay.meta),
                                    "elapsed_s": round(elapsed, 3),
                                    "held_s": round(replay_held_s, 3),
                                    "worst_lag_rad": round(replay_worst_lag, 5),
                                    "loop_hz": round(loop_hz, 1),
                                    "max_cursor_lag": MAX_CURSOR_LAG,
                                    "max_planned_joint_speed": MAX_PLANNED_JOINT_SPEED,
                                }
                                TRACKING_DIR.mkdir(parents=True, exist_ok=True)
                                stamp = dt_now().replace(":", "-")
                                out = TRACKING_DIR / f"{replay_slot}_{stamp}.json"
                                out.write_text(json.dumps(rec, indent=1) + "\n")
                                print(f"     ⭐ saved this table → "
                                      f"{out.relative_to(REPO)}")
                            except Exception as exc:  # noqa: BLE001
                                print(f"     ⚠️  could not save the tracking table: "
                                      f"{type(exc).__name__}: {exc}")
                        replay = None
                    elif t - replay_progress_t > PARK_STALL_SECONDS:
                        # ⛔ NEVER WAIT FOR EVER. Holding the clock is right for a moment
                        # and wrong for ever: an arm that cannot catch up is blocked, and a
                        # playback that sits silently holding its clock is the treadmill
                        # bug again (FINDINGS §24). Same patience the park uses.
                        mode = "hold"; enter_hold(); hint("")
                        print(f"\n⛔ PLAYBACK BLOCKED — the arm stopped following "
                              f"{rs.lag:.3f} rad behind the recording, no progress for "
                              f"{PARK_STALL_SECONDS:.0f}s. Now HOLDING.\n")
                        replay = None
                    elif t >= next_park_report:
                        next_park_report = t + 1.0
                        hint(f"  playing… {replay.duration - replay_s:.1f}s left, "
                             f"{rs.lag:.3f} rad behind")

                elif mode == "park" and park_path is not None and park_target is not None:
                    q = np.asarray(robot.get_joint_pos(), dtype=float)
                    # ⭐ Completion is judged from the MEASURED pose, never from the
                    # command — the command always arrives first, so testing it would
                    # declare success while the arm was still travelling.
                    err = float(np.max(np.abs(park_target - q)))
                    lag = float(np.max(np.abs(park_cmd - q))) if park_cmd is not None else 0.0
                    at_end = park_s >= park_path.length

                    # ⛔ ARRIVAL IS GATED ON THE CURSOR REACHING THE END, not on the
                    # error alone. A run like `p 1 2 1` finishes where it started, so
                    # the error to the FINAL target is small at t=0 too — judging on it
                    # would declare the whole sequence complete before moving.
                    if not at_end:
                        # Hold the cursor if the arm has fallen behind. The trajectory
                        # is a SHAPE now; a command racing ahead while the arm cuts its
                        # own corner is not the shape anyone chose.
                        advanced = False
                        if lag < MAX_CURSOR_LAG:
                            ramp = easing_factor(
                                EASINGS[ease_idx], park_s, park_path.length - park_s,
                                0.0 if args.no_smooth else park_ramp)
                            park_s = min(park_path.length,
                                         park_s + park_speed * ramp * dt)
                            advanced = True
                        park_cmd = park_path.point_at(park_s)
                        robot.command_joint_pos(park_cmd)

                        # Progress is "the cursor moved OR the arm closed the gap".
                        # Without the first half, a legitimately slow leg looks stalled;
                        # without the second, an arm pinned against something never does.
                        if advanced or err < park_best_err - PARK_PROGRESS_EPS:
                            park_best_err = min(park_best_err, err)
                            park_progress_t = t
                        if t - park_progress_t > PARK_STALL_SECONDS:
                            mode = "hold"; enter_hold(); hint("")
                            print(f"\n⛔ PARK BLOCKED — the arm stopped following "
                                  f"{lag:.3f} rad behind the path, no progress for "
                                  f"{PARK_STALL_SECONDS:.0f}s. Now HOLDING.\n")
                        elif park_marks and park_s >= park_marks[0][1]:
                            # ⭐ Time each waypoint. Julien: *"you can't really see how
                            # long each parking section took, you can only see the park
                            # itself."* Now each leg reports its own seconds as it is
                            # passed, which is also the number to watch when tuning
                            # speed and corner radius.
                            name, _ = park_marks.pop(0)
                            print(f"  ⭐ slot {name} in {t - park_leg_t:.1f}s"
                                  + (f" → next {park_marks[0][0]}" if park_marks else ""))
                            park_leg_t = t
                        elif t >= next_park_report:
                            next_park_report = t + 1.0
                            # ⛔ A HINT, NOT THE STATUS ROW — same fix as the sequence
                            # echo. Routed through `screen.set` this replaced the
                            # temperature heartbeat with the progress readout, so during
                            # the one motion where an operator most wants to see a
                            # temperature climbing, it was the line that had been
                            # painted over.
                            hint(f"  moving… {park_path.length - park_s:.2f} rad of path "
                                 f"left, {err:.3f} to the final pose, {lag:.3f} behind")
                    else:
                        leg = park_verdict(err, t - park_progress_t > PARK_STALL_SECONDS,
                                           PARK_TOLERANCE, PARK_SETTLED,
                                           stopped_briefly=t - park_progress_t
                                           > PARK_SETTLE_SECONDS)
                        if leg in ("arrived", "settled"):
                            extra = ("" if leg == "arrived" else
                                     " — as close as the arm holds itself under load")
                            mode = "hold"; enter_hold()
                            hint("")        # the progress readout has nothing left to say
                            # ⛔ `park_start_t`, NOT `park_leg_t`. The last leg's mark is
                            # passed at the end of the path, which resets the leg clock
                            # moments before arrival — so this line used to report a 4.4 s
                            # park as "reached in 0.0s". FINDINGS §34.3.
                            # ⭐ Both numbers are shown because they answer different
                            # questions: the total is what changes when speed, corner
                            # radius or the ease ramp is tuned, and the settling time is
                            # how long the arm took to close the last of the gap after the
                            # commanded path had already run out.
                            total = t - park_start_t
                            settling = t - park_leg_t
                            tail = (f", {settling:.1f}s of that settling"
                                    if 0.05 < settling < total - 0.05 else "")
                            print(f"⭐ PARK reached in {total:.1f}s{tail} "
                                  f"({err:.3f} rad off{extra}) → HOLD")
                            # ⭐ The handover from "drive to the start pose" to "play the
                            # recording". It lives HERE, in the arrival branch, so a park
                            # that was blocked or interrupted can never roll into a
                            # playback: only a park that actually arrived does.
                            if replay_pending is not None:
                                replay = replay_pending
                                replay_pending = None
                                replay_t0, replay_s = t, 0.0
                                replay_progress_t = t
                                replay_held_s, replay_worst_lag = 0.0, 0.0
                                replay_prev_target = list(replay.start_pose() or ())
                                tracking = TrackingLog(replay.n_joints)
                                mode = "replay"
                                print(f"\n▶  PLAYING {replay.duration:.1f}s of recorded "
                                      f"movement at {replay_speed:.2f}x. "
                                      "Press h or t to stop.\n")
                        elif leg == "blocked":
                            # ⛔ Never spin silently. If the arm has stopped closing the
                            # gap the honest thing is to say so and hold, not to keep
                            # printing a number that is not changing — which is exactly
                            # how the old treadmill bug hid for two sessions.
                            mode = "hold"; enter_hold(); hint("")
                            print(f"\n⛔ PARK BLOCKED — {err:.3f} rad still to go and no "
                                  f"progress for {PARK_STALL_SECONDS:.0f}s.")
                            print(f"   The command ran {lag:.3f} rad ahead of the arm; "
                                  f"SafeRobot limited "
                                  f"{getattr(robot, 'limited_cycles', 0)} cycles.")
                            print("   Something is blocking it, or the pose is "
                                  "unreachable. Now HOLDING.\n")
                        else:
                            robot.command_joint_pos(park_cmd)
                            if err < park_best_err - PARK_PROGRESS_EPS:
                                park_best_err, park_progress_t = err, t

                # ---- 5. report --------------------------------------------
                # CONTROLS mode reports continuously, not once a second: he is watching
                # the arm and the readout together to attribute a motion to a gesture,
                # and a 1 Hz readout is useless for that.
                if mode == "map":
                    # Both scales are always shown. Julien could not tell that ,/. were
                    # doing nothing here because only the active axis's resulting speed
                    # was displayed — a missing key looked identical to a key that worked.
                    speeds = (f"lin {args.linear_scale * CONTROLS_SCALE:.3f} m/s  "
                              f"rot {np.degrees(angular_scale * CONTROLS_SCALE):.0f}°/s"
                              f"{'' if rotation else ' (OFF)'}")
                    if last_active_axis is None:
                        print(f"\r[CONTROLS] push the puck …  {axes_readout(raw_axes)}  {speeds}   ",
                              end="", flush=True)
                    else:
                        drv = axis_map.motion_driven_by(last_active_axis)
                        if drv is None:
                            doing = "→ nothing (press 1-6 to assign)"
                        else:
                            v = axis_map.apply(isolated_axes(raw_axes, last_active_axis))[drv]
                            unit = (f"{v * args.linear_scale * CONTROLS_SCALE:+.3f} m/s" if drv < 3
                                    else f"{np.degrees(v * angular_scale * CONTROLS_SCALE):+.1f}°/s")
                            doing = f"→ {motions_for(control_frame)[drv]['short']} {unit}"
                        print(f"\r[CONTROLS] puck {PUCK_AXES[last_active_axis]:<5} "
                              f"{last_active_value:+.2f}  {doing:<28} {speeds}"
                              f"{' ' * 6}", end="", flush=True)
                elif t >= next_report:
                    next_report += 1.0
                    q = np.asarray(robot.get_joint_pos(), dtype=float)
                    extra = ""
                    if mode == "teleop" and teleop is not None:
                        extra = f"  EE {np.round(teleop.ee_position(), 3)}"
                        # ⭐ How far the goal is running ahead of the pose actually
                        # achieved. Pinned at the limit = the arm cannot follow (joint
                        # limit, singularity, something in the way), which used to
                        # present only as the arm behaving strangely. See
                        # CartesianTeleop._limit_lead().
                        lead_m, lead_r = teleop.lead()
                        if lead_m > 0.8 * teleop.max_lead_m or lead_r > 0.8 * teleop.max_lead_rad:
                            extra += f"  ⚠️ STUCK lead {lead_m * 100:.0f}cm/{np.degrees(lead_r):.0f}°"
                        # ⭐ Say WHY the arm feels slow. Near the workspace edge the
                        # solver needs several rad/s per joint for the same tip
                        # speed, so the twist gets throttled — and without this line
                        # that reads as unexplained sluggishness.
                        if teleop.speed_scale < 0.95:
                            extra += f"  ⚠️ SLOWED to {teleop.speed_scale * 100:.0f}% (near the reach limit)"
                    # ⭐ `jaw` is shown separately from `hottest` on purpose — see the
                    # comment where it is read. Watching this number plateau is the
                    # actual test of the 2π frame fix; watching `hottest` is not,
                    # because the shoulder sits hotter than the gripper all session.
                    # ⛔ "??" when the read failed, never a number. A fabricated 0 °C
                    # is exactly what made a disarmed thermal guard look healthy on
                    # screen, and the readout is the only place a human would notice.
                    therm = (f"hottest {hottest:4.0f}°C" if hottest is not None
                             else "hottest   ??°C ⚠️BLIND")
                    if jaw_temp is not None:
                        therm += f"  jaw {jaw_temp:4.0f}°C"
                    # ⭐ GUIDE reports DRIFT from where it went weightless. On 2026-08-10
                    # the arm sank to its own stops over ~33 s while this line calmly read
                    # "hottest 35°C" — because gravity compensation was 39% short at the
                    # elbow (FINDINGS §11) and nothing on screen was measuring the one
                    # quantity that was going wrong. The cause is fixed; the instrument
                    # should exist anyway. Same lesson as showing the jaw temperature
                    # separately: a readout must show what can fail, not what looks calm.
                    if mode == "guide" and guide_ref is not None:
                        sank = float(np.max(np.abs(q[:N_ARM] - guide_ref[:N_ARM])))
                        extra = f"  drift {sank:5.3f} rad ({np.degrees(sank):4.1f}°){extra}"
                    # ⭐ RECORDING HAS TO BE VISIBLE ON THE HEARTBEAT ROW, not only in the
                    # message that started it. A session where recording is silently still
                    # running produces a demonstration full of whatever happened next, and
                    # the operator finds out at training time. So it rides the one line
                    # that is always on screen.
                    rec = f"  ⏺ REC {t - take_t0:5.1f}s" if take is not None else ""
                    # ⭐ THE LOOP RATE, because it was 87 Hz for a whole session and nothing
                    # said so. It only became visible when a playback summary failed to add
                    # up. Shown only when it drops, so a healthy loop costs no width.
                    if loop_hz < 0.92 * CONTROL_HZ:
                        rec += f"  ⚠️{loop_hz:3.0f}Hz"
                    print(f"\r[{'CONTROLS' if mode == 'map' else mode.upper():8}] t={t:6.1f}s  {therm}"
                          f"{rec}  q {np.round(q[:N_ARM], 2)}{extra}   ", end="", flush=True)

                time.sleep(max(0.0, dt - (time.perf_counter() - loop_start)))

            # ---- controlled shutdown -----------------------------------------
            screen.done()
            print(f"\n⛔ stopping: {stop_reason}")

            # ⭐ CTRL-C IS A GRACEFUL SHUTDOWN, NOT A QUESTION. Julien, 2026-08-12:
            # *"if we hit control c it should just instantly go back into the starting
            # position and then disable itself, without allowing for the options,
            # because control c is typically just a quit."*
            #
            # He is right about the convention, and the park pose defaults to the pose
            # the arm was in when the session began — so this returns it to where he
            # left it and lets go, which is what "quit" should mean here.
            #
            # ⚠️ Note what this means: **Ctrl-C now MOVES the arm.** That is unusual
            # for an interrupt and is why it announces itself first and why a second
            # Ctrl-C stops the motion immediately. The motion itself is the same slow,
            # bounded, stall-guarded trajectory PARK always uses.
            #
            # ⛔ And it is automatic ONLY on the happy path. If the park stalls or the
            # chain dies, the arm is NOT released — it falls through to the consent
            # flow below, because "I could not reach the safe pose" is exactly when a
            # human should decide rather than a default.
            auto_parked = False
            if chain_alive(robot) and interrupted and park is not None:
                enter_hold()
                print("\n⭐ Ctrl-C — parking to the pose this session started in, then")
                print("   disabling. Press any key to stop the motion; Ctrl-C again forces out.")
                outcome = park_and_wait(robot, keys, park, clamp_gripper,
                                        ramp=park_ramp, speed=park_speed)
                if outcome == "arrived":
                    auto_parked = True
                    print("\n   Disabling the motors now.\n")
                else:
                    print(f"\n⚠️  the automatic park ended as {outcome!r}, so the arm is "
                          "NOT being released. Choose below.")

            if chain_alive(robot) and not auto_parked:
                enter_hold()
                print("\nThe arm is HOLDING its pose. It will not be released until you choose.")
                print("   p = PARK — drive back to the park pose, then it holds there")
                print("   g = go weightless so you can park it by hand")
                print("   d = disable now (⚠️ a raised arm will sag)")
                while True:
                    k = keys.get()
                    if k == "p":
                        # ⭐ Julien's request: *"it would also be good to do park mode
                        # [at quit], because then I can do park mode and then disable…
                        # I don't have to do anything with my hands."* With the park
                        # pose defaulting to wherever the arm started, `q p d` is a
                        # complete hands-free shutdown — and Ctrl-C now does the same
                        # thing in one keystroke.
                        park_and_wait(robot, keys, park, clamp_gripper)
                        enter_hold()
                        print("   p = park again    g = weightless    d = disable")
                    elif k == "g":
                        enter_guide()
                        print("\n⭐ weightless — park the arm, then press d to disable.")
                    elif k == "d":
                        break
                    time.sleep(0.05)
                    if not chain_alive(robot):
                        print("\n⚠️  the chain died while waiting — disabling now.")
                        break
            elif not chain_alive(robot):
                # ⚠️ `elif not chain_alive(...)`, not a bare `else`. With the Ctrl-C
                # auto-park above, a plain `else` would fire on the SUCCESS path and
                # announce a dead chain to someone whose arm had just parked fine.
                print("⚠️  the chain is already dead, so the arm is NOT being commanded.")
                print("   It will be sagging under gravity. Support it now if it is raised.")

    except KeyboardInterrupt:
        print("\ninterrupted.")
    except Exception as exc:  # noqa: BLE001
        print(f"\n⛔ {type(exc).__name__}: {exc}")
    finally:
        # Hand SIGINT back before anything else, so a Ctrl-C during shutdown behaves
        # the way the shell expects rather than being swallowed by our handler.
        try:
            signal.signal(signal.SIGINT, signal.SIG_DFL)
        except Exception:  # noqa: BLE001, S110
            pass
        try:
            handle.close()
        except Exception:  # noqa: BLE001, S110
            pass
        # ⛔ DO NOT make this an unconditional save again.
        #
        # It was one, and on 2026-08-10 it wrote a map mangled by the old bind-on-
        # deflection MAP mode straight over Julien's hand-dialled file. The values had
        # been produced on real hardware and were only recoverable because the file
        # happened to be committed. Two changes: nothing is written unless the map
        # actually changed, and the previous contents are kept alongside it.
        map_store.set(args.arm, axis_map, control_frame)
        if map_store != map_store_at_start:
            try:
                if MAP_FILE.exists():
                    BACKUP_FILE.write_text(MAP_FILE.read_text())
                map_store.save(MAP_FILE)
                print(f"\naxis map CHANGED and saved → {MAP_FILE.relative_to(REPO)}")
                print(f"  scope: {map_store.scope_note(args.arm)}")
                print(f"  previous contents kept in {BACKUP_FILE.relative_to(REPO)}")
            except Exception as exc:  # noqa: BLE001
                print(f"\n⚠️  could not save the axis map: {type(exc).__name__}: {exc}")
        if robot is not None:
            _SHUTTING_DOWN["yes"] = True
            disabled = shutdown_robot(robot)
            print(f"\nmotors confirmed disabled: {disabled}")

    print(f"\nhottest motor seen this session: {thermal.max_seen:.0f}°C")
    if thermal.max_jaw_seen:
        # The number that decides whether the gripper frame fix held. A plateau near
        # idle (31-36 °C) is the pass; a steady climb is the failure, and it is
        # invisible in `hottest` because the shoulder runs hotter all session.
        print(f"hottest the GRIPPER (motor 7) got: {thermal.max_jaw_seen:.0f}°C")
    print(f"axis map: {axis_map.one_line(control_frame)}")
    if axis_map != axis_map_at_start:
        print(f"     was: {axis_map_at_start.one_line()}")
    else:
        print("     unchanged — nothing was written.")
    if axis_map.unbound():
        names = ", ".join(motions_for(control_frame)[i]["short"] for i in axis_map.unbound())
        print(f"  ⚠️  UNBOUND, the arm will not perform these: {names}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
