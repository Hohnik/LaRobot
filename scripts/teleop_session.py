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
# ⚠️ `Any` was used in this file's annotations since long before this import existed, and
# it worked only because `from __future__ import annotations` never evaluates them. A real
# import is needed the moment it appears on a variable inside `main()`, because
# `scripts/check_restructure.py` check 4 resolves every name used there.
from typing import Any

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
from arm_session import ArmSelector, ArmSession, parse_arms  # noqa: E402
from incident import describe, write_incident  # noqa: E402
from mirror import (  # noqa: E402
    DEFAULT_ALIGN_SPEED,
    DEFAULT_MAX_GAP,
    MirrorLink,
    pick_pair,
)
from motion import EASINGS, JointPath, easing_factor  # noqa: E402
from recording import (  # noqa: E402
    Layout,
    TrackingLog,
    Trajectory,
    replay_step,
    safe_time_scale,
)
from screen import StatusLine, display_width  # noqa: E402
from teleop import (  # noqa: E402
    FLOOR_LIMIT,
    FRAMES,
    REACH_LIMIT,
    CartesianTeleop,
    clamp_to_workspace,
    effective_limits,
    workspace_room,
)
from yam_can import ARM_SERIALS, DEFAULT_ARM, YAM_JOINTS  # noqa: E402
from yam_robot import (  # noqa: E402
    SAFE_MAX_SPEED,
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

# ⚠️ `WORKSPACE_BOX = 0.30` used to live here. The workspace limit is now
# `REACH_LIMIT` and `FLOOR_LIMIT` in `src/teleop.py`, next to the code that applies
# them, because the old constant sat in the script while the clamp it fed was an
# untested inline block. FINDINGS §43.
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
# ⭐ How fast the follower closes the initial gap in MIRROR mode. ⚠️ IMPORTED from
# `src/mirror.py` rather than repeated, so there is ONE number. It is aliased here only
# because the plan line quotes it to the operator before they press Enter — the first draft
# of this line wrote `0.30` next to a comment claiming it came from the module, which is the
# staleness pattern in miniature: a duplicate plus a sentence asserting there is no duplicate.
MIRROR_ALIGN_SPEED = DEFAULT_ALIGN_SPEED
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
  ARMS      a  which arm the MODE keys aim at (B → G → BOTH). Driving always drives
               every arm; only mode changes and edits are aimed
  MIRROR    i  the SELECTED arm leads, the other follows it joint for joint. Shows the
               plan and waits for Enter; i again turns it off. Hand-guide the leader
               in GUIDE and hold it still until the row says FOLLOWING
  OTHER     r  wrist rotation on/off   ?  help
  QUIT      q  then: q = park+disable (all of it)   p = park   g = weightless   d = disable
            ⭐ to park WITHOUT quitting, press p in the session, then t to carry on
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


def _safe_fact(fn) -> Any:  # noqa: ANN001
    """Read one value for the incident file, or record why it could not be read.

    ⛔ Runs on the shutdown path, where half of these reads throw: the chain may be
    dead, `teleop` may be `None` because the session never entered TELEOP, and a local
    may be unbound if the loop never ran a cycle. **A missing field must never become
    an exception during teardown.** See `src/incident.py` for the same rule stated once.
    """
    try:
        return fn()
    except Exception as exc:  # noqa: BLE001
        return f"<unavailable: {type(exc).__name__}: {exc}>"


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


def park_arms(arms: list, keys, clamp_gripper, easing=EASINGS[2],  # noqa: ANN001
              stall_seconds: float = PARK_STALL_SECONDS) -> str:
    """Drive EVERY arm to its own base pose, **blocking**, and say how it ended.

    Returns the worst outcome across the arms: `"dead"` · `"stalled"` · `"stopped"`
    (a key was pressed) · `"arrived"`. Per-arm detail is printed as it happens.

    ⭐⭐ EVERY ARM ADVANCES ON EVERY CYCLE rather than one arm after another, and the
    reason is not speed. Sequential parking would make *"any key stops it"* stop only
    the arm currently moving, and it would leave the other arm holding a pose for the
    whole of the first arm's park with nobody watching it. One loop, N commands.

    ⚠️ A DEAD ARM IS SKIPPED, NOT FATAL. If one chain has died that arm cannot be
    commanded at all — it is already sagging — while the live arm can still be parked,
    which is better than leaving it holding and far better than disabling it. The dead
    one is named, loudly, because someone may need to catch it.

    ⛔ THE DUPLICATION THIS REMOVES, kept from the single-arm version because it is still
    the reason this function exists: the quit path used to carry its own copy of this
    loop, and the code's own comment admitted the risk — *"This is a SECOND park loop,
    and duplication is what has bitten this repo four times."*

    ⚠️ The *interleaved* park (mode == "park") is deliberately NOT folded in here. That
    one advances a single step per control cycle so the operator can still press keys and
    the temperature guard still runs; this one blocks because the session is already
    ending. Same trajectory maths, different scheduling — collapsing them would mean a
    blocking call inside the 100 Hz loop.

    ⚠️ `stall_seconds` is a parameter only so the tests can ask for a 0.2 s patience
    instead of waiting the real 4 s. Nothing in the session passes it.
    """
    #: One entry per arm that can still be commanded: where it is going, what it was last
    #: told, and when it last made progress.
    runs = []
    for one in arms:
        if not one.alive():
            print(f"\n⚠️  arm {one.name}: the chain is dead, so it cannot be parked. "
                  "It is sagging under gravity — support it if it is raised.")
            continue
        if one.base_pose is None:
            print(f"\n⚠️  arm {one.name}: no base pose saved, so there is nothing to park to.")
            continue
        tgt, warn = park_target_from(one.robot.get_joint_pos(), one.base_pose,
                                     gripper_index=N_ARM, clamp=clamp_gripper)
        if warn:
            print(f"\n  ⚠️  arm {one.name}: {warn}.")
        cmd = np.asarray(one.robot.get_joint_pos(), dtype=float)
        runs.append({
            "arm": one, "tgt": tgt, "cmd": cmd, "start": cmd.copy(),
            "best": float(np.max(np.abs(tgt - cmd))),
            "last_progress": time.perf_counter(), "outcome": None,
            # ⭐ The baseline for "how often did SafeRobot hold the command back during THIS
            # park". A running total since the session began would say nothing about it.
            "clipped_at": getattr(one.robot, "limited_cycles", 0),
        })
    if not runs:
        return "dead"

    # ⛔ DISCARD ANYTHING TYPED BEFORE THIS MOVE EXISTED. "Any key stops it" must mean a
    # key pressed *at* the moving arm, not one left over from teleop or from the menu that
    # led here. Julien saw a park announce itself and stop in the same breath — the stale
    # keystroke that cancelled it had been typed seconds earlier.
    keys.drain()
    for run in runs:
        print(f"\n⭐ PARKING arm {run['arm'].name} to "
              f"{np.round(np.asarray(run['tgt'])[:N_ARM], 2)} — any key stops it.")
    print()

    while any(run["outcome"] is None for run in runs):
        now = time.perf_counter()
        # ⛔ A key stops EVERY arm, checked once per cycle rather than per arm. "Any key
        # stops it" has to mean the whole motion, or the operator presses a key, watches
        # one arm halt, and has to guess about the other.
        if keys.get() is not None:
            for run in runs:
                if run["outcome"] is None:
                    run["outcome"] = "stopped"
            print("\n  park stopped.")
            break
        for run in runs:
            if run["outcome"] is not None:
                continue
            one = run["arm"]
            if not one.alive():
                print(f"\n⚠️  arm {one.name}: the chain died while parking.")
                run["outcome"] = "dead"
                continue
            meas = np.asarray(one.robot.get_joint_pos(), dtype=float)
            err = float(np.max(np.abs(run["tgt"] - meas)))
            since = now - run["last_progress"]
            verdict = park_verdict(err, since > stall_seconds,
                                   PARK_TOLERANCE, PARK_SETTLED,
                                   stopped_briefly=since > PARK_SETTLE_SECONDS)
            if verdict in ("arrived", "settled"):
                extra = ("" if verdict == "arrived" else
                         " — as close as the arm holds itself; the last fraction of a "
                         "degree is the controller settling under load")
                print(f"\n⭐ arm {one.name} PARKED ({err:.3f} rad off{extra}).")
                run["outcome"] = "arrived"
                continue
            if verdict == "blocked":
                # ⛔⭐ IT REPORTS WHAT IT MEASURED, then offers the guesses. This message read
                # *"Something is in the way, or the pose is unreachable"* and Julien's answer
                # to the same wording in MIRROR was *"the robot was never blocked by
                # anything."* The three numbers below distinguish the cases: how far the
                # COMMAND ran ahead of the arm, how often `SafeRobot` held it back, and
                # whether the arm was moving at all.
                lag_now = float(np.max(np.abs(run["cmd"] - meas)))
                clipped = getattr(one.robot, "limited_cycles", 0) - run["clipped_at"]
                print(f"\n⛔ arm {one.name} PARK BLOCKED — {err:.3f} rad still to go, no "
                      f"progress for {stall_seconds:.0f}s.")
                print(f"     the command ran {lag_now:.3f} rad ahead of the arm; SafeRobot "
                      f"held it back on {clipped} cycle(s) of this park")
                if lag_now > 0.8 * getattr(one.robot, "max_lag", 0.25):
                    print("     ⚠️ That is at SafeRobot's following-error limit, so the arm "
                          "is being asked for more than it is delivering: something is "
                          "resisting it, or the pose needs more torque than it has.")
                else:
                    print("     ⚠️ The command is NOT running ahead, so the arm is following "
                          "and the target itself is the problem: the pose may be "
                          "unreachable from here.")
                run["outcome"] = "stalled"
                continue
            if err < run["best"] - PARK_PROGRESS_EPS:
                run["best"], run["last_progress"] = err, now
            # Same ease-in/ease-out as the interleaved park — Ctrl-C should not shut down
            # with a jerk at both ends when a mid-session park glides.
            # ⭐ EASINGS[2] is "out": full speed from the first step, soft landing. Julien on
            # Ctrl-C: *"I want it to move into its parking position quickly and swiftly,
            # without the excessive starting and pausing."* A shutdown move should leave at
            # once; only the arrival needs to be gentle.
            factor = easing_factor(easing,
                                   float(np.max(np.abs(run["cmd"] - run["start"]))),
                                   float(np.max(np.abs(run["tgt"] - run["cmd"]))),
                                   one.park_ramp)
            run["cmd"] = advance_park_command(run["cmd"], run["tgt"],
                                              one.park_speed * factor / CONTROL_HZ)
            one.robot.command_joint_pos(run["cmd"])
        time.sleep(1.0 / CONTROL_HZ)

    # ⭐ The WORST outcome, because the caller uses it to decide whether the arms may be
    # released. One arm that stalled is a reason to keep every arm holding and ask a human.
    order = ["dead", "stalled", "stopped", "arrived"]
    outcomes = [run["outcome"] or "stopped" for run in runs]
    return next(o for o in order if o in outcomes)


def status_row(one: ArmSession, lead: str, reach: float, floor: float,
               note: str = "") -> str:
    """ONE arm's heartbeat row: its mode, its temperatures, its pose, its warnings.

    `lead` is the session's own facts — the clock, the recording, the loop rate — which
    ride the FIRST row only. Later rows get a run of spaces of the same display width, so
    the temperature columns line up down the block.

    ⭐⭐ WHY THIS IS A FUNCTION AND NOT SIXTY LINES INSIDE THE LOOP. It was those sixty
    lines until 2026-08-14, which meant the row a human reads to know what the arm is
    doing could only ever be executed on the arm — no test could reach it, and a
    formatting error in it would surface as a session dying one second after starting.
    As a function it is testable against a fake arm, which is the same argument
    `ArmSession` itself is built on: **the class decides, the script narrates, and the
    narration is worth proving too.**

    ⚠️ It reads and formats. It must never command anything, and it must never raise —
    a display fault has no business stopping a session that is holding 4.3 kg.
    """
    q = np.asarray(one.robot.get_joint_pos(), dtype=float)
    extra = ""
    if one.mode == "teleop" and one.teleop is not None:
        extra = f"  EE {np.round(one.teleop.ee_position(), 3)}"
        # ⭐⭐ SHOW THE WORKSPACE WALL, because it used to be invisible. Julien,
        # 2026-08-13: *"it stops moving in the direction I want it to move even though the
        # arm hasn't even close to fully extended."* Showing it settled the cause in one
        # session, and the limit is now a fixed sphere around the base plus a floor rather
        # than a cube that moved every time TELEOP was entered. FINDINGS §41.1 and §43.
        lim_r, lim_f = effective_limits(one.home_ee, reach, floor)
        ee_now = one.teleop.ee_position()
        out, up = workspace_room(ee_now, lim_r, lim_f)
        extra += f"  reach {out:.2f}/{lim_r:.2f}m"
        if out > 0.9 * lim_r:
            extra += " ⚠️ AT THE EDGE"
        # ⚠️ The floor is only worth screen space when it is close. With the floor at the
        # base plane this starts warning around z = 0, which is roughly desk height and
        # exactly where a pick happens. So it reads as "you are down at the desk" rather
        # than as an alarm.
        if up < 0.10:
            extra += f"  ⚠️ {up * 100:.0f}cm above the floor (z={ee_now[2]:+.2f})"
        # ⭐ How far the goal is running ahead of the pose actually achieved. Pinned at the
        # limit = the arm cannot follow (joint limit, singularity, something in the way),
        # which used to present only as the arm behaving strangely. See
        # CartesianTeleop._limit_lead().
        lead_m, lead_r = one.teleop.lead()
        if lead_m > 0.8 * one.teleop.max_lead_m or lead_r > 0.8 * one.teleop.max_lead_rad:
            extra += f"  ⚠️ STUCK lead {lead_m * 100:.0f}cm/{np.degrees(lead_r):.0f}°"
        # ⭐ Say WHY the arm feels slow. Near the workspace edge the solver needs several
        # rad/s per joint for the same tip speed, so the twist gets throttled — and without
        # this line that reads as unexplained sluggishness.
        # ⚠️ The message names the reach limit as the cause and the data does not always
        # support that: at sigma_min 0.1713, which the throttle's own docstring calls
        # comfortable, it printed "SLOWED to 19%" (FINDINGS §41.2). Tracked as ROADMAP
        # §8.2 item 21; the wording is not fixed here because the fix is a measurement.
        if one.teleop.speed_scale < 0.95:
            extra += f"  ⚠️ SLOWED to {one.teleop.speed_scale * 100:.0f}% (near the reach limit)"
    # ⭐ `jaw` is shown separately from `hottest` on purpose. Watching this number plateau
    # is the actual test of the 2π gripper frame fix; watching `hottest` is not, because
    # the shoulder sits hotter than the gripper all session.
    # ⛔ "??" when the read failed, never a number. A fabricated 0 °C is exactly what made
    # a disarmed thermal guard look healthy on screen (FINDINGS §24.1), and the readout is
    # the only place a human would have noticed.
    therm = (f"hottest {one.hottest:4.0f}°C" if one.hottest is not None
             else "hottest   ??°C ⚠️BLIND")
    if one.jaw_temp is not None:
        therm += f"  jaw {one.jaw_temp:4.0f}°C"
    # ⭐ GUIDE reports DRIFT from where it went weightless. On 2026-08-10 the arm sank to
    # its own stops over ~33 s while this line calmly read "hottest 35°C" — gravity
    # compensation was 39% short at the elbow (FINDINGS §11) and nothing on screen was
    # measuring the one quantity that was going wrong. The cause is fixed; the instrument
    # should exist anyway. A readout must show what can fail, not what looks calm.
    if one.mode == "guide" and one.guide_ref is not None:
        sank = float(np.max(np.abs(q[:N_ARM] - one.guide_ref[:N_ARM])))
        extra = f"  drift {sank:5.3f} rad ({np.degrees(sank):4.1f}°){extra}"
    # ⭐ `note` is whatever the session knows about this arm that the arm does not: today
    # only the mirror link's state, which is a relationship between two arms and therefore
    # cannot live on either.
    if note:
        extra = f"  {note}{extra}"
    label = "CONTROLS" if one.mode == "map" else one.mode.upper()
    return (f"[{one.name} {label:8}]{lead}  {therm}"
            f"  q {np.round(q[:N_ARM], 2)}{extra}   ")


def main() -> int:  # noqa: PLR0915
    ap = argparse.ArgumentParser(description="Interactive YAM session: guide, teleop, park.")
    ap.add_argument("--yes", action="store_true", help="actually energise the arm")
    # ⭐⭐ TWO SPELLINGS OF ONE IDEA, and `--arm` is the one that must not break.
    # Every other script here takes `--arm` (`ping_motors.py`, `identify_arm.py`,
    # `check_arms_match.py`), it is in every document, and it is what Julien types.
    # `--arms` is the N-arm spelling ROADMAP §6.1 step 2 asks for. They agree or the
    # session refuses; `src/arm_session.py::parse_arms` holds the rules and the tests.
    ap.add_argument("--arm", default=None, choices=sorted(ARM_SERIALS),
                    help=f"the arm, when there is one (default {DEFAULT_ARM})")
    ap.add_argument("--arms", default=None, metavar="B[,G]",
                    help="the arms this session drives, comma separated. ⛔ Two arms is "
                         "not runnable yet (ROADMAP §6.1 step 3) and passing two errors "
                         "out — see the refusal for what is still missing")
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
    # ⭐⭐ `--box` IS GONE, replaced by `--reach` and `--floor` on 2026-08-14 by Julien's
    # decision. ⛔ Deliberately removed rather than kept as an alias that silently means
    # something else: the same call `src/yam_can.py` made when `--arm arm1` became
    # `--arm B`. A flag that keeps working while its meaning has changed underneath is
    # worse than one that fails loudly. `--box` now errors, which is the point.
    ap.add_argument("--reach", type=float, default=REACH_LIMIT,
                    help=f"how far the tip may go from the BASE, in metres (default "
                         f"{REACH_LIMIT}). Replaced a ±0.30 m cube that re-centred on "
                         f"wherever TELEOP was entered, so the wall moved every session "
                         f"and stopped him at 71% of the arm's reach. The arm can reach "
                         f"about 0.74 m. ⛔ A safety limit: raise it deliberately.")
    ap.add_argument("--floor", type=float, default=FLOOR_LIMIT,
                    help=f"lowest the tip may go, in metres relative to the base plane "
                         f"(default {FLOOR_LIMIT}). ⚠️ It bounds a GROSS downward excursion "
                         f"and it is NOT desk protection: this arm can otherwise put its "
                         f"tip 0.377 m below its own base, and where the desk sits has "
                         f"never been measured. ⛔ Do NOT raise it above 0 — Julien's point, "
                         f"2026-08-14: a floor above the desk means nothing can be picked "
                         f"up off it.")
    ap.add_argument("--max-speed", type=float, default=SAFE_MAX_SPEED,
                    help=f"the ceiling on every commanded joint speed, rad/s (default "
                         f"{SAFE_MAX_SPEED}). ⛔ A SAFETY LIMIT, and the one that binds "
                         f"everything: SafeRobot clamps every command from every mode to it, "
                         f"below all control logic, so the park speed and any playback "
                         f"multiplier never bind above it. ⚠️ It is a SOFTWARE limit, not the "
                         f"hardware's — hand-guided recordings reach 2.4 to 3.7 rad/s "
                         f"(FINDINGS §37.2). Raise it one step at a time (1.0 → 1.5 → 2.0) and "
                         f"watch the STUCK lead warning rather than temperature")
    ap.add_argument("--mirror", default="copy", choices=["copy", "mirror"],
                    help="how the follower reproduces the leader in MIRROR mode (key i). "
                         "copy = the same joint angles, correct for arms standing SIDE BY "
                         "SIDE, which is how they stand today. mirror = negate the joints "
                         "that reverse under reflection, for arms FACING each other. "
                         "⚠️ The mirror signs are a geometric prediction and have never "
                         "been tried on hardware")
    ap.add_argument("--teleop-speed", type=float, default=MAX_PLANNED_JOINT_SPEED,
                    help=f"the ceiling on a PLANNED joint speed, rad/s (default "
                         f"{MAX_PLANNED_JOINT_SPEED}). ⛔ A SAFETY LIMIT, and a DIFFERENT one "
                         f"from --max-speed: this clamps how far TELEOP's inverse kinematics "
                         f"may move a joint per cycle, and it caps a playback and a park. "
                         f"⚠️ It binds BELOW --max-speed, so raising --max-speed alone leaves "
                         f"teleop exactly as fast as it was — which is why raising it felt "
                         f"like nothing happened (FINDINGS §37.0). Raise both to go faster")
    ap.add_argument("--mirror-gap", type=float, default=DEFAULT_MAX_GAP,
                    help=f"how far the MIRROR follower may fall behind the leader before the "
                         f"link stops, in radians (default {DEFAULT_MAX_GAP}). ⚠️ A TOLERANCE "
                         f"rather than a speed: past a certain leader speed the follower is "
                         f"tracking as hard as it can and still losing ground, because "
                         f"SafeRobot clips every command to 0.25 rad from the measured "
                         f"position. Loosening this lets the copy lag further behind rather "
                         f"than stopping; it does not make the follower faster")
    ap.add_argument("--fork-map", action="store_true",
                    help="give THIS arm its own axis map, copied from the one it uses now. "
                         "Without this, both arms share one map and editing changes both")
    ap.add_argument("--share-map", action="store_true",
                    help="drop this arm's own axis map and go back to the shared one")
    args = ap.parse_args()
    if args.fork_map and args.share_map:
        ap.error("--fork-map and --share-map are opposites; pass at most one")

    # ⭐⭐ THE LIST OF ARMS THIS SESSION DRIVES. ROADMAP §6.1 step 2.
    #
    # ⚠️ `arm_names[0]` appears below wherever a line still assumes one arm, on purpose:
    # each one marks a site step 2's remaining work has to turn into a loop, and it is
    # greppable. Sites that run after the object exists use `arm.name` instead.
    try:
        arm_names = parse_arms(args.arm, args.arms, ARM_SERIALS, DEFAULT_ARM)
    except ValueError as exc:
        ap.error(str(exc))
    # ⛔⭐⭐ TWO ARMS NOW RUN, AND `--start-mode guide` IS REFUSED FOR THEM.
    #
    # ROADMAP §6's ruling, and it is the one refusal that replaced the blanket one: **two
    # arms going weightless on a first run is the worst possible first run.** `g` reaches
    # the same state, but only after the operator has selected BOTH deliberately and
    # pressed a key at a rig they are watching. A flag does it before anything is on screen.
    #
    # ⚠️ The blanket "two arms cannot run yet" refusal lived here until 2026-08-14 and is
    # gone, along with the test that pinned it. That deletion is deliberate: the refusal
    # existed because the script below was single-arm, and it no longer is.
    if len(arm_names) > 1 and args.start_mode == "guide":
        ap.error(
            f"--arms {','.join(arm_names)} --start-mode guide: refused. That would make "
            f"{len(arm_names)} arms weightless before anything is on screen, and GUIDE is "
            "the mode where an error in the dynamics model becomes a falling arm rather "
            "than a droop. Start in hold, then press g once you are watching.")

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

    # ⭐ The per-cycle joint step TELEOP is allowed, derived from the flag so there is one
    # number rather than a constant and a flag that can disagree. `MAX_JOINT_STEP` remains as
    # the documented default and is what the flag's own default comes from.
    joint_step = args.teleop_speed / CONTROL_HZ
    threading.excepthook = _quiet_expected_server_exit
    rotation = not args.no_rotation
    start_frame = args.frame
    # ⛔ The store decides WHICH map this arm uses — its own override if it has one,
    # otherwise the shared one. Editing a shared map changes both arms, so the scope
    # is printed in the plan and again at exit. Never leave that implicit.
    map_store = AxisMapStore.load(MAP_FILE)
    # ⭐ Each named arm, so `--fork-map` with two arms gives EACH of them its own map
    # rather than only the first. That is what the flag's own help says it does ("give
    # THIS arm its own axis map"), and with one arm it is exactly what it did before.
    for name in arm_names:
        if args.fork_map:
            map_store.fork(name)
        elif args.share_map:
            map_store.unfork(name)
    map_store_at_start = map_store.copy()
    # ⚠️ No `axis_map` local any more. Each arm carries its own map and its own
    # start-of-session copy, handed to `ArmSession` at construction so it cannot be
    # forgotten. The store stays the source; this file only reads it here and at exit.
    # `scripts/check_restructure.py` RETIRED_LOCALS keeps the old name from coming back.
    # ⭐⭐ THE BASE POSE AND THE WAYPOINTS ARE DIFFERENT THINGS. Julien's ruling,
    # 2026-08-12: *"the control-c park to disable needs to always go back to the stable
    # parking save. If I save a new parking option it shouldn't go back to that and then
    # disable. It should always go back to the base parking option."*
    #
    # ⛔ That is a safety requirement, not a preference. Ctrl-C is the "get me out of
    # here" key: it parks and then RELEASES the motors, so the pose it chooses must be
    # arm that is safe to be let go in. A waypoint saved mid-task — arm extended over
    # the desk, gripper holding something — is exactly what that must never be.
    #
    # So `park` (slot 0, the base) is only ever changed deliberately with `s 0`, while
    # `s 1`…`s 9` fill waypoints that Ctrl-C ignores completely.
    # ⚠️ Read here, per arm, and handed to `ArmSession` at construction. There is no
    # session-level `park` or `slots` any more: both are this arm's own, keyed by arm name
    # in `config/park_pose.json` since 2026-08-12. `check_restructure.py` RETIRED_LOCALS
    # keeps the old names from coming back.
    saved_slots = {name: park_slots(load_json(PARK_FILE, {}), name) for name in arm_names}
    # ⚠️ The eleven park fields used to be initialised here. They are `ArmSession`
    # fields now, and the class's constructor sets every arm to the identical value —
    # `PARK_SPEED` 0.40 and `PARK_RAMP` 0.20 are the same constant in both files, which
    # was checked before the move rather than assumed.
    # ⛔ Leaving them here as `arm.park_* = …` would run BEFORE `arm` exists, which is
    # the fault `scripts/check_restructure.py`'s ordering check exists to catch. It
    # caught all eleven. FINDINGS §48.
    # The blended path being followed, the cursor along it, and where each waypoint
    # falls so the readout can say which arm it is heading for.
    blend_idx = 1                       # "smooth" — the sensible default
    ease_idx = 3                        # "both" — see motion.EASINGS
    # ⛔⭐ TWO CLOCKS, AND CONFLATING THEM PRINTED A WRONG NUMBER FOR A DAY.
    # `park_leg_t` is reset every time the cursor passes a waypoint, because Julien asked
    # for each leg's own duration. `park_start_t` is set once and never reset, because the
    # arrival message wants the whole park. Using `park_leg_t` for both reported
    # **"PARK reached in 0.0s"** on a park that had just taken 4.4 seconds: the last leg's
    # mark is passed at the end of the path, so the reset happened moments before arrival.
    # See [FINDINGS §34.3](../docs/FINDINGS.md).
    # ⭐⭐ HAND-TAUGHT MOVEMENTS. Julien, 2026-08-12: *"arm good idea is definitely
    # recording everything in the guide mode and then replaying it. That's a smart idea,
    # definitely."* `w` records, `l` plays arm back. The reasoning for why this may beat
    # saved waypoints is docs/ROADMAP.md §6.6; the movement itself lives in
    # src/recording.py so every decision about it is testable without an arm.
    #
    # ⛔⭐ THIS BLOCK DOES **NOT** MOVE INTO `ArmSession`, and an earlier note here said it
    # did. Corrected 2026-08-13 after checking it against the target data format.
    # `amazon-far/abc` wants `states_actions.bin` with **14 states and 14 actions per
    # timestep — two arms in ONE timeline** ([ROADMAP.md](../docs/ROADMAP.md) §9.2). A
    # recorder owned by an arm produces arm file per arm and **cannot** produce that.
    # ⭐ So recording and playback are **session-level and span every arm**: arm recorder
    # samples all arms each cycle, and arm playback cursor drives them all. Splitting the
    # cursor per arm would let the two arms drift apart in time, which is the arm thing a
    # bimanual demonstration must not do. Migration map: [ROADMAP.md](../docs/ROADMAP.md)
    # §6.1.
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
    # ⭐⭐ WHICH ARMS A PLAYBACK DRIVES, in the RECORDING's order, and how its samples map
    # onto them. Both come from the file's own metadata rather than from the session, so a
    # recording made with `--arms B,G` cannot be replayed onto `--arms G,B` with each arm
    # driven by the other's joints.
    replay_layout: Layout | None = None
    replay_arms: list[ArmSession] = []
    #: ⛔ Which of those arms have finished parking to the start pose. The playback may not
    #: begin until EVERY one has: a two-arm demonstration whose arms start seconds apart is
    #: not the demonstration that was recorded, and the whole point of one timeline is that
    #: they stay in step.
    replay_ready: set[str] = set()
    # ⭐⭐ ONE ARM FOLLOWS THE OTHER, joint for joint. Julien's idea, 2026-08-11: *"be able
    # to move one of the arms in the guide mode and have the second arm just mirror the exact
    # movements with zero latency."*
    #
    # ⚠️ SESSION-LEVEL, because a mirror is a RELATIONSHIP between two arms rather than a
    # property of either. The follower's `mode` is `"mirror"`; the leader keeps whatever mode
    # it is in, which is the point — hand-guide it in GUIDE and the follower copies.
    #
    # ⛔ The engagement logic lives in `src/mirror.py::MirrorLink` and has 18 tests, because
    # the risky part is the FIRST cycle: the two arms are never in the same pose, so
    # commanding the leader's angles straight across would make the follower jump the gap.
    mirror_link: MirrorLink | None = None
    #: ⚠️ Declared here so the stop report can read it even if it somehow runs before a link
    #: was ever engaged. It is a count, so 0 is the honest starting value.
    mirror_clipped_at = 0
    mirror_leader: ArmSession | None = None
    mirror_follower: ArmSession | None = None
    # A pending `s` or `p` waiting for its digit, and the sequence being typed after `p`.
    pending: str | None = None
    park_sequence: list[str] = []
    angular_scale = ANGULAR_SCALE
    gripper_step = args.gripper_step
    # ⚠️ CONTROLS mode's memory of "the control you just used" — `last_active_axis`,
    # `last_active_value` and `last_input_kind` — plus `learn_button` and `buttons_prev`
    # were declared here. They are `ArmSession` fields now: two pucks have two answers to
    # "which control did you just use", and the class's constructor sets exactly the values
    # these lines did.
    # ⛔ Not left here as `arm.last_active_axis = None`: that would run before `arm` exists,
    # which is the fault `scripts/check_restructure.py` check 3 catches.

    print("=== plan ===")
    # ⭐ Named ARMS, plural, and it prints the serial of each. With arm arm the line reads
    # exactly as it did before apart from the label, so nothing he checks before pressing
    # --yes has moved.
    for name in arm_names:
        print(f"  ARM         : {name}  (serial {ARM_SERIALS[name]})")
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
    # ⭐ One map line per arm, read from the store — the same values the arms are about to
    # be built with. With arm arm the line reads as it always did apart from the arm's name.
    for name in arm_names:
        plan_map = map_store.for_arm(name, start_frame)
        print(f"  axis map {name}  : {plan_map.one_line(start_frame)}   (m to change it live)")
        print(f"  map scope   : {map_store.scope_note(name)}")
        if plan_map.unbound():
            names = ", ".join(motions_for(start_frame)[i]["short"] for i in plan_map.unbound())
            print(f"  ⚠️  UNBOUND  : {names} — arm {name} will NOT perform these until they "
                  "are bound (m)")
    print(f"  control fr. : {CartesianTeleop.FRAME_NOTES[start_frame]}  (v cycles it live)")
    for name in arm_names:
        base = saved_slots[name].get(BASE_SLOT)
        print(f"  park pose {name} : "
              f"{np.round(base, 3).tolist() if base else 'none saved — press s to set arm'}")
    print(f"  workspace   : {args.reach} m from the base, tip stays above {args.floor} m")
    # ⭐ The ceiling that binds every mode, printed because it was invisible for four days
    # and explained every "why is it so slow" question in that time (FINDINGS §37.0).
    # ⭐⭐ EVERY LAYER, BECAUSE ONLY THE LOWEST ONE BINDS. Julien raised `--max-speed` to 5
    # and teleop felt identical, because the per-cycle IK clamp is a separate 1.5 rad/s and it
    # sits below. Four days were lost to the same invisibility once already (FINDINGS §37.0),
    # and the fix then was to name the number; the fix now is to show which one wins.
    raised = []
    if args.max_speed != SAFE_MAX_SPEED:
        raised.append("--max-speed")
    if args.teleop_speed != MAX_PLANNED_JOINT_SPEED:
        raised.append("--teleop-speed")
    note = f"  ⚠️ RAISED: {', '.join(raised)}" if raised else ""
    print(f"  joint speed : teleop {min(args.teleop_speed, args.max_speed):.2f} · "
          f"planned {min(args.teleop_speed, args.max_speed):.2f} · "
          f"mirror {args.max_speed:.2f} rad/s{note}")
    print(f"                (SafeRobot caps everything at {args.max_speed:.2f} rad/s AND holds "
          f"the command within 0.25 rad of the measured pose)")
    print(f"  temperature : warn {TEMP_WARN}°C, stop {TEMP_STOP}°C")
    print(HELP)

    if not args.yes:
        print("DRY RUN — nothing transmitted, nothing energised. Re-run with --yes.")
        return 0

    # ⚠️ ONE PUCK, and with two arms this becomes arm call per arm with `exclude=` holding
    # the ones already taken — `pick_device_by_wiggle` already supports that and it is
    # tested (`scripts/test_puck_assignment.py`). Not wired yet: ROADMAP §6.1 step 2.
    # ⭐⭐ ONE PUCK PER ARM, ASSIGNED BY BEING MOVED. Two SpaceMice both report an EMPTY
    # serial number (measured 2026-08-10), so the trick that made the CAN adapters
    # unambiguous — select by serial, never by position — does not transfer. The device
    # identifies itself by being wiggled.
    #
    # ⛔ `exclude` IS THE PART THAT MATTERS WITH TWO ARMS, and without it this function can
    # hand the SAME puck to both: the single-device shortcut returns it unconditionally, and
    # nothing stops the operator moving the arm they already assigned. Both failures are
    # silent, and the symptom — two arms following arm hand — reads as a control bug rather
    # than a device-assignment bug (`src/spacemouse.py`, 6 tests).
    #
    # ⚠️ Opened BEFORE `build_robot()`, deliberately: if a puck is missing the session
    # returns here, with nothing energised.
    pucks: dict[str, Any] = {}
    for name in arm_names:
        info = pick_device_by_wiggle(label=name,
                                     exclude=[h["path"] for h in pucks.values()])
        if info is None:
            print(f"No SpaceMouse found for arm {name} (or none was moved).")
            for opened in pucks.values():
                # ⛔ Close what was already opened. Returning without this leaves a claimed
                # HID device behind, and the next run's wiggle then cannot see it.
                try:
                    opened["handle"].close()
                except Exception:  # noqa: BLE001, S110
                    pass
            return 1
        countdown_hands_off(3)
        handle = open_device(info)
        handle.set_nonblocking(True)
        pucks[name] = {"path": info["path"], "handle": handle,
                       "reader": TwistReader(handle)}

    # ⚠️ `robot = None` was declared here. Every robot handle now lives on its own
    # `ArmSession`, and the teardown iterates `arms`, which is empty when nothing was built.
    # ⛔⭐ DECLARED HERE, BEFORE THE `try`, AND IT IS None ON PURPOSE. FINDINGS §48.3.
    #
    # The closing summary at the bottom of this function reads a field off `arm`, and it
    # runs on the path where `build_robot()` FAILED — the `except Exception` prints the
    # error and falls through. That is the path Julien sees whenever the CAN adapters are
    # in DFU, which happens often. Without this declaration that line would raise
    # `UnboundLocalError` and replace a clear "No candleLight CAN adapter found" with a
    # traceback.
    #
    # ⚠️ So every read of `arm` outside the `try` MUST be guarded by `if arm is not None`.
    # `scripts/check_restructure.py` finds the construction point by locating the
    # `ArmSession(` call rather than this line, so it can still catch a genuine
    # use-before-build.
    arm: ArmSession | None = None
    # ⛔⭐ DECLARED HERE FOR THE SAME REASON `arm` IS, and the reason is now stronger than
    # it was for `arm`. The `finally` block and the closing summary both iterate `arms` —
    # to save each arm's axis map, and to report it — and both run on the path where
    # `build_robot()` FAILED. An unbound name there would replace *"No candleLight CAN
    # adapter found"* with a `NameError`, on the failure Julien hits most often.
    # ⭐ An empty list is better than a `None` guard: the loops simply do not run, so
    # there is no second code path to keep correct. FINDINGS §48.3.
    arms: list[ArmSession] = []
    # ⛔⭐⭐ `start_mode` IS A SEPARATE NAME FROM `arm.mode`, AND THAT IS THE WHOLE REASON
    # `mode` WAS THE LAST FIELD TO MOVE. `build_robot()` below is called with
    # `zero_gravity=(start_mode == "guide")`, and it runs BEFORE the robot exists — so
    # before the `ArmSession` that would hold the mode can exist either. The name with the
    # most references (48) was therefore the last arm that could move, which is the
    # opposite of the order anyone would choose for comfort. FINDINGS §50.
    #
    # ⚠️ It is deliberately NOT the same variable. Keeping arm `mode` and assigning it
    # twice would put the script and the object out of step for the lines in between,
    # which is the state neither models.
    start_mode = args.start_mode
    stop_reason: str | None = None
    # ⚠️ `teleop` was declared here as None. It is `ArmSession.teleop` now, and the class's
    # own constructor already sets it to None. ⛔ Leaving this line would run before `arm`
    # exists, which the ordering check in scripts/check_restructure.py catches.
    # ⚠️ The thermal guard used to be created here. It is `ArmSession`'s now, built by its
    # constructor from the same `warn_at=TEMP_WARN, stop_at=TEMP_STOP` this line passed.
    # ⛔ Leaving it here as `arm.thermal = …` would run before `arm` exists.
    #
    # ⭐ It stays an object rather than a pair of floats, and the reason is worth keeping:
    # "I cannot read the temperature" is a state that has to be tracked and acted on, and
    # it used to be indistinguishable from 0 °C. See `ThermalGuard`.
    # ⚠️ `hottest` and `jaw_temp` were declared here. They are `ArmSession` fields now, so
    # each arm reports its OWN temperatures on its own status row — as session locals they
    # were arm arm's reading painted on whichever row was being drawn.
    # ⛔ Not left here as `arm.hottest = None`: that would run before `arm` exists, which
    # is the ordering fault `scripts/check_restructure.py` check 3 catches.
    next_park_report = 0.0
    # ⚠️ `gripper_value` and `stall_since` used to be initialised here. They are now
    # `ArmSession` fields, and the class's own constructor sets exactly the same values
    # (0.0 and None). ⛔ Leaving the assignments here as `arm.gripper_value = 0.0` would
    # run BEFORE `arm` exists, which is nine lines below inside the `try`. See the
    # ordering check in scripts/check_restructure.py.

    try:
        n_motors = N_ARM if args.no_gripper else N_ARM + 1
        # ⭐⭐⭐ ONE ROBOT PER ARM, BUILT IN ORDER. ROADMAP §6.1 step 3.
        #
        # ⛔ `build_robot()` energises motors and is the single most dangerous call in the
        # project, so it stays here, visible, in the script — never inside `ArmSession`.
        # With two arms it happens twice, and the second arm starts while the first is
        # already holding its pose under power.
        #
        # ⚠️ If the SECOND build fails, the first arm is already energised. The `finally`
        # block disables every arm it finds in `arms`, and the arm is appended as soon as it
        # is constructed, so a half-built session still shuts the built half down properly.
        for name in arm_names:
            print(f"building arm {name} — enables {n_motors} motors, "
                  "starts the control loop …")
            robot, note = build_robot(name, zero_gravity=(start_mode == "guide"),
                                      with_gripper=not args.no_gripper,
                                      max_speed=args.max_speed)
            print(f"  {note}\n")

            arm = ArmSession(robot, name=name, frame=start_frame,
                             gripper_min=GRIPPER_MIN, gripper_max=GRIPPER_MAX,
                             warn_at=TEMP_WARN, stop_at=TEMP_STOP,
                             axis_map=map_store.for_arm(name, start_frame),
                             slots=saved_slots[name], base_slot=BASE_SLOT,
                             reader=pucks[name]["reader"])
            # ⛔⭐⭐ THIS LINE IS NOT OPTIONAL, AND ITS ABSENCE WOULD HAVE BEEN SILENT.
            #
            # `ArmSession.__init__` sets `self.mode = "hold"`, which is the right default for
            # a class that may be built before anyone has chosen a mode. **The script has
            # already chosen arm**, from `--start-mode`, and `build_robot()` above has
            # already acted on it by deciding `zero_gravity`.
            #
            # ⛔ Without this assignment, `--start-mode guide` would build a WEIGHTLESS robot
            # and then run the loop believing it was in HOLD. **Nothing would raise.** The arm
            # would hang from gravity compensation alone while the screen said HOLD, which is
            # the defect class FINDINGS §0 exists for. Found by asking what the class's own
            # default is, not by anything failing. FINDINGS §50.2.
            arm.mode = start_mode
            arm.prev_q = np.asarray(arm.robot.get_joint_pos(), dtype=float)[:N_ARM]
            arms.append(arm)

            # ⭐ DEFAULT PARK POSE = WHEREVER THIS ARM STARTED. Julien: *"if the standard set
            # position for park mode is just the starting position, then I can always just
            # press p and then d, and I don't have to do anything with my hands."*
            #
            # ⭐ Per arm, which removes a real dependency he flagged: the two arms do NOT have
            # to be physically placed the same way before a session, because each arm parks
            # back to its own measured start rather than to a pose recorded from the other.
            #
            # ⚠️ It is only as good as the pose you start in. Start with the arm drooped and
            # PARK will faithfully return it to drooped, which is why the plan prints the
            # actual numbers rather than just saying "default".
            if arm.base_pose is None:
                arm.base_pose = np.asarray(arm.robot.get_joint_pos(), dtype=float).tolist()
                print(f"  park pose {name} : none saved — defaulting to the pose the arm is "
                      f"in NOW, {np.round(np.asarray(arm.base_pose)[:N_ARM], 3).tolist()}")
                print("                (press s to set a different arm; q then p then d "
                      "parks and quits)")

        # ⛔ Mode keys are AIMED; driving never is. A global `g` would put 8.6 kg weightless
        # in arm keypress, and GUIDE is where a dynamics-model error becomes a falling arm
        # rather than a droop (FINDINGS §11.1). Each arm always follows its own puck.
        # `src/arm_session.py::ArmSelector` holds the cycle and its tests.
        selection = ArmSelector(arm_names)

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

        # ⭐⭐ EVERY FUNCTION BELOW TAKES THE ARM IT ACTS ON. Step 2 plumbing, 2026-08-14.
        #
        # They were closures over the single `robot` and the single `arm`, which is the last
        # structural reason two arms could not run: a closure cannot be pointed at a second
        # arm. Passing the arm changes nothing at N=1 — the same object is handed in — and
        # it is what lets the mode dispatch loop over `selection.names()`.
        #
        # ⚠️ `clamp_gripper` above takes NO arm on purpose: the band is a property of the
        # gripper hardware, identical on both arms, and it is passed to `park_target_from`
        # as a one-argument callable.
        #
        # ⛔ These are still the script's own copies rather than `ArmSession`'s methods, and
        # that is a recorded decision, not an oversight: FINDINGS §52.1. Collapsing them is
        # ROADMAP §8.2 item 23, it changes what the arm is commanded at the margins, and it
        # therefore needs its own bench pass.
        def resync(one: ArmSession) -> None:
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
            one.prev_q = np.asarray(one.robot.get_joint_pos(), dtype=float)[:N_ARM]
            if hasattr(one.robot, "resync"):
                one.robot.resync()

        def enter_teleop(one: ArmSession) -> None:
            resync(one)
            q = np.asarray(one.robot.get_joint_pos(), dtype=float)
            one.robot.command_joint_pos(q)       # leaves zero-gravity mode
            # Take the jaws exactly where they are. Do NOT clamp here: clamping on
            # entry is a command to move, and nobody asked for that.
            one.gripper_value = float(q[N_ARM]) if len(q) > N_ARM else 0.5
            # ⭐ The frame is this ARM's now, not the session's, so two arms can be driven
            # in different frames — one in `world` while the other follows its own wrist in
            # `tool`. The axis map store was already per-arm-per-frame, so nothing new had
            # to be invented for it (`--fork-map`).
            one.teleop = CartesianTeleop(frame=one.frame)
            one.teleop.reset(q[:N_ARM])
            one.home_ee = one.teleop.ee_position().copy()

        def enter_hold(one: ArmSession) -> None:
            resync(one)
            one.robot.command_joint_pos(
                np.asarray(one.robot.get_joint_pos(), dtype=float))

        def enter_guide(one: ArmSession) -> None:
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
            resync(one)
            one.guide_ref = np.asarray(one.robot.get_joint_pos(), dtype=float)
            fn = getattr(one.robot, "enter_gravity_comp_idle", None)
            if callable(fn):
                fn()
                return
            print("  ⚠️  enter_gravity_comp_idle() missing — staying in HOLD (NOT weightless)")

        def sample_layout() -> Layout:
            """How a recording's flat sample maps onto this session's arms, right now.

            ⛔⭐ A FUNCTION RATHER THAN A VARIABLE, AND THAT IS THE POINT. The first draft
            assigned `take_layout` in the per-cycle sampler (section 3.4) and read it in the
            `w` key handler (section 3) — **which runs earlier in the same cycle.** Pressing
            `w` on the very first cycle of a session would have raised `NameError` inside the
            control loop with the motors live, and no test or dry run reaches that line. It is
            the same ordering fault `check_restructure.py` check 3 exists for, one variable
            over.

            ⚠️ Recomputed on each call, which costs one `num_dofs()` per use and cannot go
            stale. `--no-gripper` changes the answer, so caching it at startup would be wrong
            as well as fragile.
            """
            return Layout(tuple(one.name for one in arms), arms[0].robot.num_dofs())

        def park_plan_line(one: ArmSession) -> str:
            """The one line showing what a run will do and how it will feel.

            ⭐ Printed while typing the sequence, on every knob change, and again at the
            confirm step — so speed and corner style are never something discovered
            only after the arm is already moving.

            ⚠️ The speed and the ease ramp are **per arm** (they live on `ArmSession`),
            so with two arms selected this line describes ONE of them. The rule that
            resolves it follows ROADMAP §6: the knob keys aim at the selection, exactly
            like the mode keys. Not implemented while two arms cannot run.
            """
            seq = " → ".join(park_sequence) if park_sequence else "0"
            name, radius = BLEND_MODES[blend_idx]
            # ⭐ ONE line, so changing a knob repaints instead of appending. Six taps
            # on `+` should leave one line showing the final speed, not six blocks.
            return (f"RUN {seq} · speed {one.park_speed:.2f} (-/+) · corners {name} "
                    f"{radius:.2f} (,/.) · ease {EASINGS[ease_idx].name} over "
                    f"{one.park_ramp:.2f} (e, ö/ä) · Enter=go")

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
            if taught > args.teleop_speed:
                note = (f" ⚠️ taught {taught:.1f} rad/s exceeds the "
                        f"{args.teleop_speed:.1f} allowed, so 1.00x will lag")
            return (f"PLAY {replay_slot} · {replay_pending.duration:.1f}s taught at "
                    f"{taught:.2f} rad/s · speed {replay_speed:.2f}x (-/+){note} · Enter=go")

        def begin_path(one: ArmSession, legs: list, what: str) -> None:
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
            targets = []
            for _, pose in legs:
                tgt, warn = park_target_from(one.robot.get_joint_pos(), pose,
                                             gripper_index=N_ARM, clamp=clamp_gripper)
                if warn:
                    print(f"\n  ⚠️  {warn}.")
                targets.append(tgt)
            start = np.asarray(one.robot.get_joint_pos(), dtype=float)
            one.park_path = JointPath([start, *targets], blend=BLEND_MODES[blend_idx][1])
            one.park_marks = list(zip([n for n, _ in legs], one.park_path.arrival_lengths()[1:]))
            one.park_s = 0.0
            one.park_target = targets[-1]
            one.park_cmd = start.copy()
            # ⛔ THE ORDER MATTERS AND IT IS NOT THE OBVIOUS ONE. `enter_hold()` here is the
            # script's own, which commands the measured pose and does NOT touch the mode —
            # so setting `park` first is safe. `ArmSession.enter_hold()` DOES set the mode,
            # and swapping it in without moving this line would leave a park running while
            # the screen said HOLD. FINDINGS §52.1 keeps this trap next to item 23.
            one.mode = "park"
            enter_hold(one)
            one.park_best_err = float(np.max(np.abs(one.park_target - start)))
            one.park_progress_t = t
            one.park_leg_t = t
            one.park_start_t = t
            # The plan has become the thing happening; the progress readout replaces it.
            hint("")
            print(f"\n⭐ MODE: PARK → {what}, {one.park_path.length:.2f} rad of travel at "
                  f"{one.park_speed:.2f} rad/s, corners {BLEND_MODES[blend_idx][0]}. "
                  "Press h or t to stop.\n")

        # ⭐ Each arm enters its start mode, per arm. It used to run once, after the single
        # build, reading the one `robot` local.
        for one in arms:
            if one.mode == "teleop":
                enter_teleop(one)
            elif one.mode == "hold":
                enter_hold(one)
            elif one.mode == "guide":
                # ⚠️ GUIDE at startup is established by build_robot(zero_gravity=True), not
                # by enter_guide() — so the drift reference has to be taken here too, or the
                # readout silently shows nothing for the whole first GUIDE period. That gap
                # is exactly the 33 seconds in which the arm sank unremarked on 2026-08-10.
                one.guide_ref = np.asarray(one.robot.get_joint_pos(), dtype=float)

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
            print("⭐ MODE: "
                  + " · ".join(f"{one.name} {one.mode.upper()}" for one in arms) + "\n")

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

                # ---- 1. is every robot still there? -----------------------
                # ⛔⭐ A FAULT ON ONE ARM STOPS ALL OF THEM. ROADMAP §6's ruling, and the
                # reason is physical: a chain death on B must not leave G uncommanded and
                # sagging while the operator is still looking at B.
                #
                # ⛔⭐⭐ NOTE THE SHAPE, BECAUSE `break` CHANGED MEANING HERE. This used to
                # be `if not chain_alive(robot): stop_reason = …; break`, straight out of the
                # `while`. Inside a `for one in arms:` a `break` leaves only the FOR, so the
                # cycle would carry on commanding arms with a stop already decided. The stop
                # is recorded in the loop and acted on after it.
                for one in arms:
                    if not one.alive():
                        stop_reason = (
                            f"arm {one.name}: the motor chain STOPPED — I2RT's control "
                            "thread exited, almost certainly on a motor fault. Commands "
                            "are no longer reaching the arm."
                        )
                if stop_reason:
                    break

                # ---- 2. temperatures and the gripper stall guard -----------
                # ⛔⭐ ONLY THE READ IS WRAPPED. The decisions are not, and that is the
                # entire point of this shape. The previous version wrapped the read AND
                # every check that followed in one `try`, whose handler set
                # `hottest = 0.0` — so a failed read silently disarmed the thermal stop
                # and printed a calm "hottest 0°C". A guard with a path around it is
                # the defect this repo keeps paying for (working contract rule 7);
                # here the path was its own exception handler. See ThermalGuard.
                #
                # ⚠️ Per arm, and each arm keeps its OWN last reading, because the incident
                # record wants the last good values from a chain that may now be dead.
                for one in arms:
                    try:
                        one.states = one.robot.motor_chain.read_states()
                        read_error = None
                    except Exception as exc:  # noqa: BLE001
                        one.states, read_error = None, f"{type(exc).__name__}: {exc}"

                    if one.states is None:
                        one.hottest, one.jaw_temp = None, None
                        one.stall_since = None            # cannot judge a stall we cannot see
                        verdict = one.thermal.update(None)
                    else:
                        one.temps, one.hottest, one.jaw_temp = motor_temperatures(
                            one.states, N_ARM)
                        verdict = one.thermal.update(
                            one.hottest, one.jaw_temp,
                            motor=one.temps.index(one.hottest)
                            if one.hottest is not None else None)
                        # ---- gripper stall guard ------------------------------
                        # ⚠️ With --no-gripper the chain has 6 motors, so states[6] would
                        # IndexError. It used to be guarded by raising StopIteration out of
                        # the shared try — which worked, but meant the "no gripper" path and
                        # the "read failed" path were the same code path. Now it is just an
                        # if, because there is nothing left to jump out of.
                        jaw = one.states[N_ARM] if len(one.states) > N_ARM else None
                        if jaw is None:
                            one.stall_since = None
                        elif (abs(getattr(jaw, "eff", 0.0)) > GRIPPER_STALL_TORQUE
                                and abs(getattr(jaw, "vel", 0.0)) < GRIPPER_STALL_VEL):
                            if one.stall_since is None:
                                one.stall_since = loop_start
                            elif loop_start - one.stall_since > GRIPPER_STALL_SECONDS:
                                measured_jaw = float(np.asarray(
                                    one.robot.get_joint_pos(), dtype=float)[N_ARM])
                                print(f"\n⚠️  ARM {one.name} GRIPPER STALLED "
                                      f"({jaw.eff:+.2f} Nm, not moving) — releasing it to "
                                      f"{measured_jaw:.3f} so it stops pushing.\n")
                                one.gripper_value = measured_jaw
                                one.stall_since = None
                        else:
                            one.stall_since = None

                    if verdict.warning:
                        detail = f"  ({read_error})" if read_error else ""
                        print(f"\n⚠️  arm {one.name}: {verdict.warning}{detail}\n")
                    # ⛔ Recorded, not `break`ed — see the note on the liveness loop above.
                    # ⚠️ A thermal stop on ONE arm stops the session, same ruling as a chain
                    # death: the alternative is one arm cooking while the other is driven.
                    if verdict.stop_reason:
                        stop_reason = f"arm {one.name}: {verdict.stop_reason}"
                if stop_reason:
                    break

                # ---- 3. keys ----------------------------------------------
                for k in keys.drain():
                    # ⭐⭐ WHICH ARMS THIS KEYPRESS IS AIMED AT. ROADMAP §6's split, and the
                    # three names below are the whole of it:
                    #
                    #   `aimed`    — every selected arm. Mode changes, poses, the gripper and
                    #                the park knobs act on all of them.
                    #   `edit_arm` — the FIRST selected arm, and the only one a MAP edit ever
                    #                touches. ⛔ Not a shortcut: `AxisMapStore.for_arm()`
                    #                returns the SAME `AxisMap` object to both arms while the
                    #                scope is SHARED (the default), so applying an edit to
                    #                each selected arm would flip a motion TWICE, back to
                    #                where it started, printing two confirmations
                    #                (FINDINGS §53.5). A map edit also comes from one physical
                    #                gesture on one puck, so one arm is the honest target.
                    #   `wizard`   — the arm inside CONTROLS, if any. At most one can be:
                    #                `m` refuses when two arms are selected, and `a` refuses
                    #                while CONTROLS is open.
                    #
                    # ⚠️ Recomputed every keypress, because `a` changes the selection inside
                    # this very loop.
                    aimed = [one for one in arms if one.name in selection.names()]
                    edit_arm = aimed[0]
                    wizard = next((one for one in arms if one.mode == "map"), None)
                    aimed_label = "+".join(one.name for one in aimed)
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
                            # ⭐ EVERY SELECTED ARM SAVES ITS OWN POSE into that slot. The file
                            # is keyed by arm, so `s 1` with BOTH selected records two
                            # different poses under one digit, which is what a two-arm
                            # waypoint is.
                            # ⚠️ `data` is threaded through the loop and written ONCE, because
                            # `with_park_slot` returns a new dict rather than mutating. Saving
                            # inside the loop would write the first arm's version and then
                            # overwrite it with a copy that never saw it.
                            name = BASE_SLOT if k == "0" else k
                            data = load_json(PARK_FILE, {})
                            for one in aimed:
                                q = np.asarray(one.robot.get_joint_pos(), dtype=float)
                                data = with_park_slot(data, one.name, name, q.tolist())
                                if k == "0":
                                    one.base_pose = q.tolist()
                                    print(f"\n  ⭐ arm {one.name} BASE pose (0) saved — this is "
                                          f"where Ctrl-C parks before disabling:"
                                          f"\n     {np.round(q[:N_ARM], 3)}\n")
                                else:
                                    print(f"\n  ✓ arm {one.name} waypoint {k} saved: "
                                          f"{np.round(q[:N_ARM], 3)}"
                                          f"     (p {k} drives back to it; Ctrl-C ignores it)\n")
                            save_json(PARK_FILE, data)
                            for one in aimed:
                                one.slots = park_slots(data, one.name)
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
                        loaded = Trajectory.load(path)
                        # ⭐ THE LAYOUT COMES FROM THE FILE. `Layout.from_meta` also reads
                        # recordings made before two arms existed — Julien has several in
                        # slots 1-6 — so they stay playable on the arm they name.
                        layout = Layout.from_meta(loaded.meta, loaded.n_joints)
                        by_name = {one.name: one for one in arms}
                        missing = [n for n in layout.arms if n not in by_name]
                        if missing:
                            print(f"\n  ⚠️  recording {k} was made with arm(s) "
                                  f"{', '.join(layout.arms)} and this session has "
                                  f"{', '.join(by_name)}. Missing: {', '.join(missing)}.")
                            print("     Start the session with those arms to play it.\n")
                            continue
                        if layout.n_joints != loaded.n_joints:
                            print(f"\n  ⚠️  recording {k} says {layout.n_joints} joints in its "
                                  f"metadata and holds {loaded.n_joints}. Refusing rather "
                                  "than guessing which is right.\n")
                            continue
                        replay_pending = loaded
                        replay_layout = layout
                        replay_arms = [by_name[n] for n in layout.arms]
                        replay_ready = set()
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
                                                    args.teleop_speed)
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
                                replay_pending.joint_speed(99), args.teleop_speed))
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
                            # ⭐⭐ EVERY ARM THE RECORDING DRIVES PARKS TO ITS OWN SLICE of
                            # the start pose. This is the safety point of the whole feature:
                            # playback commands poses a hand physically put the arm in, and
                            # the only dangerous command is the FIRST one, which would jump
                            # from wherever the arm is now.
                            start = list(replay_pending.start_pose() or ())
                            for one in replay_arms:
                                begin_path(one, [("recording start",
                                                  start[replay_layout.slice_for(one.name)])],
                                           f"arm {one.name}'s start pose in recording "
                                           f"{replay_slot}")
                            print("     then it plays the recording. Press h or t to stop.\n")
                        else:
                            replay_pending = None
                            hint("")
                            print("\n  play cancelled.\n")
                        continue

                    if pending == "mirror_go":
                        # ⭐ `i` AT THE PROMPT SWITCHES copy ↔ mirror and re-prints the plan.
                        # Without it, discovering that `copy` is the wrong choice for how the
                        # arms are standing means quitting the session and restarting with
                        # `--mirror mirror`, which costs a puck assignment and two builds.
                        # ⚠️ The plan line says what `i` does here, so this is not one key
                        # with two hidden meanings — it is the mirror key, inside the mirror
                        # prompt, changing the mirror.
                        if k == "i":
                            args.mirror = "mirror" if args.mirror == "copy" else "copy"
                            print(f"     ⭐ now {args.mirror.upper()}: "
                                  + ("the follower reproduces the leader's angles unchanged, "
                                     "for arms side by side" if args.mirror == "copy" else
                                     "the follower negates the joints that reverse under "
                                     "reflection, for arms FACING each other")
                                  + "\n     Enter engages · i switches again · any other "
                                    "key cancels\n")
                            continue
                        pending = None
                        if k in ("\r", "\n", " ") and mirror_follower is not None:
                            # ⛔ The follower goes under POSITION control before anything is
                            # commanded. If it were left weightless the commands would do
                            # nothing at all, and the readout would show it tracking.
                            mirror_follower.mode = "mirror"
                            enter_hold(mirror_follower)
                            # ⭐ THE FOLLOW SPEED IS READ FROM THE FOLLOWER'S OWN CAP, not
                            # repeated here. `MirrorLink`'s default is 1.0 because that is
                            # SafeRobot's default; if `--max-speed` raises the cap, a
                            # hardcoded 1.0 here would quietly become the binding limit and
                            # the mirror would stay slow for no visible reason.
                            mirror_link = MirrorLink(
                                mode=args.mirror, align_speed=MIRROR_ALIGN_SPEED,
                                follow_speed=getattr(mirror_follower.robot, "max_speed",
                                                     args.max_speed),
                                max_gap=args.mirror_gap)
                            # ⭐ The baseline for "how often did SafeRobot hold the command
                            # back during THIS link", which is the hardware-side half of the
                            # diagnosis. A running total since the session began would say
                            # nothing about the mirror run.
                            mirror_clipped_at = getattr(mirror_follower.robot,
                                                        "limited_cycles", 0)
                            print(f"\n▶  MIRROR engaged: arm {mirror_follower.name} is "
                                  f"following arm {mirror_leader.name}. "
                                  "Press h, t, g or i to stop it.\n")
                        else:
                            mirror_leader = mirror_follower = None
                            hint("")
                            print("\n  mirror cancelled.\n")
                        continue

                    if pending in ("park", "confirm"):
                        # ⭐ SPEED AND CORNERS ADJUSTABLE WHILE TYPING, not only while
                        # moving. Julien: *"I can change the park speeds whilst it's
                        # parking, but not whilst I'm putting in the numbers, which is
                        # a bit annoying."* Deciding how a move should feel belongs to
                        # the moment you are choosing the move.
                        if k in "+=":
                            for one in aimed:
                                one.park_speed = min(args.teleop_speed,
                                                     one.park_speed * 1.25)
                            hint(park_plan_line(edit_arm)); continue
                        if k == "-":
                            for one in aimed:
                                one.park_speed = max(0.05, one.park_speed / 1.25)
                            hint(park_plan_line(edit_arm)); continue
                        if k == ".":
                            blend_idx = min(len(BLEND_MODES) - 1, blend_idx + 1)
                            hint(park_plan_line(edit_arm)); continue
                        if k == ",":
                            blend_idx = max(0, blend_idx - 1)
                            hint(park_plan_line(edit_arm)); continue
                        if k in KEY_STEP_UP:
                            # ⭐ How LONG the ease lasts, separately from its shape.
                            # Julien: *"the smoothing should maybe be adjustable at the
                            # beginning of the park, similar to the parking speed."*
                            # ö/ä (or [/]) mean gripper step elsewhere, which is
                            # meaningless while choosing a park — same
                            # context-dependence as +/-.
                            for one in aimed:
                                one.park_ramp = min(1.0, one.park_ramp * 1.4)
                            hint(park_plan_line(edit_arm)); continue
                        if k in KEY_STEP_DOWN:
                            for one in aimed:
                                one.park_ramp = max(
                                    0.0, one.park_ramp / 1.4 if one.park_ramp > 0.03 else 0.0)
                            hint(park_plan_line(edit_arm)); continue
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
                            hint(park_plan_line(edit_arm)); continue

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
                                hint(park_plan_line(edit_arm))
                                continue
                            pending = None
                            wanted = park_sequence[:] or ["0"]
                            park_sequence.clear()
                            # ⭐ EACH SELECTED ARM RUNS ITS OWN SEQUENCE, resolved against
                            # its own slots. Two arms driving to their own saved poses at the
                            # same time is what a two-arm waypoint run means.
                            # ⚠️ A slot empty on ONE arm is skipped for that arm only, never
                            # substituted, and never cancels the other arm's run.
                            ran = False
                            for one in aimed:
                                legs, missing = resolve_park_legs(wanted, one.base_pose,
                                                                  one.slots)
                                if missing:
                                    print(f"\n  ⚠️  arm {one.name}: nothing saved in slot "
                                          f"{', '.join(missing)} — press s then that digit "
                                          "to record one.\n")
                                if legs:
                                    begin_path(one, legs, f"slot {legs[0][0]}")
                                    ran = True
                            if not ran:
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
                            ran = False
                            for one in aimed:
                                legs, missing = resolve_park_legs(wanted, one.base_pose,
                                                                  one.slots)
                                if missing:
                                    print(f"\n  ⚠️  arm {one.name}: skipping empty slot(s) "
                                          f"{', '.join(missing)}.\n")
                                if legs:
                                    begin_path(one, legs, " → ".join(n for n, _ in legs))
                                    ran = True
                            if not ran:
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
                        edit_arm.learn_button = "open"
                        print(f"\n⭐ LEARNING THE GRIPPER BUTTONS for arm {edit_arm.name}.")
                        print("   Press the puck button you want for OPEN …")
                        print("   (learned by pressing, never assumed — which physical button")
                        print("    sets which HID bit has never been measured on this unit)\n")
                        continue
                    if k == "a":
                        # ⭐⭐ WHICH ARM THE MODE KEYS AIM AT — ROADMAP §6's decision, and
                        # the reason is in `ArmSelector`: `g` on two arms at once is 8.6 kg
                        # going weightless on one keypress.
                        #
                        # ⚠️ Handled HERE, above the mode dispatch, for the same reason `b`
                        # and `v` are: which arm a key applies to is a property of the
                        # SESSION, not of a mode. A selector that worked only in TELEOP
                        # would be the `b` defect again (FINDINGS §17.1).
                        if any(other.mode == "map" for other in arms):
                            # ⛔ CONTROLS is a wizard that belongs to the arm it was
                            # entered on: it asks the operator to push one axis at a time
                            # and edits that arm's map from the answers. Re-aiming the
                            # keys underneath it would write one arm's answers into
                            # another arm's map, which is the blast-radius bug the
                            # per-arm map store exists to prevent.
                            hint("leave CONTROLS (m) before changing which arm is selected")
                        elif selection.only_one():
                            hint(f"arm {selection.label} is the only arm in this session "
                                 f"— two arms is ROADMAP §6.1 step 3")
                        else:
                            print(f"\n⭐ SELECTED: {selection.cycle()} — mode keys apply to "
                                  "it. Driving always applies to every arm.\n")
                        continue
                    if k == "i":
                        # ⭐⭐ MIRROR MODE. The selected arm leads; the other follows.
                        #
                        # ⛔ IT ASKS TWICE, exactly like `l`. Engaging starts a MOTION on the
                        # follower — it ramps to the leader's pose — and the operator's hands
                        # and eyes are on the leader at that moment. A single keypress that
                        # moves an arm nobody is looking at is the one thing this session's
                        # design refuses.
                        if mirror_link is not None:
                            mirror_link = None
                            for one in arms:
                                if one.mode == "mirror":
                                    one.mode = "hold"; enter_hold(one)
                            hint("")
                            print("\n  ⭐ MIRROR off — the follower is HOLDING.\n")
                            continue
                        try:
                            lead_name, follow_name = pick_pair(
                                [one.name for one in arms], selection.names())
                        except ValueError as exc:
                            hint(str(exc))
                            continue
                        mirror_leader = next(o for o in arms if o.name == lead_name)
                        mirror_follower = next(o for o in arms if o.name == follow_name)
                        pending = "mirror_go"
                        start_gap = float(np.max(np.abs(
                            np.asarray(mirror_follower.robot.get_joint_pos(), dtype=float)[:N_ARM]
                            - np.asarray(mirror_leader.robot.get_joint_pos(), dtype=float)[:N_ARM])))
                        print(f"\n⭐ MIRROR: arm {lead_name} LEADS, arm {follow_name} FOLLOWS "
                              f"({args.mirror}).")
                        print(f"     arm {follow_name} will first close a {start_gap:.2f} rad "
                              f"gap at {MIRROR_ALIGN_SPEED} rad/s, then track continuously.")
                        print(f"     ⚠️ HOLD ARM {lead_name} STILL until it says FOLLOWING, and "
                              f"keep the space around arm {follow_name} clear.")
                        # ⛔ SAID OUT LOUD BECAUSE NOTHING CHECKS IT. There is no collision
                        # model anywhere in this project: no arm knows where the other one is.
                        # MIRROR is the first mode where an arm moves with no hand on it, so
                        # the operator is the only thing standing between two arms reaching
                        # into the same space. ROADMAP §8.2 item 25.
                        print("     ⛔ NOTHING CHECKS FOR THE ARMS COLLIDING. No arm knows "
                              "where the other one is.")
                        print("     Enter engages · i switches copy/mirror · any other key "
                              "cancels\n")
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
                        # ⭐ ONE arm's frame, like every other map-scoped edit (§53.5). Two
                        # arms may sit in different frames, which is the point: one driven in
                        # `world` while the other follows its own wrist in `tool`.
                        map_store.set(edit_arm.name, edit_arm.axis_map, edit_arm.frame)
                        edit_arm.frame = order[
                            (order.index(edit_arm.frame) + 1) % len(order)]
                        edit_arm.axis_map = map_store.for_arm(edit_arm.name, edit_arm.frame)
                        if edit_arm.teleop is not None:
                            edit_arm.teleop.frame = edit_arm.frame
                        # ⛔⭐ THERE IS NOW ONE COPY OF THE FRAME, AND THAT IS THE REAL FIX
                        # FOR WHAT FINDINGS §52.7 CAUGHT. `ArmSession.frame` used to sit
                        # beside a session-level `control_frame` local, and only the local
                        # was updated here — so the object silently disagreed with the
                        # session after the first `v`. It was patched by assigning both.
                        # ⭐ Deleting the local is better than keeping them in step: a
                        # second copy that has to be maintained will eventually not be.
                        print(f"\n  ⭐ arm {edit_arm.name} CONTROL FRAME → "
                              f"{CartesianTeleop.FRAME_NOTES[edit_arm.frame]}")
                        print("     controls for this frame: "
                              f"{edit_arm.axis_map.one_line(edit_arm.frame)}")
                        print("     press m to edit THESE controls; each frame has its own\n")
                        continue
                    if k == "f" and edit_arm.last_input_kind == "button":
                        # Same key, same meaning everywhere: reverse the control just
                        # used. Axis -> flip its sign; button -> swap open/close.
                        edit_arm.axis_map.swap_buttons()
                        print("\n  ↔ SWAPPED the gripper buttons")
                        print(edit_arm.axis_map.buttons_row() + "\n")
                        continue

                    # ---- MAP mode owns the keyboard while it is active --------
                    # ⚠️ 1-6 mean "select a motion" here and "flip a rotation sign" in
                    # the drive modes. Overloading is a real footgun in a codebase
                    # whose motto is that this stack fails by lying, so it is bounded:
                    # MAP mode is entered explicitly, announces itself loudly, holds
                    # the arm still, and echoes the effect of every key. Nothing it
                    # can do moves a motor.
                    if wizard is not None:
                        # ⛔ EVERY EDIT IN THIS BRANCH IS KEY-DRIVEN. Moving the puck
                        # must never change the map — see FINDINGS §11 for what
                        # happened when it did.
                        active = wizard.last_active_axis
                        driven = wizard.axis_map.motion_driven_by(active) if active is not None else None
                        if k == "q":
                            stop_reason = "quit requested"
                        elif k in "tghm":
                            print("\n  controls now:")
                            print(wizard.axis_map.describe(wizard.frame))
                            if k == "t":
                                wizard.mode = "teleop"; enter_teleop(wizard)
                                print("\n⭐ MODE: TELEOP — SpaceMouse drives, all axes\n")
                            elif k == "g":
                                wizard.mode = "guide"; enter_guide(wizard)
                                print("\n⭐ MODE: GUIDE — arm is weightless\n")
                            else:
                                wizard.mode = "hold"; enter_hold(wizard)
                                print("\n⭐ MODE: HOLD\n")
                        elif k == "f":
                            if active is None:
                                print("\n  push the puck first — f reverses the control you just used.\n")
                            elif driven is None:
                                print(f"\n  puck {PUCK_AXES[active]} drives nothing, so there is no "
                                      f"direction to reverse. Press 1-6 to give it a motion.\n")
                            else:
                                wizard.axis_map.flip(driven)
                                print(f"\n  ↔ REVERSED → {wizard.axis_map.row(driven, wizard.frame).strip()}"
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
                                    wizard.axis_map.swap(driven, target)
                                    print(f"\n  ⇄ SWAPPED {motions_for(wizard.frame)[driven]['short']} ↔ "
                                          f"{motions_for(wizard.frame)[target]['short']}")
                                    print(f"      {wizard.axis_map.row(target, wizard.frame).strip()}")
                                    print(f"      {wizard.axis_map.row(driven, wizard.frame).strip()}")
                                    print("      (press the same key again to swap back)\n")
                                else:
                                    # The active control drove nothing, so there is nothing
                                    # to exchange with. The direction he was last pushing
                                    # becomes this motion's positive sense.
                                    displaced = wizard.axis_map.bind(target, active, wizard.last_active_value)
                                    print(f"\n  ✓ puck {PUCK_AXES[active]} now drives "
                                          f"{motions_for(wizard.frame)[target]['short']} → "
                                          f"{wizard.axis_map.row(target, wizard.frame).strip()}")
                                    if displaced is not None:
                                        print(f"  ⚠️  {motions_for(wizard.frame)[displaced]['short']} was using that "
                                              f"control and is now UNBOUND — it will not move.")
                                    print()
                        elif k == "u":
                            if driven is None:
                                print("\n  that control already drives nothing.\n")
                            else:
                                wizard.axis_map.unbind(driven)
                                print(f"\n  unbound {motions_for(wizard.frame)[driven]['short']} — it will not move\n")
                        elif k == "0":
                            wizard.axis_map = wizard.axis_map_at_start.copy()
                            print("\n  reverted to the controls this session started with:")
                            print(wizard.axis_map.describe(wizard.frame) + "\n")
                        elif k == "?":
                            print(map_reference(wizard.frame))
                            print(MAP_HELP)
                            print(wizard.axis_map.describe(wizard.frame) + "\n")
                        # ⚠️ The rotation pair was MISSING here while the linear pair was
                        # present, so in CONTROLS mode roll/pitch/yaw could not be sped up
                        # or slowed down at all — Julien found it on the wizard. The keys were
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
                    elif k == "m" and len(aimed) > 1:
                        # ⛔ CONTROLS EDITS ONE MAP FROM ONE WIGGLE, so it cannot be aimed at
                        # two arms. Refused rather than silently applied to the first: the
                        # operator who selected BOTH and pressed `m` asked for something this
                        # wizard has no meaning for.
                        hint(f"CONTROLS is one arm at a time — press a to pick one "
                             f"(selected: {aimed_label})")
                    elif k == "m" and edit_arm.mode != "map":
                        # ⭐ CONTROLS mode DRIVES the arm — that is the whole point, and it
                        # is why this calls enter_teleop() rather than enter_hold(). The
                        # previous version held the arm still, which made it useless for
                        # the actual task: you cannot decide that a direction is wrong
                        # until you have watched the arm go that way. Julien:
                        # *"the actual mapping has to happen while the arm is moving so I
                        # can see what the different directions are doing."*
                        edit_arm.mode = "map"; enter_teleop(edit_arm)
                        edit_arm.last_active_axis = None
                        print(f"\n⭐ MODE: CONTROLS on arm {edit_arm.name} — the arm MOVES, "
                              "one isolated axis, half speed.\n")
                        print(map_reference(edit_arm.frame))
                        print(MAP_HELP)
                        print(edit_arm.axis_map.explain(edit_arm.frame))
                        print("\n  Push the puck one way at a time and watch the arm. If a direction is")
                        print("  wrong, press f. If a control should do something else, press 1-6.\n")
                        if not rotation:
                            print("  ⚠️  wrist rotation is OFF (r toggles) — ROLL/PITCH/YAW will not move.\n")
                    # ⭐⭐ MODE KEYS APPLY TO EVERY SELECTED ARM, which is what `a` is for.
                    # ⛔ `g` on two arms is 8.6 kg going weightless in one keypress, and GUIDE
                    # is the mode where an error in the dynamics model becomes a FALLING arm
                    # rather than a droop (FINDINGS §11.1). That is why the selector exists at
                    # all, and why it starts on one arm rather than on BOTH.
                    elif k == "g" and any(one.mode != "guide" for one in aimed):
                        hint("")
                        for one in aimed:
                            if one.mode != "guide":
                                one.mode = "guide"; enter_guide(one)
                        print(f"\n⭐ MODE: GUIDE on {aimed_label} — weightless, "
                              "you are holding it now\n")
                    elif k == "t" and any(one.mode != "teleop" for one in aimed):
                        hint("")
                        for one in aimed:
                            if one.mode != "teleop":
                                one.mode = "teleop"; enter_teleop(one)
                        print(f"\n⭐ MODE: TELEOP on {aimed_label} — each arm follows its "
                              "own SpaceMouse\n")
                    elif k == "h" and any(one.mode != "hold" for one in aimed):
                        hint("")
                        for one in aimed:
                            if one.mode != "hold":
                                one.mode = "hold"; enter_hold(one)
                        print(f"\n⭐ MODE: HOLD on {aimed_label}\n")
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
                    # ⚠️ `w` and `l` REFUSED with two arms until 2026-08-14 night, because
                    # `Trajectory` held one arm's joints and a two-arm demonstration would have
                    # been saved as half of itself. The recorder now samples every arm into one
                    # timeline, which is ABC's own shape (ROADMAP §9.2), so the refusal is gone
                    # along with the test that pinned it.
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
                                # ⚠️ ONE arm's name today. ABC's format is two arms in one
                                # timeline (ROADMAP §9.2), so when the recorder spans N arms
                                # this becomes the list of names in the same order as the
                                # samples. Changing it now would write a shape nothing reads.
                                # ⭐ `arm` is kept as well as `arms` so that anything already
                                # reading the old field still finds one, and `arms` is what
                                # playback uses. `Layout.from_meta` reads either.
                                "arm": arms[0].name,
                                **sample_layout().to_meta(),
                                "method": "live:" + "+".join(
                                    f"{one.name}:{one.mode}" for one in arms),
                                "nominal_hz": CONTROL_HZ,
                                "frame": arms[0].frame,
                            })
                            take_t0 = t
                            take_modes = [f"{one.name}:{one.mode}" for one in arms]
                            print("\n⏺  RECORDING " + " · ".join(
                                f"{one.name} {one.mode.upper()}" for one in arms)
                                + f"  ({sample_layout().n_joints} joints per sample). "
                                "Press w again to stop.\n")
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
                        saved_now = "; ".join(
                            f"{one.name}: " + (", ".join(sorted(n for n in one.slots
                                                                if n != BASE_SLOT)) or "none")
                            for one in aimed)
                        print(f"\n  SAVE this pose to which slot?  0 = the BASE pose "
                              f"(where Ctrl-C parks), 1-9 = a waypoint.")
                        print(f"     waypoints already saved: {saved_now or 'none'}"
                              f"        any other key cancels\n")
                    elif k == "p":
                        pending = "park"
                        park_sequence.clear()
                        have = "; ".join(
                            f"{one.name}: " + (", ".join(sorted(n for n in one.slots
                                                                if n != BASE_SLOT)) or "none")
                            for one in aimed)
                        print(f"\n  PARK to which?  0 = base, 1-9 = a waypoint, "
                              f"Enter = base.")
                        print(f"     Type several digits for a SEQUENCE, then Enter."
                              f"   waypoints: {have or 'none'}\n")
                    elif k in "oc" and any(one.mode == "teleop" for one in aimed):
                        # ⭐ Every selected arm's jaws. `o` and `c` are commands, and BOTH
                        # selected means the operator asked for both grippers.
                        jaw_step = gripper_step if k == "o" else -gripper_step
                        for one in aimed:
                            if one.mode == "teleop":
                                one.gripper_value = clamp_gripper(one.gripper_value + jaw_step)
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
                        for one in aimed:
                            one.park_ramp = min(1.0, one.park_ramp * 1.4)
                        hint(ease_note(EASINGS[ease_idx].name, edit_arm.park_ramp))
                    elif k in KEY_STEP_DOWN:
                        for one in aimed:
                            one.park_ramp = max(
                                0.0, one.park_ramp / 1.4 if one.park_ramp > 0.03 else 0.0)
                        hint(ease_note(EASINGS[ease_idx].name, edit_arm.park_ramp))
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
                        hint(ease_note(EASINGS[ease_idx].name, edit_arm.park_ramp))
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
                        edit_arm.axis_map.flip(idx)
                        print(f"\n  arm {edit_arm.name}: "
                              f"{motions_for(edit_arm.frame)[idx]['short']} flipped → "
                              f"{edit_arm.axis_map.row(idx, edit_arm.frame).strip()}\n")
                    elif k in "123":
                        # Rotation motions: 1 roll, 2 pitch, 3 yaw. Digits because every
                        # sensible letter was taken, and because they read as an
                        # ordered triple the way x/y/z do.
                        idx = 3 + "123".index(k)
                        edit_arm.axis_map.flip(idx)
                        print(f"\n  arm {edit_arm.name}: "
                              f"{motions_for(edit_arm.frame)[idx]['short']} flipped → "
                              f"{edit_arm.axis_map.row(idx, edit_arm.frame).strip()}\n")
                    elif k == "+" or k == "=":
                        # ⭐ In PARK these mean the park speed. The teleop linear scale
                        # is meaningless while the puck is not driving, and a key that
                        # does nothing where you are is the defect class that made `b`
                        # look broken (FINDINGS §17.1).
                        if any(one.mode == "park" for one in aimed):
                            for one in aimed:
                                one.park_speed = min(args.teleop_speed,
                                                     one.park_speed * 1.25)
                            hint(f"park speed {edit_arm.park_speed:.2f} rad/s")
                        else:
                            args.linear_scale *= 1.25
                            hint(f"linear speed {args.linear_scale:.3f} m/s")
                    elif k == "-":
                        if any(one.mode == "park" for one in aimed):
                            for one in aimed:
                                one.park_speed = max(0.05, one.park_speed / 1.25)
                            hint(f"park speed {edit_arm.park_speed:.2f} rad/s")
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
                # ⛔ Leaving MIRROR by any route drops the link. One place rather than a
                # cancel in each of g/t/h/i, because the one that gets forgotten is the one
                # that matters: a follower still tracking after the operator pressed HOLD is
                # an arm moving for a reason nobody can see.
                if mirror_link is not None and not any(one.mode == "mirror" for one in arms):
                    mirror_link = None
                    hint("")
                    print("  ⭐ MIRROR off — the follower left the mode.\n")

                # ⚠️ Per arm, because one arm can be parking while the other is being
                # driven. Each arm's run is abandoned by ITS OWN mode leaving `park`.
                for one in arms:
                    if one.mode == "park" or one.park_path is None:
                        continue
                    left = one.park_path.length - one.park_s
                    unfinished = left > PARK_TOLERANCE
                    if unfinished:
                        print(f"\n  ⚠️  arm {one.name}: run abandoned with {left:.2f} rad of "
                              "path left — leaving PARK cancels the rest.\n")
                    one.park_path, one.park_marks = None, []
                    # ⛔ A park that was interrupted must not hand over to a playback. The
                    # handover lives in the arrival branch, but this is the second gate:
                    # pressing h or t while driving to the start pose cancels the whole
                    # thing, rather than leaving a recording queued to fire later.
                    #
                    # ⛔⭐⭐ `unfinished` IS THE WHOLE FIX, and without it two-arm playback
                    # could never start. This used to cancel whenever it found a path on an
                    # arm that had left `park`, which includes an arm that ARRIVED. With one
                    # arm that never showed, because the arrival handed over in the same
                    # cycle and left nothing pending. With two arms the first arrival waits
                    # for the second, so the pending playback was still there to cancel:
                    # *"arm B is at the start pose; waiting for G"* and then *"playback
                    # cancelled — it never reached the start pose"*, one line apart, in
                    # Julien's own log. FINDINGS §57.1.
                    #
                    # ⭐ Deciding from the MEASURED remaining path rather than from whether
                    # some other branch remembered to tidy up is the fix that cannot rot: a
                    # future exit that forgets to clear the path still cannot cancel a
                    # playback whose park actually finished.
                    if unfinished and replay_pending is not None:
                        replay_pending = None
                        replay_ready = set()
                        hint("")
                        print("  ⚠️  playback cancelled — it never reached the start pose.\n")
                # ⛔ Same rule for a playback in progress: leaving the mode abandons it.
                # An arm resuming a recorded movement after the operator pressed HOLD is
                # doing something nobody asked for.
                # ⚠️ Session-level, because the playback cursor is: it is abandoned when NO
                # arm is in replay any more. `l` refuses at N>1 until the two-arm recorder
                # exists (ROADMAP §8.2 item 7), so today that is one arm leaving the mode.
                if replay is not None and not any(one.mode == "replay" for one in arms):
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
                #
                # ⭐⭐ EVERY ARM, CONCATENATED IN `--arms` ORDER, which is exactly the shape
                # ABC wants: 14 states per timestep, two arms in ONE timeline (ROADMAP §9.2).
                # `src/recording.py::Layout` owns the mapping and its tests.
                #
                # ⚠️ Sampled in ONE list comprehension so every arm's position comes from the
                # same cycle. Reading the arms in separate statements would put a control
                # cycle between them, and a demonstration whose two halves are 10 ms apart is
                # a demonstration with a lie in it.
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
                        take.append(t - take_t0,
                                    [v for one in arms
                                     for v in np.asarray(one.robot.get_joint_pos(),
                                                         dtype=float)])
                        # ⛔⭐ RECORD EVERY MODE THE RECORDING PASSED THROUGH, not only the
                        # one it started in. Julien's recording of 2026-08-13 17:21 was
                        # stamped `method: live:hold` because he pressed `w` while in HOLD
                        # and then switched to GUIDE to hand-guide it. **The stamp described
                        # the keypress rather than the demonstration**, and provenance is the
                        # thing ROADMAP §6.6 says matters most about a recording. A dataset
                        # that mislabels how a demonstration was produced is worse than one
                        # that omits it. FINDINGS §35.4.
                        # ⭐ Every arm's mode, so a two-arm demonstration records that arm B
                        # was hand-guided while arm G was mirroring it. `method` is the one
                        # thing ROADMAP §6.6 says matters most about a recording.
                        for one in arms:
                            stamp = f"{one.name}:{one.mode}"
                            if stamp not in take_modes:
                                take_modes.append(stamp)
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
                # ⭐⭐ EVERY ARM READS ITS OWN PUCK, EVERY CYCLE, IN EVERY MODE.
                #
                # ⛔ This whole block used to read `arm.reader` once, outside any loop.
                # With two arms that reads ONE hand and hands its deflection to both
                # arms — and the leaked loop variable at the bottom of it made the
                # gripper follow whichever arm the previous loop ended on
                # (FINDINGS §54.1).
                for one in arms:
                    one.raw_axes = one.reader.read()
                    buttons = getattr(one.reader, "buttons", 0)
                    pressed = buttons & ~one.buttons_prev              # rising edge only
                    one.buttons_prev = buttons

                    if one.learn_button is not None and pressed:
                        warn = one.axis_map.learn_button(one.learn_button, pressed)
                        if warn:
                            print(f"\n  ⚠️  {warn}\n")
                        elif one.learn_button == "open":
                            one.learn_button = "close"
                            print(f"  ✓ OPEN  ← button 0x{pressed:02x}")
                            print("   Now press the button you want for CLOSE …\n")
                        else:
                            one.learn_button = None
                            print(f"  ✓ CLOSE ← button 0x{pressed:02x}")
                            print(one.axis_map.buttons_row())
                            print("   (f swaps them if they are the wrong way round)\n")
                    elif pressed:
                        # A press counts as "the control you just used", so f reverses it.
                        # ⛔ But ONLY keys edit the map, exactly as for the axes: pressing
                        # a button never rebinds anything.
                        one.last_input_kind = "button"
                        if one.axis_map.button_action(pressed) is None:
                            print(f"\n  button 0x{pressed:02x} is not assigned — press b to set the "
                                  f"gripper buttons (works in any mode)\n")
                        elif one.mode not in ("teleop", "map"):
                            print(f"\n  gripper buttons move the jaws in TELEOP (t) and CONTROLS (m); "
                                  f"you are in {one.mode.upper()}\n")

                    if one.learn_button is None and one.robot.num_dofs() > N_ARM and one.mode in ("teleop", "map"):
                        action = one.axis_map.button_action(buttons)
                        if action == "open":
                            one.gripper_value = clamp_gripper(one.gripper_value + GRIPPER_BUTTON_RATE * dt)
                        elif action == "close":
                            one.gripper_value = clamp_gripper(one.gripper_value - GRIPPER_BUTTON_RATE * dt)

                # ⭐⭐ EVERY ARM ACTS ON ITS OWN MODE, in `--arms` order. ROADMAP §6.1.
                #
                # ⛔ Driving is NOT aimed by the selector: each arm follows its own puck
                # every cycle, which is the whole point of two arms. Only mode changes and
                # edits are aimed (`a`, and `ArmSelector`).
                #
                # ⚠️ The replay branch reads SESSION state — one cursor, one recording —
                # because ABC wants both arms in one timeline (ROADMAP §9.2). With two arms
                # in replay it would drive them from the same slice, which is wrong; that is
                # why `l` refuses when more than one arm is connected until the two-arm
                # recorder exists.
                for one in arms:
                    if one.mode in ("teleop", "map") and one.teleop is not None:

                        if one.mode == "map":
                            # ⭐ AXIS ISOLATION — Julien's design: only the strongest puck
                            # direction is applied, so the arm performs exactly one motion and
                            # it is obvious which gesture caused it. Half speed, because this
                            # is the mode you experiment in.
                            #
                            # ⛔ Note what is NOT here: any call that edits the map. Deflection
                            # observes; keys edit. The mode this replaced bound on deflection
                            # and destroyed the hand-dialled map (FINDINGS §11).
                            keep, value = isolate(one.raw_axes, one.last_active_axis)
                            if keep is not None:
                                one.last_active_axis, one.last_active_value = keep, value
                                one.last_input_kind = "axis"
                            drive_axes = isolated_axes(one.raw_axes, keep)
                            scale_l = args.linear_scale * CONTROLS_SCALE
                            scale_a = angular_scale * CONTROLS_SCALE
                        else:
                            drive_axes = one.raw_axes
                            scale_l, scale_a = args.linear_scale, angular_scale

                        axes = one.axis_map.apply(drive_axes)
                        twist = np.array([
                            axes[0] * scale_l, axes[1] * scale_l, axes[2] * scale_l,
                            axes[3] * scale_a if rotation else 0.0,
                            axes[4] * scale_a if rotation else 0.0,
                            axes[5] * scale_a if rotation else 0.0,
                        ])
                        q_target = one.teleop.step(twist, dt)

                        # ⭐⭐ THE WORKSPACE LIMIT, changed on 2026-08-14 by Julien's decision.
                        #
                        # It used to be a ±0.30 m cube centred on wherever TELEOP was entered.
                        # Measured on the arm, that stopped him at 0.524 m from the base while
                        # the arm reaches 0.738, and the wall sat somewhere different every
                        # session. It is now a fixed 0.60 m sphere around the base plus a floor.
                        # Why a floor: the cube had been providing one for free, and this arm
                        # can put its tip below its own base. See teleop.clamp_to_workspace and
                        # FINDINGS §43.
                        #
                        # ⚠️ Clamped against the ACHIEVED position so the limit ratchets inward
                        # and never yanks an arm that starts outside it. The old cube could not
                        # be entered from outside; a fixed one can.
                        ee = one.teleop.ee_position()
                        lim_r, lim_f = effective_limits(one.home_ee, args.reach, args.floor)
                        allowed = clamp_to_workspace(ee, lim_r, lim_f)
                        if not np.allclose(allowed, ee):
                            import mink  # noqa: PLC0415
                            one.teleop.target = mink.SE3.from_rotation_and_translation(
                                rotation=one.teleop.target.rotation(),
                                translation=allowed,
                            )

                        step = q_target - one.prev_q
                        q_target = one.prev_q + np.clip(step, -joint_step, joint_step)

                        lo = np.array([YAM_JOINTS[i][1] for i in range(1, N_ARM + 1)]) + JOINT_LIMIT_MARGIN
                        hi = np.array([YAM_JOINTS[i][2] for i in range(1, N_ARM + 1)]) - JOINT_LIMIT_MARGIN
                        q_target = np.clip(q_target, lo, hi)

                        full = np.zeros(one.robot.num_dofs())
                        full[:N_ARM] = q_target
                        if one.robot.num_dofs() > N_ARM:
                            full[N_ARM] = clamp_gripper(one.gripper_value)
                        one.robot.command_joint_pos(full)
                        one.prev_q = q_target.copy()

                    elif one.mode == "mirror" and mirror_link is not None:
                        # ⭐⭐ ONE ARM FOLLOWS THE OTHER. Every decision is `MirrorLink`'s
                        # (18 tests, no robot handle); this branch reads the two poses,
                        # carries the command out, and narrates. Same split as `replay_step`
                        # and `ArmSession` — the code that commands an arm is the code that
                        # cannot be tested without one, so it is kept as thin as possible.
                        lead_q = np.asarray(mirror_leader.robot.get_joint_pos(), dtype=float)
                        follow_q = np.asarray(one.robot.get_joint_pos(), dtype=float)
                        cmd = mirror_link.step(lead_q, follow_q, real_dt)
                        if cmd is None:
                            # ⛔ The link stopped itself. Continuing would keep commanding a
                            # pose the follower cannot reach, which is how a motor ends up
                            # held against a stop.
                            #
                            # ⭐ The reason NAMES the joint and the measured leader speed, and
                            # this line adds the joint's real name plus how to start again —
                            # both were missing when Julien first hit it.
                            # ⛔⭐ ONE FACT PER LINE. `StatusLine.say()` truncates each line
                            # to the terminal width while the live block is on screen, and
                            # Julien's first high-speed run lost the end of a long stop message
                            # to an ellipsis — the half that named the cause.
                            joint_name = ""
                            if mirror_link.stop_joint is not None:
                                joint_name = YAM_JOINTS.get(
                                    mirror_link.stop_joint + 1, ("joint",))[0]
                            print(f"\n⛔ MIRROR STOPPED — {mirror_link.stop_reason}"
                                  + (f", {joint_name}" if joint_name else ""))
                            print(f"     {mirror_link.stop_detail}")
                            # ⭐⭐ THE HARDWARE'S OWN EVIDENCE, which the pure class cannot
                            # see. `SafeRobot` counts every cycle on which one of its two
                            # limits actually bit, and one of those limits is the 0.25 rad
                            # following-error clip. A high count during a mirror run says the
                            # COMMAND was being held back from running ahead of the arm, which
                            # is the mechanism behind the "the arm could not track" case.
                            clipped = getattr(one.robot, "limited_cycles", 0) - mirror_clipped_at
                            if clipped > 0:
                                print(f"     ⚠️ SafeRobot held the command back on {clipped} "
                                      f"cycle(s) (its {getattr(one.robot, 'max_lag', 0.25)} rad "
                                      "following-error limit).")
                            if mirror_link.stop_cause == "follow_limit":
                                print(f"     ⭐ That allowance is `--max-speed`, now "
                                      f"{args.max_speed} rad/s. Raise it one step.")
                            elif mirror_link.stop_cause == "tracking":
                                print("     ⭐ More `--max-speed` will NOT help: the arm, not "
                                      "the software, is the limit.")
                                print(f"     Either guide the leader more slowly, or loosen "
                                      f"the tolerance with --mirror-gap "
                                      f"{mirror_link.max_gap * 2:.2f}.")
                            print("     Press i then Enter to engage it again.\n")
                            one.mode = "hold"; enter_hold(one); hint("")
                            mirror_link = None
                        else:
                            full = np.asarray(cmd, dtype=float).copy()
                            # ⛔ The jaws go through the clamp, never straight from the leader.
                            # A leader whose jaws rest on a stop would otherwise drive the
                            # follower's onto its own stop and HOLD there, which is stall
                            # torque and is how motor 7 was cooked three times (FINDINGS §4).
                            if one.robot.num_dofs() > N_ARM and len(full) > N_ARM:
                                full[N_ARM] = clamp_gripper(float(full[N_ARM]))
                            one.robot.command_joint_pos(full)
                            one.prev_q = full[:N_ARM].copy()

                    elif one.mode == "park" and one.park_path is not None and one.park_target is not None:
                        q = np.asarray(one.robot.get_joint_pos(), dtype=float)
                        # ⭐ Completion is judged from the MEASURED pose, never from the
                        # command — the command always arrives first, so testing it would
                        # declare success while the arm was still travelling.
                        err = float(np.max(np.abs(one.park_target - q)))
                        lag = float(np.max(np.abs(one.park_cmd - q))) if one.park_cmd is not None else 0.0
                        at_end = one.park_s >= one.park_path.length

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
                                    EASINGS[ease_idx], one.park_s, one.park_path.length - one.park_s,
                                    0.0 if args.no_smooth else one.park_ramp)
                                one.park_s = min(one.park_path.length,
                                             one.park_s + one.park_speed * ramp * dt)
                                advanced = True
                            one.park_cmd = one.park_path.point_at(one.park_s)
                            one.robot.command_joint_pos(one.park_cmd)

                            # Progress is "the cursor moved OR the arm closed the gap".
                            # Without the first half, a legitimately slow leg looks stalled;
                            # without the second, an arm pinned against something never does.
                            if advanced or err < one.park_best_err - PARK_PROGRESS_EPS:
                                one.park_best_err = min(one.park_best_err, err)
                                one.park_progress_t = t
                            if t - one.park_progress_t > PARK_STALL_SECONDS:
                                one.mode = "hold"; enter_hold(one); hint("")
                                print(f"\n⛔ PARK BLOCKED — the arm stopped following "
                                      f"{lag:.3f} rad behind the path, no progress for "
                                      f"{PARK_STALL_SECONDS:.0f}s. Now HOLDING.\n")
                            elif one.park_marks and one.park_s >= one.park_marks[0][1]:
                                # ⭐ Time each waypoint. Julien: *"you can't really see how
                                # long each parking section took, you can only see the park
                                # itself."* Now each leg reports its own seconds as it is
                                # passed, which is also the number to watch when tuning
                                # speed and corner radius.
                                name, _ = one.park_marks.pop(0)
                                print(f"  ⭐ slot {name} in {t - one.park_leg_t:.1f}s"
                                      + (f" → next {one.park_marks[0][0]}" if one.park_marks else ""))
                                one.park_leg_t = t
                            elif t >= next_park_report:
                                next_park_report = t + 1.0
                                # ⛔ A HINT, NOT THE STATUS ROW — same fix as the sequence
                                # echo. Routed through `screen.set` this replaced the
                                # temperature heartbeat with the progress readout, so during
                                # the one motion where an operator most wants to see a
                                # temperature climbing, it was the line that had been
                                # painted over.
                                hint(f"  moving… {one.park_path.length - one.park_s:.2f} rad of path "
                                     f"left, {err:.3f} to the final pose, {lag:.3f} behind")
                        else:
                            leg = park_verdict(err, t - one.park_progress_t > PARK_STALL_SECONDS,
                                               PARK_TOLERANCE, PARK_SETTLED,
                                               stopped_briefly=t - one.park_progress_t
                                               > PARK_SETTLE_SECONDS)
                            if leg in ("arrived", "settled"):
                                extra = ("" if leg == "arrived" else
                                         " — as close as the arm holds itself under load")
                                one.mode = "hold"; enter_hold(one)
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
                                total = t - one.park_start_t
                                settling = t - one.park_leg_t
                                tail = (f", {settling:.1f}s of that settling"
                                        if 0.05 < settling < total - 0.05 else "")
                                print(f"⭐ PARK reached in {total:.1f}s{tail} "
                                      f"({err:.3f} rad off{extra}) → HOLD")
                                # ⛔⭐⭐ THE ARRIVAL CLEARS ITS OWN PATH, AND THIS IS THE FIX
                                # FOR THE BUG THAT KILLED THE FIRST TWO-ARM PLAYBACK.
                                #
                                # The generic "leaving PARK abandons the run" block below runs
                                # once a cycle and fires for any arm whose mode is no longer
                                # `park` while `park_path` is still set. An ARRIVAL sets the
                                # mode to `hold` and used to leave the path in place, so on
                                # the next cycle that block treated a COMPLETED park as an
                                # abandoned one — and cancelled `replay_pending`, the pending
                                # playback the park existed to reach.
                                #
                                # ⚠️ It never showed with one arm, because the handover
                                # happened in the same cycle as the arrival, so
                                # `replay_pending` was already None when the block ran. With
                                # two arms the first arrival WAITS for the second, so the
                                # pending playback is still set and gets cancelled. Julien's
                                # own log: *"arm B is at the start pose; waiting for G"*
                                # immediately followed by *"playback cancelled — it never
                                # reached the start pose"*. FINDINGS §57.1.
                                one.park_path, one.park_marks = None, []
                                # ⭐ The handover from "drive to the start pose" to "play the
                                # recording". It lives HERE, in the arrival branch, so a park
                                # that was blocked or interrupted can never roll into a
                                # playback: only a park that actually arrived does.
                                if replay_pending is not None and one in replay_arms:
                                    # ⛔⭐ EVERY ARM MUST ARRIVE BEFORE ANY ARM PLAYS. Each
                                    # one parks a different distance and therefore finishes at
                                    # a different moment. Starting the clock on the first
                                    # arrival would have the second arm still parking while
                                    # the recording ran, and a two-arm demonstration whose
                                    # halves are seconds apart is not the one that was taught.
                                    replay_ready.add(one.name)
                                    waiting = [a.name for a in replay_arms
                                               if a.name not in replay_ready]
                                    if waiting:
                                        print(f"     arm {one.name} is at the start pose; "
                                              f"waiting for {', '.join(waiting)}.")
                                    else:
                                        replay = replay_pending
                                        replay_pending = None
                                        replay_t0, replay_s = t, 0.0
                                        replay_progress_t = t
                                        replay_held_s, replay_worst_lag = 0.0, 0.0
                                        replay_prev_target = list(replay.start_pose() or ())
                                        tracking = TrackingLog(replay.n_joints)
                                        for a in replay_arms:
                                            a.mode = "replay"
                                        print(f"\n▶  PLAYING {replay.duration:.1f}s of "
                                              f"recorded movement on "
                                              f"{'+'.join(a.name for a in replay_arms)} at "
                                              f"{replay_speed:.2f}x. Press h or t to stop.\n")
                            elif leg == "blocked":
                                # ⛔ Never spin silently. If the arm has stopped closing the
                                # gap the honest thing is to say so and hold, not to keep
                                # printing a number that is not changing — which is exactly
                                # how the old treadmill bug hid for two sessions.
                                one.mode = "hold"; enter_hold(one); hint("")
                                print(f"\n⛔ PARK BLOCKED — {err:.3f} rad still to go and no "
                                      f"progress for {PARK_STALL_SECONDS:.0f}s.")
                                print(f"   The command ran {lag:.3f} rad ahead of the arm; "
                                      f"SafeRobot limited "
                                      f"{getattr(one.robot, 'limited_cycles', 0)} cycles.")
                                print("   Something is blocking it, or the pose is "
                                      "unreachable. Now HOLDING.\n")
                            else:
                                one.robot.command_joint_pos(one.park_cmd)
                                if err < one.park_best_err - PARK_PROGRESS_EPS:
                                    one.park_best_err, one.park_progress_t = err, t

                # ---- 4a. the playback: ONE cursor, every arm it was recorded from ----
                #
                # ⭐⭐ SESSION-LEVEL, AND IT HAS TO BE. The cursor is a clock, and one clock
                # drives every arm. Inside the per-arm loop this block called `replay_step`
                # once per arm, which with two arms would advance the SAME cursor twice per
                # cycle — a playback running at double speed, silently.
                #
                # ⭐ FOLLOW THE RECORDING IN TIME, not along its length. A park traverses a
                # *shape* at a constant joint speed, which throws away the thing hand-guiding
                # provides: human timing and hesitation are the signal (ROADMAP §6.6).
                if replay is not None and replay_layout is not None and all(
                        a.mode == "replay" for a in replay_arms):
                    # ⛔ MEASURED IN THE RECORDING'S ARM ORDER, never the session's. The
                    # layout came out of the file, so the slices line up with the samples
                    # even if `--arms` was given the other way round.
                    measured = np.concatenate([
                        np.asarray(a.robot.get_joint_pos(), dtype=float) for a in replay_arms])
                    # ⛔ The grippers are left out of the "is it keeping up" check by INDEX,
                    # because with two arms the first gripper sits in the middle of the
                    # vector. Jaws legitimately sit far from their commanded value while
                    # closing on an object, and counting that as lag would stall every
                    # playback that grips anything.
                    rs = replay_step(replay, replay_s, measured, real_dt, speed=replay_speed,
                                     max_lag=MAX_CURSOR_LAG,
                                     compare=replay_layout.tracked_indices(N_ARM))
                    replay_s = rs.cursor
                    for a in replay_arms:
                        piece = np.asarray(rs.target[replay_layout.slice_for(a.name)],
                                           dtype=float)
                        full = np.asarray(a.robot.get_joint_pos(), dtype=float).copy()
                        n_j = min(N_ARM, len(piece))
                        full[:n_j] = piece[:n_j]
                        if a.robot.num_dofs() > N_ARM and len(piece) > N_ARM:
                            # ⛔ Through the clamp, never straight from the file. A recording
                            # made while the jaws rested on a stop would otherwise drive them
                            # back onto it and HOLD there. That is stall torque, and it is how
                            # motor 7 was cooked three times (FINDINGS §4).
                            full[N_ARM] = clamp_gripper(float(piece[N_ARM]))
                        a.robot.command_joint_pos(full)
                        a.prev_q = full[:N_ARM].copy()
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
                        for a in replay_arms:
                            a.mode = "hold"; enter_hold(a)
                        hint("")
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
                                # ⭐ Names per ARM, because with two arms the table is 14 rows
                                # and `YAM_JOINTS` only names seven. A table with "base_yaw"
                                # twice and no way to tell which arm is a measurement nobody
                                # can act on.
                                names = [
                                    f"{a.name} {YAM_JOINTS.get(j + 1, ('joint',))[0]}"
                                    for a in replay_arms
                                    for j in range(replay_layout.per_arm)
                                ][:tracking.n_joints]
                                rec = tracking.to_dict(names)
                                rec["meta"] = {
                                    "arms": [a.name for a in replay_arms],
                                    "joints_per_arm": replay_layout.per_arm,
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
                                    "max_planned_joint_speed": args.teleop_speed,
                                    "safe_max_speed": args.max_speed,
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
                        for a in replay_arms:
                            a.mode = "hold"; enter_hold(a)
                        hint("")
                        print("\n⛔ PLAYBACK BLOCKED — "
                              f"{'+'.join(a.name for a in replay_arms)} stopped following "
                              f"{rs.lag:.3f} rad behind the recording, no progress for "
                              f"{PARK_STALL_SECONDS:.0f}s. Now HOLDING.\n")
                        replay = None
                    elif t >= next_park_report:
                        next_park_report = t + 1.0
                        hint(f"  playing… {replay.duration - replay_s:.1f}s left, "
                             f"{rs.lag:.3f} rad behind")


                # ---- 5. report --------------------------------------------
                # CONTROLS mode reports continuously, not once a second: he is watching
                # the arm and the readout together to attribute a motion to a gesture,
                # and a 1 Hz readout is useless for that.
                # ⚠️ CONTROLS owns the whole live block while it is open, so the other arm's
                # row is not painted during it. That is a real gap at N>1 and it is deliberate
                # for now: the wizard is a full-screen conversation with one arm, and `m`
                # refuses when two arms are selected. Tracked in FINDINGS §54.2.
                wizard = next((one for one in arms if one.mode == "map"), None)
                if wizard is not None:
                    # ⛔ NOT `arm = wizard`. Rebinding the session's own `arm` here would repoint it
                    # at the wizard for the REST of the loop, including the incident record and
                    # the shutdown. At N=1 it is the same object, so it would have worked and
                    # proved nothing — the leaked-variable defect again (FINDINGS §54.1).
                    # Both scales are always shown. Julien could not tell that ,/. were
                    # doing nothing here because only the active axis's resulting speed
                    # was displayed — a missing key looked identical to a key that worked.
                    speeds = (f"lin {args.linear_scale * CONTROLS_SCALE:.3f} m/s  "
                              f"rot {np.degrees(angular_scale * CONTROLS_SCALE):.0f}°/s"
                              f"{'' if rotation else ' (OFF)'}")
                    if wizard.last_active_axis is None:
                        print(f"\r[CONTROLS] push the puck …  {axes_readout(wizard.raw_axes)}  {speeds}   ",
                              end="", flush=True)
                    else:
                        drv = wizard.axis_map.motion_driven_by(wizard.last_active_axis)
                        if drv is None:
                            doing = "→ nothing (press 1-6 to assign)"
                        else:
                            v = wizard.axis_map.apply(isolated_axes(wizard.raw_axes, wizard.last_active_axis))[drv]
                            unit = (f"{v * args.linear_scale * CONTROLS_SCALE:+.3f} m/s" if drv < 3
                                    else f"{np.degrees(v * angular_scale * CONTROLS_SCALE):+.1f}°/s")
                            doing = f"→ {motions_for(wizard.frame)[drv]['short']} {unit}"
                        print(f"\r[CONTROLS] puck {PUCK_AXES[wizard.last_active_axis]:<5} "
                              f"{wizard.last_active_value:+.2f}  {doing:<28} {speeds}"
                              f"{' ' * 6}", end="", flush=True)
                elif t >= next_report:
                    next_report += 1.0
                    # ⭐⭐ ONE ROW PER ARM. ROADMAP §6.1 step 2.
                    #
                    # ⭐ THE SESSION FACTS RIDE THE FIRST ROW AND ARE NOT REPEATED. The
                    # clock, the recording and the loop rate belong to the session, not to
                    # an arm, and printing them twice would invite reading two arms as
                    # having two clocks. Later rows are padded to the same width so the
                    # temperature columns line up down the block.
                    #
                    # ⚠️ Padded with `display_width`, not `len`: `⏺` and `⚠️` are one
                    # character and two columns, so `len` would misalign the rows by
                    # exactly the number of symbols (`src/screen.py::display_width`).
                    lead = f" t={t:6.1f}s"
                    # ⭐ RECORDING HAS TO BE VISIBLE ON THE HEARTBEAT, not only in the
                    # message that started it. A session where recording is silently still
                    # running produces a demonstration full of whatever happened next, and
                    # the operator finds out at training time.
                    if take is not None:
                        lead += f"  ⏺ REC {t - take_t0:5.1f}s"
                    # ⭐ THE LOOP RATE, because it was 87 Hz for a whole session and nothing
                    # said so. It only became visible when a playback summary failed to add
                    # up. Shown only when it drops, so a healthy loop costs no width.
                    if loop_hz < 0.92 * CONTROL_HZ:
                        lead += f"  ⚠️{loop_hz:3.0f}Hz"
                    pad = " " * display_width(lead)
                    screen.set_rows([
                        status_row(one, lead if i == 0 else pad, args.reach, args.floor,
                                   note=(mirror_link.status(
                                       mirror_leader.robot.get_joint_pos(),
                                       one.robot.get_joint_pos())
                                       if mirror_link is not None and one.mode == "mirror"
                                       else ""))
                        for i, one in enumerate(arms)])

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
            # ⭐⭐ EXTENDED 2026-08-14 FROM CTRL-C TO EVERY UNPLANNED STOP. Julien's
            # request, after a chain death dropped the arm: *"when the robot is being
            # moved and stuff, and then it crashes for some reason, it should always
            # resort to trying to do the safe crash… It should be, like, when I do
            # control c."*
            #
            # ⛔⭐ BE EXACT ABOUT WHAT THIS CAN AND CANNOT DO, because the failure that
            # prompted it is the one case it cannot help. When the CAN link dies —
            # which is what happened, all seven motors latching `0xD loss of
            # communication` (FINDINGS §46) — **the arm cannot be commanded at all**,
            # so no park is possible and it sags. `chain_alive()` is the gate, and it
            # was already false in that session.
            #
            # ⭐ What it DOES cover is every other way a session can end badly, and
            # those are the majority: an exception in our own loop, a thermal stop, a
            # guard refusing, an IK failure. In all of those the chain is still alive
            # and the arm was previously left holding until a human answered a menu.
            #
            # ⚠️ A thermal stop parks too, deliberately. Holding keeps current in a hot
            # motor indefinitely and disabling drops the arm; parking gets it to a
            # supported pose and THEN removes current, which is better than both.
            #
            # ⛔ `q` is deliberately NOT auto-parked. Julien uses `q p d` and may want
            # `g` instead, so a planned quit keeps its menu. Only unplanned stops park
            # themselves.
            planned_quit = bool(stop_reason) and "quit requested" in (stop_reason or "")
            unplanned = not planned_quit
            auto_parked = False
            # ⚠️ `any(...)` rather than `all(...)`: if one chain has died the other arm can
            # still be parked, and parking it is better than leaving it holding. `park_arms`
            # skips the dead one and names it.
            live = [one for one in arms if one.alive()]
            if live and (interrupted or unplanned) and any(
                    one.base_pose is not None for one in live):
                for one in live:
                    enter_hold(one)
                if interrupted:
                    print("\n⭐ Ctrl-C — parking to the pose this session started in, then")
                else:
                    print(f"\n⭐ SAFE STOP after {stop_reason!r} — the chain is still alive, so")
                    print("   the arms are being parked to the pose this session started in, then")
                print("   disabling. Press any key to stop the motion; Ctrl-C again forces out.")
                outcome = park_arms(live, keys, clamp_gripper)
                if outcome == "arrived":
                    auto_parked = True
                    print("\n   Disabling the motors now.\n")
                else:
                    print(f"\n⚠️  the automatic park ended as {outcome!r}, so nothing is "
                          "being released. Choose below.")

            if any(one.alive() for one in arms) and not auto_parked:
                for one in arms:
                    if one.alive():
                        enter_hold(one)
                print("\nEvery arm is HOLDING its pose. Nothing is released until you choose.")
                print("   q = PARK then DISABLE — the whole shutdown in one key")
                print("   p = PARK — drive back to the park pose, then it holds there")
                print("   g = go weightless so you can park it by hand")
                print("   d = disable now (⚠️ a raised arm will sag)")
                # ⭐ Discoverability, not a new feature. `p` in a NORMAL session already
                # parks and leaves the arm holding, so `t` afterwards carries straight on.
                # Julien described wanting *"q p doing the base position and then going
                # back to teleoperate and continuing"*, and the plain `p` key does that
                # today without quitting at all. Saying so here costs one line.
                print("   ⭐ to park WITHOUT quitting, use p in the session itself, then t")
                while True:
                    k = keys.get()
                    if k == "q":
                        # ⭐⭐ PARK THEN DISABLE, ONE KEY. Julien's request, 2026-08-14:
                        # *"There should be an option that just combines park and disable.
                        # Maybe just pressing q should allow for park and disable."*
                        #
                        # `q q` is now the keyboard equivalent of Ctrl-C, which has parked
                        # and disabled in one action since 2026-08-12. Same motion, same
                        # guards, same interruptibility.
                        #
                        # ⛔ IT ONLY RELEASES THE ARM IF THE PARK ACTUALLY ARRIVED. That is
                        # the same rule the Ctrl-C path follows: *"I could not reach the safe
                        # pose"* is exactly when a human should decide rather than a default.
                        # A stalled or interrupted park leaves the arm holding and the menu
                        # open.
                        outcome = park_arms([one for one in arms if one.alive()],
                                            keys, clamp_gripper)
                        if outcome == "arrived":
                            print("\n   Parked. Disabling the motors now.\n")
                            break
                        for one in arms:
                            if one.alive():
                                enter_hold(one)
                        print(f"\n⚠️  the park ended as {outcome!r}, so nothing is being "
                              "released.")
                        print("   q = try again    p = park    g = weightless    d = disable")
                    elif k == "p":
                        # ⭐ Julien's request: *"it would also be good to do park mode
                        # [at quit], because then I can do park mode and then disable…
                        # I don't have to do anything with my hands."* With the park
                        # pose defaulting to wherever the arm started, `q p d` is a
                        # complete hands-free shutdown — and Ctrl-C now does the same
                        # thing in one keystroke.
                        park_arms([one for one in arms if one.alive()], keys, clamp_gripper)
                        for one in arms:
                            if one.alive():
                                enter_hold(one)
                        print("   q = park+disable    p = park again    g = weightless    d = disable")
                    elif k == "g":
                        # ⛔ EVERY arm, and with two that is 8.6 kg going weightless at once.
                        # The operator asked for it at the quit menu, where the alternative is
                        # disabling, so it is the safer of the two. The banner says how much.
                        for one in arms:
                            if one.alive():
                                enter_guide(one)
                        print(f"\n⭐ weightless: {'+'.join(one.name for one in arms if one.alive())}"
                              " — park them by hand, then press d to disable.")
                    elif k == "d":
                        break
                    time.sleep(0.05)
                    if not any(one.alive() for one in arms):
                        print("\n⚠️  every chain died while waiting — disabling now.")
                        break
            elif not any(one.alive() for one in arms):
                # ⚠️ `elif not chain_alive(...)`, not a bare `else`. With the Ctrl-C
                # auto-park above, a plain `else` would fire on the SUCCESS path and
                # announce a dead chain to someone whose arm had just parked fine.
                print("⚠️  every chain is already dead, so no arm is being commanded.")
                print("   They will be sagging under gravity. Support them now if raised.")

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
            for one_puck in pucks.values():
                one_puck["handle"].close()
        except Exception:  # noqa: BLE001, S110
            pass
        # ⛔ DO NOT make this an unconditional save again.
        #
        # It was one, and on 2026-08-10 it wrote a map mangled by the old bind-on-
        # deflection MAP mode straight over Julien's hand-dialled file. The values had
        # been produced on real hardware and were only recoverable because the file
        # happened to be committed. Two changes: nothing is written unless the map
        # actually changed, and the previous contents are kept alongside it.
        # ⚠️ THIS IS THE `finally` BLOCK, so it also runs on the path where
        # `build_robot()` failed and no arm was ever created — the path Julien hits
        # whenever the CAN adapters are in DFU (FINDINGS §48.3). `arms` is empty there,
        # so the loop simply does not run and nothing is written unless `--fork-map`
        # already changed the store.
        #
        # ⭐ Each arm saves ITS map under the frame IT ended in. Both live on the object,
        # which is why this no longer needs a session-level copy of either.
        for one in arms:
            map_store.set(one.name, one.axis_map, one.frame)
        if map_store != map_store_at_start:
            try:
                if MAP_FILE.exists():
                    BACKUP_FILE.write_text(MAP_FILE.read_text())
                map_store.save(MAP_FILE)
                print(f"\naxis map CHANGED and saved → {MAP_FILE.relative_to(REPO)}")
                print(f"  scope: {map_store.scope_note(arm_names[0])}")
                print(f"  previous contents kept in {BACKUP_FILE.relative_to(REPO)}")
            except Exception as exc:  # noqa: BLE001
                print(f"\n⚠️  could not save the axis map: {type(exc).__name__}: {exc}")
        # ⛔⭐ EVERY ARM IS DISABLED, and each one is wrapped on its own. If the FIRST
        # `shutdown_robot()` raised, an unwrapped loop would leave the second arm's motors
        # ENERGISED and unattended, which is the worst possible outcome of a teardown. An
        # arm is appended to `arms` the moment it is constructed, so a session whose second
        # build failed still disables the first arm here.
        if arms:
            _SHUTTING_DOWN["yes"] = True
            for one in arms:
                try:
                    disabled = shutdown_robot(one.robot)
                    print(f"\narm {one.name} motors confirmed disabled: {disabled}")
                except Exception as exc:  # noqa: BLE001
                    print(f"\n⛔ arm {one.name}: could not confirm the motors are disabled: "
                          f"{type(exc).__name__}: {exc}")
                    print("   ⚠️ TREAT THAT ARM AS LIVE. Cut the mains if it is raised.")

            # ⭐⭐ RECORD THE MOMENT, IF SOMETHING WENT WRONG. FINDINGS §45.
            #
            # On 2026-08-14 the arm fell because a motor stopped answering the CAN bus,
            # and everything about that instant was lost: no torques, no temperatures,
            # and no record of the USB bus, which is where the leading explanation
            # turned out to be. Recovering the gravity torques took a simulation of the
            # joint angles the arm had already measured and discarded.
            #
            # ⛔ PLACED HERE ON PURPOSE: after `shutdown_robot()`, so the motors are
            # already off before any of this is attempted. `write_incident` cannot
            # raise, and every field it gathers is individually guarded, because half
            # of them throw on a chain that has already died. A crash report that
            # delays the teardown would be worse than no crash report.
            #
            # ⚠️ Only on a bad stop. A normal `q p d` writes nothing.
            #
            # ⛔⭐ THE WHOLE BLOCK IS WRAPPED, and the reason is FINDINGS §42.0: a dry run
            # returns long before this line, and no headless test can reach it either,
            # because it needs a real robot. So this code path's FIRST execution will be
            # on the arm, during a failure. `src/incident.py` is unit-tested and every
            # field is individually guarded — this outer guard exists because a path that
            # cannot be tested should not be able to add a second traceback on top of the
            # one the operator is already reading.
            try:
                bad_stop = bool(stop_reason) and "quit requested" not in (stop_reason or "")
                # ⚠️ Every value below is the LAST one the loop managed to read, not a
                # fresh read. A fresh read on a dead chain raises, and the last good
                # reading is what actually describes the failure. Each arm keeps its own
                # last read in `one.states` / `one.temps` for exactly this.
                #
                # ⭐ ONE ENTRY PER ARM, in `arms` order. With two arms a single `"arm"` key
                # would have to name one of them, and the other arm's torques at the moment
                # of the fall are the ones that might explain it. `arms` is empty on the
                # failed-build path, so the list is simply empty rather than guarded.
                facts = {} if not bad_stop else {
                    "stop_reason": stop_reason,
                    "arms": [one.name for one in arms] or arm_names,
                    "reach_limit": args.reach,
                    "floor_limit": args.floor,
                    "loop_hz": _safe_fact(lambda: round(loop_hz, 1)),
                    "per_arm": [
                        {
                            "arm": one.name,
                            "mode": _safe_fact(lambda one=one: one.mode),
                            "commanded_joints": _safe_fact(
                                lambda one=one: [round(float(v), 4) for v in one.prev_q]),
                            "measured_joints": _safe_fact(
                                lambda one=one: [round(float(getattr(s, "pos", float("nan"))), 4)
                                                 for s in one.states]),
                            "ee": _safe_fact(
                                lambda one=one: [round(float(v), 4)
                                                 for v in one.teleop.ee_position()]),
                            "hottest_seen_c": _safe_fact(lambda one=one: one.thermal.max_seen),
                            "hottest_jaw_seen_c": _safe_fact(
                                lambda one=one: one.thermal.max_jaw_seen),
                            "last_temperatures_c": _safe_fact(
                                lambda one=one: [round(float(v), 1) for v in one.temps]),
                            # ⭐ The field whose absence cost the most on 2026-08-14: the
                            # gravity torques at the moment of failure had to be recovered by
                            # simulating the joint angles, when the arm had measured them and
                            # thrown them away.
                            "last_torques_nm": _safe_fact(
                                lambda one=one: [round(float(getattr(s, "eff", float("nan"))), 3)
                                                 for s in one.states]),
                            "chain_alive": _safe_fact(lambda one=one: bool(one.alive())),
                        }
                        for one in arms
                    ],
                }
                if bad_stop:
                    print("\n" + describe(write_incident(stop_reason or "unknown", facts)))
            except Exception as exc:  # noqa: BLE001
                # ⛔ Swallowed on purpose. The motors are already disabled; a traceback
                # here would sit on top of the real failure and read like a second fault.
                print(f"\n⚠️  could not record the incident: {type(exc).__name__}: {exc}")

    # ⛔⭐ GUARDED, AND THIS IS THE WHOLE REASON `arm` IS DECLARED None ABOVE.
    #
    # This runs after the `finally`, which means it also runs when `build_robot()` FAILED
    # and the `except Exception` printed the error. That is the path Julien sees whenever
    # the CAN adapters are in DFU. Without the guard, `arm.thermal` would raise there and
    # replace *"No candleLight CAN adapter found"* with a traceback. FINDINGS §48.3.
    #
    # ⭐ The failed-build branch also reads better than what it replaced: it used to print
    # `hottest motor seen this session: 0°C`, which is a fabricated number for a session
    # that never ran. A thermal guard reporting a plausible zero is the exact defect
    # `ThermalGuard` was written to remove ([FINDINGS §24](../docs/FINDINGS.md)).
    # ⭐ One temperature report per arm, and `arms` is EMPTY when the build failed — the
    # same guard as everywhere else in this teardown, expressed as a loop that does not run.
    for one in arms:
        print(f"\narm {one.name} hottest motor seen this session: "
              f"{one.thermal.max_seen:.0f}°C")
        if one.thermal.max_jaw_seen:
            # The number that decides whether the gripper frame fix held. A plateau near
            # idle (31-36 °C) is the pass; a steady climb is the failure, and it is
            # invisible in `hottest` because the shoulder runs hotter all session.
            print(f"  hottest the GRIPPER (motor 7) got: {one.thermal.max_jaw_seen:.0f}°C")
    if not arms:
        print("\nno temperatures to report — no robot was built, so no motor ran.")
    # ⚠️ One report per arm, and `arms` is EMPTY on the failed-build path — same guard as
    # the thermal lines above, expressed as a loop that does not run rather than as an
    # `if`. That is why `arms` is declared before the `try` (FINDINGS §48.3).
    for one in arms:
        print(f"axis map {one.name}: {one.axis_map.one_line(one.frame)}")
        if one.axis_map != one.axis_map_at_start:
            print(f"     was: {one.axis_map_at_start.one_line()}")
        else:
            print("     unchanged — nothing was written.")
        if one.axis_map.unbound():
            names = ", ".join(motions_for(one.frame)[i]["short"]
                              for i in one.axis_map.unbound())
            print(f"  ⚠️  UNBOUND, arm {one.name} will not perform these: {names}")
    if not arms:
        # ⭐ Say what the map WOULD have been rather than nothing at all. The build failed,
        # so no map was edited, and the store still holds what the session would have used.
        print(f"\naxis map {arm_names[0]}: "
              f"{map_store.for_arm(arm_names[0], start_frame).one_line(start_frame)}")
        print("     nothing was edited — the robot was never built.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
