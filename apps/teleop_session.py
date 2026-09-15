#!/usr/bin/env python3
"""Interactive YAM operator session.

Run ./teleop --help for flags and docs/COMMANDS.md for keys. Without --yes
the program prints its plan and opens no devices. --sim uses fake arms and
still pucks. Physical operation needs a clear workspace and an operator.

This application coordinates acquisition, the shared control loop, prompts and
shutdown. ArmSession owns per-arm modes. RecordingSession owns the trajectory
lifecycle. Historical source notes are in docs/archive/teleop-source-notes.md.
"""

from __future__ import annotations

import argparse
import json
import signal
import sys
import threading
import time
from datetime import datetime
from pathlib import Path
# ⚠️ `Any` was used in this file's annotations since long before this import existed, and
# it worked only because `from __future__ import annotations` never evaluates them. A real
# import is needed the moment it appears on a variable inside `main()`, because
# `checks/check_restructure.py` check 4 resolves every name used there.
from typing import Any

import numpy as np

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "third_party" / "i2rt"))
from yam.inputs.axis_map import (  # noqa: E402
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
from yam.inputs.axis_map import AxisMapStore  # noqa: E402
from yam.inputs.axis_map import N as N_AXES  # noqa: E402
from yam.inputs.keyboard import KeyReader  # noqa: E402
from yam.inputs.spacemouse import (  # noqa: E402
    TwistReader,
    countdown_hands_off,
    find_all_devices,
    open_device,
    pick_device_by_wiggle,
)
from yam.session import ArmSelector, ArmSession, ParkLeg, parse_arms  # noqa: E402
from yam.incident import describe, write_incident  # noqa: E402
from yam.mirror import (  # noqa: E402
    DEFAULT_ALIGN_SPEED,
    DEFAULT_CATCHUP,
    DEFAULT_MAX_GAP,
    MirrorLink,
    pick_pair,
)
from yam.motion import EASINGS, easing_factor  # noqa: E402
from yam.recording import (  # noqa: E402
    Layout,
    describe_slot,
    slot_overview,
    Trajectory,
    SCRUB_MAX_RATE,
    safe_time_scale,
)
from yam.ui.session_status import flat_joint_names, status_row, tracking_table  # noqa: E402
from yam.ui.screen import StatusLine, display_width  # noqa: E402
from yam.teleop import (  # noqa: E402
    FLOOR_LIMIT,
    FRAMES,
    REACH_LIMIT,
    CartesianTeleop,
    clamp_to_workspace,
    effective_limits,
)
from yam.recording_store import save_take  # noqa: E402
from yam.recording_session import RecordingSession  # noqa: E402
from yam.composite import CompositeRun  # noqa: E402
from yam.playback_session import PlaybackSession  # noqa: E402
from yam.provenance import dt_now, git_commit  # noqa: E402
from yam.cameras.session import open_session_cameras  # noqa: E402
from yam.cameras.capture import CaptureSet  # noqa: E402
from yam.cameras.specs import sim_camera_error  # noqa: E402
from yam.files import listing  # noqa: E402 — the OS-litter filter, FINDINGS §76
from yam.timing import LoopTimer  # noqa: E402 — the worst pass, PERFORMANCE.md §2
from yam.cameras.writer import (  # noqa: E402
    FrameSink,
    pending_frames_dir,
)
from yam.fake.arm import StillPuck, build_fake_robot  # noqa: E402
from yam.settings import (  # noqa: E402
    LIVE_ORDER,
    adjust as adjust_setting,
    live_lines,
    one_line as setting_line,
    defaults_path,
    describe as describe_defaults,
    effective as effective_settings,
    load_defaults,
    looser_than_builtin,
    rejected_keys,
    save_defaults,
)
from yam.can import ARM_SERIALS, DEFAULT_ARM, YAM_JOINTS  # noqa: E402
from yam.robot import (  # noqa: E402
    SAFE_MAX_LAG,
    SAFE_MAX_SPEED,
    VEL_FF_CEILING,
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
# ⭐ Defined in src/yam/inputs/axis_map.py so `apps/map_axes.py` reports the exact speeds
# this session commands. Dialling a mapping against speeds the arm does not use
# would teach the wrong feel.
LINEAR_SCALE = DEFAULT_LINEAR_SCALE     # m/s at full deflection  (was 0.04)
ANGULAR_SCALE = DEFAULT_ANGULAR_SCALE   # rad/s at full deflection (was 0.25)

# ⚠️ `WORKSPACE_BOX = 0.30` used to live here. The workspace limit is now
# `REACH_LIMIT` and `FLOOR_LIMIT` in `src/yam/teleop.py`, next to the code that applies
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
# German-keyboard aliases are decoded by KeyReader as UTF-8.
# Live speed bounds prevent runaway key repeat; joint-rate and lag limits still apply.
# The 15 m/s input ceiling preserves the operator's configured range (FINDINGS §62.2).
MAX_LINEAR_SCALE = 15.0
MIN_LINEAR_SCALE = 0.005

#: ⭐ The same backstop for rotation. Default is 0.6 rad/s, so 12 rad/s is 20x it and about
#: 690°/s — well past useful and nowhere near reachable, which is what a backstop should be.
#: ⚠️ His 2026-08-17 readout showed `rot 954°/s`, so this direction was being pushed too.
MAX_ANGULAR_SCALE = 12.0
MIN_ANGULAR_SCALE = 0.02

KEY_STEP_DOWN = ("[", "ö")          # shorter ease ramp
KEY_STEP_UP = ("]", "ä")            # longer ease ramp
# ⚠️ `m` is absent on purpose: CONTROLS owns the keyboard while it is active, so `m`
# pressed inside it is handled by that branch and never reaches the check that uses this.
MODE_KEYS = {"g": "GUIDE", "t": "TELEOP", "h": "HOLD"}
# Path lag bounds tracking fidelity; SafeRobot separately bounds command lead.
# Reuse the mirror module's alignment speed so its plan and execution agree.
MIRROR_ALIGN_SPEED = DEFAULT_ALIGN_SPEED
MAX_CURSOR_LAG = 0.15
# Allow brief settling near the target; use the longer stall timeout for obstructions.
PARK_SETTLE_SECONDS = 0.5
# Judged on the MEASURED pose, so it must allow for a position controller's
# steady-state error. 0.02 rad is 1.1°, which is "arrived" for parking.
PARK_TOLERANCE = 0.02
# Measured steady-state error can exceed PARK_TOLERANCE.
# A settled pose inside this second threshold is success; a distant stall is not.
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

# Keep operator-requested jaw commands off mechanical stops to avoid stall heating.
# Do not apply this clamp on mode entry: that would move jaws without a request.
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
# Recordings are local data; calibration lives separately in tracked config files.
TAKES_DIR = REPO / "recordings"
# Playback logs are timestamped under the current recordings directory.
# This is the allowed planned joint speed, not measured tracking capability.
MAX_PLANNED_JOINT_SPEED = MAX_JOINT_STEP * CONTROL_HZ

# ⛔⭐ How far (rad, worst ARM joint, jaws excluded) a replay arm may sit from the recording's start pose at the moment playback would begin. A park arrives within ~0.05 rad and settles within ~0.05 more, so 0.25 — the default max-lag, the point past which the first command is a real yank — separates "settled short" from "never parked here at all". FINDINGS §72.1: a playback once began 1.28 rad off because the READY BOOKKEEPING lied; this check measures the arm instead of trusting the bookkeeping, so that whole failure class refuses instead of dragging.
REPLAY_START_TOLERANCE = 0.25
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
  SETTINGS  n  the speed and safety limits, LIVE — then s saves them for every session
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
    see `src/yam/inputs/axis_map.py` for the numbers and for why "forward" is not claimed."""
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
    an exception during teardown.** See `src/yam/incident.py` for the same rule stated once.
    """
    try:
        return fn()
    except Exception as exc:  # noqa: BLE001
        return f"<unavailable: {type(exc).__name__}: {exc}>"


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
    """Park live arms with bounded motion, then return whether every park succeeded.

    Used by controlled shutdown and its quit menu. Resync each command limiter
    first; stop on operator input, lost liveness or stalled progress. A failed
    park must return control to the operator rather than imply a safe disable.
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
        # Resync the rate limiter before parking, including after hand-guiding in the quit menu.
        # Otherwise its cached command can pull the arm toward the previous pose.
        # Test fakes may omit resync; live handles are SafeRobot-wrapped.
        resync = getattr(one.robot, "resync", None)
        if resync is not None:
            resync()
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
                # Report measured error first. Controller deadband, load and contact are possible causes.
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
            # Use the same easing for quit-menu parks and interleaved in-session parks.
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


def save_slot_action(key: str, has_take: bool, was_replacing: bool,
                     replace_slot: str | None, occupied: str | None) -> tuple:
    """Choose save, ask or discard for the recording slot prompt.

    An occupied slot needs the same digit twice. A different digit re-aims.
    A non-digit discards the new take and keeps the occupied slot. Returns
    (save, slot), (ask, slot, reaimed), or (discard, previous_slot_or_None).
    """
    if not (key.isdigit() and has_take):
        return ("discard", replace_slot if was_replacing else None)
    confirmed = was_replacing and key == replace_slot
    if occupied is not None and not confirmed:
        return ("ask", key, bool(was_replacing and key != replace_slot))
    return ("save", key)








def main() -> int:  # noqa: PLR0915
    ap = argparse.ArgumentParser(description="Interactive YAM session: guide, teleop, park.")
    ap.add_argument("--yes", action="store_true", help="actually energise the arm")
    ap.add_argument("--sim", action="store_true",
                    help="run the WHOLE loop against simulated arms — no hardware, no CAN "
                         "adapter, no SpaceMouse needed. See src/yam/fake/arm.py for what it "
                         "can and cannot tell you.")
    # ⭐⭐ TWO SPELLINGS OF ONE IDEA, and `--arm` is the one that must not break.
    # Every other script here takes `--arm` (`ping_motors.py`, `identify_arm.py`,
    # `check_arms_match.py`), it is in every document, and it is what Julien types.
    # `--arms` is the N-arm spelling ROADMAP §6.1 step 2 asks for. They agree or the
    # session refuses; `src/yam/session.py::parse_arms` holds the rules and the tests.
    ap.add_argument("--arm", default=None, choices=sorted(ARM_SERIALS),
                    help=f"the arm, when there is one (default {DEFAULT_ARM})")
    ap.add_argument("--arms", default=None, metavar="B[,G]",
                    help="the arms this session drives, comma separated; use "
                         "--start-mode hold when starting both arms")
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
    # something else: the same call `src/yam/can.py` made when `--arm arm1` became
    # `--arm B`. A flag that keeps working while its meaning has changed underneath is
    # worse than one that fails loudly. `--box` now errors, which is the point.
    ap.add_argument("--reach", type=float, default=REACH_LIMIT,
                    help=f"how far the tip may go from the BASE, in metres (default "
                         f"{REACH_LIMIT}). Replaced a ±0.30 m cube that re-centred on "
                         f"wherever TELEOP was entered, so the wall moved every session "
                         f"and stopped him at 71%% of the arm's reach. The arm can reach "
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
    ap.add_argument("--max-lag", type=float, default=SAFE_MAX_LAG,
                    help=f"how far the command may run ahead of the MEASURED pose, in radians "
                         f"(default {SAFE_MAX_LAG}). ⛔⭐ THIS IS THE LIMIT THAT ACTUALLY CAPS "
                         f"TRACKING, and it had no flag until 2026-08-15. A MIRROR follower "
                         f"cannot be pulled closer by more --max-speed, because the command is "
                         f"clipped to this distance from where the arm already is "
                         f"(FINDINGS §57.3). ⚠️ IT IS ALSO A TORQUE LIMIT: the motor's push is "
                         f"kp × (command − measured), so raising this makes the arm push harder "
                         f"to catch up. That is how you get faster tracking AND how you get a "
                         f"harder hit. Raise it in small steps (0.25 → 0.35) and watch what the "
                         f"arm does when it meets something")
    ap.add_argument("--vel-ff", type=float, default=0.0, metavar="GAIN",
                    help="⭐ velocity feedforward gain, 0..1 (default 0 = off, item 44). "
                         "The motors' MIT-mode frame carries a velocity setpoint and this "
                         "stack always sent zero, so all torque came from position error — "
                         "the measured 0.033 s × speed lag is that (FINDINGS §66.1). At "
                         "GAIN > 0 each motor also receives GAIN × the rate-limited "
                         "command's own derivative, so torque flows before error builds. "
                         "1 = exactly the command's speed, the physically-motivated value "
                         "and the hard cap. ⛔ Values above 1 existed for one day and are "
                         "a measured dead end — jitter raw, 5-10 Hz stepping gated "
                         "(FINDINGS §68.6); anything higher is clamped to 1. The jaw "
                         "never gets feedforward. Live: setting 9 on the n screen.")
    ap.add_argument("--scrub-max", type=float, default=SCRUB_MAX_RATE, metavar="RATE",
                    help="⭐ the puck scrub's full-push pace, in recording-seconds per "
                         "second (default %(default)s). 1 = the recording's own pace; "
                         "higher = a time lapse for skimming to a moment. -/+ change it "
                         "LIVE during a scrub. Safe high: a fast cursor is held back by "
                         "the lag hold, so only the clock is fast, never the arm.")
    ap.add_argument("--mirror-catchup", type=float, default=DEFAULT_CATCHUP,
                    metavar="PER_SECOND",
                    help="⭐ how fast MIRROR corrects the follower's STANDING offset. 0 = off "
                         "(the default). Try 3. The follower is position-controlled, so it "
                         "settles short of its command by 0.012-0.024 rad, which is 5-11 mm "
                         "at the tip and is why the mirrored arm sits in a ~2 cm sphere "
                         "instead of on the spot. This aims slightly past the leader until "
                         "the follower actually arrives, only while the leader is moving "
                         "slowly, clamped to 0.06 rad.")
    ap.add_argument("--mirror-gap", type=float, default=DEFAULT_MAX_GAP,
                    help=f"how far the MIRROR follower may fall behind the leader before the "
                         f"link stops, in radians (default {DEFAULT_MAX_GAP}). ⚠️ A TOLERANCE "
                         f"rather than a speed: past a certain leader speed the follower is "
                         f"tracking as hard as it can and still losing ground, because "
                         f"SafeRobot clips every command to 0.25 rad from the measured "
                         f"position. Loosening this lets the copy lag further behind rather "
                         f"than stopping; it does not make the follower faster")
    ap.add_argument("--cameras", default=None, metavar="SPEC[,SPEC]",
                    help="⭐ record camera frames while a take records (ROADMAP §8.2 item "
                         "48): comma- or space-separated specs — c920 · a raw index like 2 "
                         "· d405:<USB serial>. Cameras open at session start, in YOUR "
                         "terminal because macOS grants capture per app (FINDINGS §61.3); "
                         "frames land beside the recording under recordings/frames/<slot>/. "
                         "⛔ Refused with --sim, and never saved by --save-defaults")
    ap.add_argument("--fork-map", action="store_true",
                    help="give THIS arm its own axis map, copied from the one it uses now. "
                         "Without this, both arms share one map and editing changes both")
    ap.add_argument("--share-map", action="store_true",
                    help="drop this arm's own axis map and go back to the shared one")
    ap.add_argument("--save-defaults", action="store_true",
                    help="write this run's settings to config/session_defaults.json so "
                         "they apply to every later session without the flags. A flag "
                         "still overrides the file for one run.")

    # Settings precedence: built-ins < saved defaults < explicit command-line flags.
    # Install saved defaults after defining arguments and before parsing.
    settings_file = defaults_path(REPO)
    # ⚠️ `ap.get_default()` rather than a second `parse_args([])`: parsing an empty list
    # would run every validation the parser carries, and a future `required=` argument would
    # make reading the defaults exit the program.
    from yam.settings import TUNABLE as _TUNABLE
    builtin_defaults = {k: ap.get_default(k) for k in _TUNABLE}
    saved_defaults = load_defaults(settings_file)
    ignored_defaults = rejected_keys(settings_file)
    if saved_defaults:
        ap.set_defaults(**saved_defaults)

    args = ap.parse_args()
    if args.fork_map and args.share_map:
        ap.error("--fork-map and --share-map are opposites; pass at most one")
    cam_error = sim_camera_error(args.sim, args.cameras)
    if cam_error:
        ap.error(cam_error)

    # ⭐⭐ THE LIST OF ARMS THIS SESSION DRIVES. ROADMAP §6.1 step 2.
    #
    # ⚠️ `arm_names[0]` appears below wherever a line still assumes one arm, on purpose:
    # each one marks a site step 2's remaining work has to turn into a loop, and it is
    # greppable. Sites that run after the object exists use `arm.name` instead.
    try:
        arm_names = parse_arms(args.arm, args.arms, ARM_SERIALS, DEFAULT_ARM)
    except ValueError as exc:
        ap.error(str(exc))
    # Require HOLD or TELEOP when starting multiple arms.
    # GUIDE can then be selected deliberately for each supported arm.
    if len(arm_names) > 1 and args.start_mode == "guide":
        ap.error(
            f"--arms {','.join(arm_names)} --start-mode guide: refused. That would make "
            f"{len(arm_names)} arms weightless before anything is on screen, and GUIDE is "
            "the mode where an error in the dynamics model becomes a falling arm rather "
            "than a droop. Start in hold, then press g once you are watching.")

    # Rotation is enabled by default. The argument selects the initial mode;
    # subsequent mode changes belong to ArmSession.
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
    # Maps and saved poses are per arm. Slot 0 is the stable base used before disable;
    # waypoints 1-9 never change that shutdown destination.
    saved_slots = {name: park_slots(load_json(PARK_FILE, {}), name) for name in arm_names}
    # ArmSession owns path progress and park state; these indices select presentation options.
    blend_idx = 1                       # "smooth" — the sensible default
    ease_idx = 3                        # "both" — see motion.EASINGS
    # Park leg duration resets per waypoint; total path duration does not.
    # Recording and replay each use one shared timeline across every arm.
    recording = RecordingSession()
    playback = PlaybackSession()
    saving_summary: tuple[str, float, int] | None = None
    # Display the effective scrub rate over the last reporting window.
    scrub_ref_t = scrub_ref_s = scrub_ref_h = 0.0
    # Initialize overwrite-prompt state before the first save, even if no guard ran yet.
    replace_slot: str | None = None     # the occupied slot the guard is asking about
    # ⭐ Where the six live-editable settings stood when the session began, so `0` in the
    # SETTINGS screen can put them back. ⚠️ Taken AFTER the file and the flags have been
    # layered, so "how this session started" means what the plan printed, not the built-ins.
    settings_at_start = {k: getattr(args, k) for k in LIVE_ORDER if hasattr(args, k)}
    settings_pick: str = LIVE_ORDER[0]  # which setting -/+ moves in the SETTINGS screen
    # Capture each arrival's purpose before advancing a composite leg (FINDINGS §72.1).
    park_purpose: dict[str, str] = {}
    # Mirror state belongs to the session: the link relates a leader and follower.
    mirror_link: MirrorLink | None = None
    #: ⚠️ Declared here so the stop report can read it even if it somehow runs before a link
    #: was ever engaged. It is a count, so 0 is the honest starting value.
    mirror_clipped_at = 0
    mirror_leader: ArmSession | None = None
    mirror_follower: ArmSession | None = None
    # A pending `s` or `p` waiting for its digit, and the sequence being typed after `p`.
    pending: str | None = None
    park_sequence: list[str] = []      # "3" = pose slot, "w3" = recording (take) slot
    park_take_next = False             # w inside the p prompt arms "the next digit is a take"
    angular_scale = ANGULAR_SCALE
    gripper_step = args.gripper_step
    # The active control and button binding belong to the selected ArmSession.

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
    # Show the effective cap below mode-specific limits so the operator can see what binds.
    raised = []
    if args.max_speed != SAFE_MAX_SPEED:
        raised.append("--max-speed")
    if args.teleop_speed != MAX_PLANNED_JOINT_SPEED:
        raised.append("--teleop-speed")
    note = f"  ⚠️ RAISED: {', '.join(raised)}" if raised else ""
    print(f"  joint speed : teleop {min(args.teleop_speed, args.max_speed):.2f} · "
          f"planned {min(args.teleop_speed, args.max_speed):.2f} · "
          f"mirror {args.max_speed:.2f} rad/s{note}")
    if args.mirror_catchup > 0.0:
        # ⭐ Say it in the plan, because it changes what the follower does and a control term
        # nobody can see on startup is one nobody can rule out later.
        print(f"  mirror fix  : ON at {args.mirror_catchup:g}/s — corrects the follower's "
              f"standing offset while the leader is slow, clamped to 0.06 rad")
    if args.vel_ff > 0.0:
        # ⭐ Same rule as mirror-catchup: a control term that changes what the arm does is
        # named in the plan, so it can be ruled out (or blamed) later.
        shown_ff = min(args.vel_ff, VEL_FF_CEILING)
        capped = "" if args.vel_ff <= VEL_FF_CEILING else \
            f"  ⚠️ capped from {args.vel_ff:g}: above 1 is a measured dead end " \
            f"(FINDINGS §68.6)"
        print(f"  feedforward : ON at {shown_ff:g} — motors also receive that "
              f"fraction of the command's own speed, so torque starts before error builds "
              f"(item 44; jaw excluded){capped}")
    lag_note = "" if args.max_lag == SAFE_MAX_LAG else "  ⚠️ RAISED"
    # ⭐⭐ THE PLAN NOW SAYS WHAT EACH LIMIT DOES, NOT JUST ITS VALUE. Julien, 2026-08-17:
    # *"I want to understand what MaxLag exactly does. I think I understand Max speed… but then
    # what does MaxLag mean?"* ⚠️ The line above already printed both numbers and neither
    # meaning, which is the same fault as printing a limit's value without its flag name.
    print(f"                (SafeRobot caps everything at {args.max_speed:.2f} rad/s AND holds "
          f"the command within {args.max_lag:.2f} rad of the measured pose{lag_note})")
    print(f"  what limits  : ⭐ FOUR limits in series, and the SMALLEST one binds:")
    print(f"                 linear {args.linear_scale:.2f} m/s  how fast a full puck push "
          f"asks the TIP to move  (- / + or setting 8)")
    print(f"                 teleop {args.teleop_speed:.2f} rad/s  how far the IK answer may "
          f"move any ONE JOINT per cycle")
    print(f"                 max-speed {args.max_speed:.2f} rad/s  the same cap again, below "
          f"all control logic, so nothing can reach around it")
    print(f"                 max-lag {args.max_lag:.2f} rad  ⭐ how far the COMMAND may run "
          f"ahead of where the arm actually IS.")
    print(f"                   ⚠️ This is not a speed. The command is pulled back to "
          f"measured+{args.max_lag:.2f} every cycle, so")
    print(f"                   reaching a far target becomes a ratchet: the arm moves, the "
          f"command advances, repeat. A")
    print(f"                   BLOCKED joint therefore never gets there, and that is the "
          f"point — it bounds the push.")
    print(f"  temperature : warn {TEMP_WARN}°C, stop {TEMP_STOP}°C")
    # ⭐⭐ SAY WHICH SETTINGS CAME FROM THE FILE, AND FLAG A PERMANENT LOOSENING. A flag
    # typed on the command line is visible in the shell history and on screen; a saved
    # default is not. ⛔ Without these lines a session could run at three times the built-in
    # speed limit with nothing on screen explaining why.
    for line in describe_defaults(saved_defaults, ignored_defaults,
                                  looser_than_builtin(saved_defaults, builtin_defaults),
                                  settings_file, builtin_defaults):
        print(line)
    # ⭐ Saved HERE, after the plan has printed the values, so what lands in the file is
    # exactly what was just shown. ⚠️ Before anything is energised, so a session that is
    # about to be abandoned still records the settings that were being aimed for.
    if args.save_defaults:
        save_defaults(settings_file, effective_settings(args))
        print(f"\n⭐ SAVED these settings to "
              f"{settings_file.parent.name}/{settings_file.name}. Later sessions pick them "
              f"up with no flags at all.\n"
              f"   ⚠️ A flag still overrides the file for one run. Delete the file to go "
              f"back to the built-in constants.")
    print(HELP)

    if not args.yes:
        print("DRY RUN — nothing transmitted, nothing energised. Re-run with --yes.")
        return 0

    # ⭐ Cameras open FIRST, before the puck wiggle and before anything can energise: a camera refusal here costs nothing, while the same refusal after the wiggle would waste the operator's assignment gesture. `capture` outlives every take (opening costs seconds per device); the per-take writers below are the cheap part.
    capture: CaptureSet | None = None
    pucks: dict[str, Any] = {}
    arms: list[ArmSession] = []
    # Own a robot as soon as its builder returns, before wrapping or reading it.
    # A failed ArmSession constructor must not leave an enabled handle unowned.
    robots: dict[str, Any] = {}
    stop_reason: str | None = None
    exit_code = 0
    previous_sigint = signal.getsignal(signal.SIGINT)
    previous_thread_hook = threading.excepthook
    threading.excepthook = _quiet_expected_server_exit
    _SHUTTING_DOWN["yes"] = False
    try:
        capture_names: list[str] = []
        if args.cameras:
            # ⚠️ Platform-aware since FINDINGS §75.10: this line claimed a macOS permission on the
            # Linux station, where the gate is group membership instead. Two different facts, and
            # printing the wrong one teaches the reader a wrong thing about their own machine.
            from yam.platform import IS_LINUX  # noqa: PLC0415

            print("opening cameras "
                  + ("(this needs the `video` group, which this user has):" if IS_LINUX
                     else "(this holds the macOS capture permission of THIS terminal):"))
            capture, capture_names = open_session_cameras(args.cameras)

        # Assign one distinct puck per driven arm. Open inputs before enabling motors.
        # Motion permissions and device ownership remain separate.
        if args.sim:
            # Simulation uses still pucks. Keys exercise mode, pose, recording and replay paths.
            # It does not synthesize hand motion or read physical SpaceMice.
            for name in arm_names:
                pucks[name] = {"path": f"sim:{name}", "handle": None,
                               "reader": StillPuck()}
            print("⭐ SIMULATED PUCKS — both report zero deflection, so TELEOP holds still.\n"
                  "   Drive the loop with the KEYS: modes, p, w, l, a, i, q.\n")
        for name in arm_names if not args.sim else []:
            # A puckless follower still needs its robot for mirror operation.
            # It uses a still reader and must be explicitly named by the operator.
            already = {h["path"] for h in pucks.values()}
            if pucks and not [d for d in find_all_devices() if d.get("path") not in already]:
                pucks[name] = {"path": f"none:{name}", "handle": None, "reader": StillPuck()}
                print(f"\n⚠️  arm {name} has NO puck — every attached SpaceMouse is already "
                      f"assigned.\n   It still joins the session: HOLD, GUIDE, playback, "
                      f"scrub and MIRROR-follower\n   all work. Only its own TELEOP is dead "
                      f"(its puck reads zero deflection).\n   Attach a second SpaceMouse and "
                      f"restart to give it one.\n")
                continue
            info = pick_device_by_wiggle(label=name,
                                         exclude=[h["path"] for h in pucks.values()])
            if info is None:
                print(f"No SpaceMouse found for arm {name} (or none was moved).")
                return 1
            countdown_hands_off(3)
            handle = open_device(info)
            pucks[name] = {"path": info["path"], "handle": handle}
            handle.set_nonblocking(True)
            pucks[name]["reader"] = TwistReader(handle)

        # Initialized ArmSessions drive the loop. The separate robots registry owns every
        # acquired handle for cleanup, including a handle whose session failed to initialize.
        # Summary fields must also exist if startup fails before the first cycle.
        _real_pucks = [p for p in pucks.values()
                       if not isinstance(p.get("reader"), StillPuck)]
        shared_puck = (_real_pucks[0]["reader"]
                       if len(arm_names) > 1 and len(_real_pucks) == 1 else None)
        if shared_puck is not None:
            print("⭐ ONE puck for the whole session — it FOLLOWS THE SELECTION: press a to")
            print("   aim it (B → G → BOTH). BOTH drives both arms at once, each from its")
            print("   own pose. Unaimed arms read a centred puck and hold their position.\n")
        arm: ArmSession | None = None
        # Initialize summaries before acquisition so partial startup can still be reported.
        start_mode = args.start_mode
        # ArmSession initializes its solver and thermal state. Local summary state must
        # remain available on failures that happen before the control loop.
        next_park_report = 0.0
        # ⚠️ `gripper_value` and `stall_since` used to be initialised here. They are now
        # `ArmSession` fields, and the class's own constructor sets exactly the same values
        # (0.0 and None). ⛔ Leaving the assignments here as `arm.gripper_value = 0.0` would
        # run BEFORE `arm` exists, which is nine lines below inside the `try`. See the
        # ordering check in checks/check_restructure.py.

        # ⭐⭐ SIMULATED RECORDINGS GO SOMEWHERE ELSE. Defence in depth alongside the
        # metadata stamp: `recordings/sim/` never contains a real demonstration, so a glob
        # over `recordings/*.json` cannot pick one up by accident.
        takes_dir = (TAKES_DIR / "sim") if args.sim else TAKES_DIR
        tracking_dir = takes_dir / "tracking"

        def slot_for_reading(digit: str) -> Path:
            """Where to LOOK for a recording. Writes always go to `takes_dir`.

            ⭐⭐ A --sim SESSION CAN STILL PLAY A REAL RECORDING, and that is deliberate. When
            the folder split was first written it applied to reads as well, which quietly
            removed one of the best uses of a simulator: **replaying a real take against
            simulated arms to check the playback before committing it to 4.3 kg of hardware.**
            Sim recordings win when both exist, so a sim session never silently reaches past
            its own work.
            """
            mine = takes_dir / f"{digit}.json"
            if mine.is_file() or takes_dir == TAKES_DIR:
                return mine
            return TAKES_DIR / f"{digit}.json"
        if args.sim:
            print(f"⭐ SIMULATED recordings go to {takes_dir.relative_to(REPO)}/, never "
                  f"alongside the real ones, and each is stamped simulated=true.\n")

        n_motors = N_ARM if args.no_gripper else N_ARM + 1
        # Build arms in the requested layout order. Register each returned handle
        # before wrapper configuration or initial reads can fail.
        for name in arm_names:
            print(f"building arm {name} — enables {n_motors} motors, "
                  "starts the control loop …")
            if args.sim:
                # ⭐⭐ THE WHOLE POINT OF --sim, AND IT IS ONE BRANCH ON PURPOSE. Everything
                # below this line is the same code in both modes, so a simulated session
                # exercises the real loop rather than a parallel one. `build_fake_robot`
                # returns the same `(robot, note)` tuple for exactly that reason.
                robot, note = build_fake_robot(
                    name, n_joints=n_motors,
                    max_speed=args.max_speed, max_lag=args.max_lag)
            else:
                robot, note = build_robot(name, zero_gravity=(start_mode == "guide"),
                                          with_gripper=not args.no_gripper,
                                          max_speed=args.max_speed, max_lag=args.max_lag)
            # ⭐ On the SafeRobot in both modes, so a simulated session exercises the same
            # feedforward plumbing the arm gets (the fake records the setpoints, item 44).
            robots[name] = robot
            robot.vel_ff = args.vel_ff
            print(f"  {note}\n")

            arm = ArmSession(robot, name=name, frame=start_frame,
                             gripper_min=GRIPPER_MIN, gripper_max=GRIPPER_MAX,
                             warn_at=TEMP_WARN, stop_at=TEMP_STOP,
                             axis_map=map_store.for_arm(name, start_frame),
                             slots=saved_slots[name], base_slot=BASE_SLOT,
                             reader=pucks[name]["reader"])
            # Transfer the mode used to build the robot into its session state.
            arm.mode = start_mode
            arm.prev_q = np.asarray(arm.robot.get_joint_pos(), dtype=float)[:N_ARM]
            arms.append(arm)

            # Without a saved base, use this arm's measured startup pose and say so.
            # Do not borrow the other arm's base pose.
            if arm.base_pose is None:
                arm.base_pose = np.asarray(arm.robot.get_joint_pos(), dtype=float).tolist()
                print(f"  park pose {name} : none saved — defaulting to the pose the arm is "
                      f"in NOW, {np.round(np.asarray(arm.base_pose)[:N_ARM], 3).tolist()}")
                print("                (press s to set a different arm; q then p then d "
                      "parks and quits)")

        # ⛔ Mode keys are AIMED; driving never is. A global `g` would put 8.6 kg weightless
        # in arm keypress, and GUIDE is where a dynamics-model error becomes a falling arm
        # rather than a droop (FINDINGS §11.1). Each arm always follows its own puck.
        # `src/yam/session.py::ArmSelector` holds the cycle and its tests.
        selection = ArmSelector(arm_names)

        # build_robot verifies the frame-corrected jaw range before returning.
        # Do not compare raw jaw readings to unshifted saved limits here: that can
        # misdiagnose the valid ±2π correction and suggest unnecessary recalibration.

        def clamp_gripper(v: float) -> float:
            return float(np.clip(v, GRIPPER_MIN, GRIPPER_MAX))

        # Per-arm operations receive their target explicitly. The common jaw clamp
        # is independent of arm selection; ArmSession performs mode-transition resync.

        def make_teleop(frame: str) -> CartesianTeleop:
            """⭐ Injected into `ArmSession.enter_teleop` (item 23 group ③): the class
            stays testable because it never constructs the IK itself, and the frame is
            the ARM's own, so two arms can be driven in different frames at once."""
            return CartesianTeleop(frame=frame)

        # ⭐ enter_hold lives on ArmSession now (item 23 group ①, 2026-08-18): the class
        # method resyncs, commands the measured pose AND sets mode="hold" — so the two
        # sites that want a DIFFERENT mode afterwards (the park seed, the mirror engage)
        # write their mode AFTER the call. The script's own copy is gone.

        # ⭐ enter_guide lives on ArmSession now (item 23 group ②, 2026-08-18). The class
        # method records guide_ref, sets mode="guide" and RETURNS the "NOT weightless"
        # warning instead of printing it — every caller prints the return, so the
        # warning that once explained a falling arm (FINDINGS §11) cannot be dropped.
        # The kp=0 physics and the API-name history live in the class docstring.

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

        def park_seq_shown() -> str:
            """The typed sequence as the operator should read it: `1 -> >2 -> 3`."""
            return " → ".join(("▶" + e[1:]) if e.startswith("w") else e
                              for e in park_sequence)

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
            seq = park_seq_shown() if park_sequence else "0"
            name, radius = BLEND_MODES[blend_idx]
            # ⭐ A grab is visible BEFORE Enter (ROADMAP §6.6.2 item 4): a leg where only
            # the jaws move splits the run and pauses it, and the count says so here,
            # while the sequence is still being typed.
            # ⚠️ Take legs (`w<digit>`, ROADMAP §6.6.1a) are counted separately: their
            # jaw motion is whatever the hand taught, so no stop-counting applies.
            poses = [e for e in (park_sequence[:] or ["0"]) if not e.startswith("w")]
            takes = [e[1:] for e in park_sequence if e.startswith("w")]
            if poses:
                legs, _ = resolve_park_legs(poses, one.base_pose, one.slots)
            else:
                legs = []
            stops = one.count_gripper_stops([ParkLeg(n, list(p)) for n, p in legs]) if legs else 0
            stop_note = f" · ⏸ {stops} jaw stop{'s' if stops != 1 else ''}" if stops else ""
            if takes:
                stop_note += " · ▶ play " + ", ".join(takes)
            # ⭐ ONE line, so changing a knob repaints instead of appending. Six taps
            # on `+` should leave one line showing the final speed, not six blocks.
            return (f"RUN {seq}{stop_note} · speed {one.park_speed:.2f} (-/+) · corners {name} "
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
            if playback.pending is None:
                return ""
            taught = playback.pending.joint_speed(99)
            note = ""
            if taught > args.teleop_speed:
                note = (f" ⚠️ taught {taught:.1f} rad/s exceeds the "
                        f"{args.teleop_speed:.1f} allowed, so 1.00x will lag")
            return (f"PLAY {playback.slot} · {playback.pending.duration:.1f}s taught at "
                    f"{taught:.2f} rad/s · speed {playback.speed:.2f}x (-/+){note} · Enter=go "
                    f"· j=puck scrub")

        def begin_path(one: ArmSession, legs: list, what: str,
                       for_replay: bool = False, for_composite: bool = False) -> None:
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
            # An unrelated park cancels pending playback. Only replay's own park-to-start
            # may authorize that playback to begin.
            if not for_replay and playback.pending is not None:
                playback.cancel_pending()
                print("\n  ⚠️  playback cancelled — a new park replaced the drive to its "
                      "start pose.\n")
            # ⭐ Same rule one level up (ROADMAP §6.6.1a trap ①): a park the COMPOSITE did
            # not start replaces the composite. Its own pose legs pass for_composite=True
            # and its take legs pass for_replay=True, so only operator-initiated parks
            # land here — which is exactly who may abandon a queued run.
            if not for_replay and not for_composite:
                composite.abandon("a new park replaced it")
            # ⭐ item 23 group ④: the CLASS builds and runs the park now — the tested
            # `begin_path`/`step_path` pair with its 48 tests is finally the code that
            # moves the arm. The session's live dials are copied on at start, and the
            # `e` key keeps `easing` current mid-park.
            one.blend = BLEND_MODES[blend_idx][1]
            one.easing = EASINGS[ease_idx]
            park_purpose[one.name] = ("replay" if for_replay
                                      else "composite" if for_composite else "operator")
            for warn in one.begin_path([ParkLeg(n, list(pose)) for n, pose in legs], t,
                                       smooth=not args.no_smooth,
                                       mixed_leg_advice=not for_replay):
                print(f"\n  ⚠️  {warn}.")
            # The plan has become the thing happening; the progress readout replaces it.
            hint("")
            # ⭐ Total travel counts every queued segment, and a run that will pause for
            # the jaws says so up front — the pause must never read as a stall.
            stop_note = (f", pausing {one.park_stops}× for the jaws"
                         if one.park_stops else "")
            print(f"\n⭐ MODE: PARK → {what}, {one.park_total_length:.2f} rad of travel at "
                  f"{one.park_speed:.2f} rad/s, corners {BLEND_MODES[blend_idx][0]}"
                  f"{stop_note}. Press h or t to stop.\n")

        def load_take(slot: str):  # noqa: ANN202
            """Load recording `slot` and check it fits this session. None = refused, said why.

            ⭐ THE LAYOUT COMES FROM THE FILE. `Layout.from_meta` also reads recordings
            made before two arms existed — Julien has several in slots 1-6 — so they stay
            playable on the arm they name. Shared by the `l` prompt and by composite runs,
            which validate EVERY take at Enter, before any motion.
            """
            path = slot_for_reading(slot)
            if not path.is_file():
                print(f"\n  ⚠️  nothing saved in recording {slot} — press w to record one.\n")
                return None
            loaded = Trajectory.load(path)
            layout = Layout.from_meta(loaded.meta, loaded.n_joints)
            by_name = {one.name: one for one in arms}
            missing = [n for n in layout.arms if n not in by_name]
            if missing:
                print(f"\n  ⚠️  recording {slot} was made with arm(s) "
                      f"{', '.join(layout.arms)} and this session has "
                      f"{', '.join(by_name)}. Missing: {', '.join(missing)}.")
                print("     Start the session with those arms to play it.\n")
                return None
            if layout.n_joints != loaded.n_joints:
                print(f"\n  ⚠️  recording {slot} says {layout.n_joints} joints in its "
                      f"metadata and holds {loaded.n_joints}. Refusing rather "
                      "than guessing which is right.\n")
                return None
            return loaded, layout, [by_name[n] for n in layout.arms]

        def park_to_take_start() -> None:
            """Every replay arm parks to its own slice of the pending take's start pose.

            ⛔⭐ THIS IS THE SAFETY POINT OF PLAYBACK. A recording commands poses a hand
            physically put the arm in, so the only dangerous command is the FIRST one:
            if the arm is somewhere else right now, the recording's opening pose is a
            jump across whatever separates them. The existing, tested, interruptible
            park drives there, and the ARRIVAL branch — never an interruption — hands
            over to the playback (FINDINGS §57.1).
            """
            start = list(playback.pending.start_pose() or ())
            for one in playback.arms:
                begin_path(one, [("recording start",
                                  start[playback.layout.slice_for(one.name)])],
                           f"arm {one.name}'s start pose in recording "
                           f"{playback.slot}", for_replay=True)

        def start_take(slot: str, loaded, layout, take_arms) -> None:  # noqa: ANN001
            """A composite take-leg begins: exactly the `l` Enter flow, minus the prompt."""
            playback.prepare(loaded, layout, take_arms, slot, args.teleop_speed)
            park_to_take_start()

        composite = CompositeRun(load_take=load_take, begin_path=begin_path,
                                 start_take=start_take, emit=print,
                                 clear_hint=lambda: hint(""))

        # ⭐ Each arm enters its start mode, per arm. It used to run once, after the single
        # build, reading the one `robot` local.
        for one in arms:
            if one.mode == "teleop":
                one.enter_teleop(make_teleop)
            elif one.mode == "hold":
                one.enter_hold()
            elif one.mode == "guide":
                # ⚠️ GUIDE at startup is established by build_robot(zero_gravity=True), not
                # by enter_guide() — so the drift reference has to be taken here too, or the
                # readout silently shows nothing for the whole first GUIDE period. That gap
                # is exactly the 33 seconds in which the arm sank unremarked on 2026-08-10.
                one.guide_ref = np.asarray(one.robot.get_joint_pos(), dtype=float)

        dt = 1.0 / CONTROL_HZ
        # Use measured cycle time for time-based motion and logs. Keep the nominal
        # period only where an API explicitly expects the configured rate.
        real_dt = dt
        prev_t = 0.0
        loop_hz = CONTROL_HZ
        # ⭐⭐ THE WORST SINGLE PASS, which `loop_hz` cannot show because it is an average with a 0.02 time constant. [PERFORMANCE.md](../docs/PERFORMANCE.md) §2 named this as the one missing measurement and [FINDINGS §77.4](../docs/FINDINGS.md) as the one worth taking next: jitter is what moving the cameras to their own process would fix, and nobody has ever looked at it. It takes the RAW interval, not the clamped `real_dt` below, because that clamp is what would hide a stall.
        loop_timer = LoopTimer(CONTROL_HZ)
        t0 = time.perf_counter()
        next_report = 1.0

        with KeyReader() as keys:
            if not keys.enabled:
                print("⚠️  stdin is not a terminal — keys will not work. Ctrl-C still does.\n")
            print("⭐ MODE: "
                  + " · ".join(f"{one.name} {one.mode.upper()}" for one in arms) + "\n")

            # The SIGINT handler requests a controlled stop; it never disables motors itself.
            # The first request attempts parking through the stop flow; another interrupts it.
            interrupted: list[bool] = []

            def on_sigint(signum, frame) -> None:  # noqa: ANN001, ARG001
                signal.signal(signal.SIGINT, signal.SIG_DFL)
                interrupted.append(True)

            signal.signal(signal.SIGINT, on_sigint)

            while True:
                loop_start = time.perf_counter()
                t = loop_start - t0
                raw_dt = t - prev_t
                # ⛔ Recorded at the TOP of the pass, before anything can `break` out of the body, so no pass goes uncounted. The first sample is the interval from the loop's own zero point and `LoopTimer` discards it.
                loop_timer.record(raw_dt, at_s=t)
                real_dt = min(0.1, max(1e-4, raw_dt))
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
                        # ⭐⭐ SAY IT ONCE WHEN THE LATCH LETS GO. His 2026-08-17 log showed
                        # three stalls at 0.117, 0.098 and 0.104 and there was **no way to
                        # tell** whether those were three deliberate squeezes or one latch
                        # being cleared twice by a jittering measurement. A latch that
                        # silently comes and goes is indistinguishable from one that never
                        # worked, so the next run must not leave the same ambiguity.
                        if one.jaw_unblocked_from is not None:
                            print(f"\n  ⭐ arm {one.name} jaws opened past the block at "
                                  f"{one.jaw_unblocked_from:.3f} — free to close again.\n")
                            one.jaw_unblocked_from = None
                        jaw = one.states[N_ARM] if len(one.states) > N_ARM else None
                        if jaw is None:
                            one.stall_since = None
                        elif one.jaw_block is not None:
                            # A latched jaw remains at its captured position. Opening beyond the latch
                            # margin releases it; repeated close commands must not resume stall torque.
                            one.stall_since = None
                        elif (abs(getattr(jaw, "eff", 0.0)) > GRIPPER_STALL_TORQUE
                                and abs(getattr(jaw, "vel", 0.0)) < GRIPPER_STALL_VEL):
                            if one.stall_since is None:
                                one.stall_since = loop_start
                            elif loop_start - one.stall_since > GRIPPER_STALL_SECONDS:
                                measured_jaw = float(np.asarray(
                                    one.robot.get_joint_pos(), dtype=float)[N_ARM])
                                one.gripper_value = measured_jaw
                                # Latch at the measured jaw position so later teleop updates cannot reapply the stall.
                                one.block_jaw_at(measured_jaw)
                                one.stall_since = None
                                one.stall_count += 1
                                # Report a newly detected stall once; throttle repeated notifications with a count.
                                if (one.stall_count == 1
                                        or loop_start - one.stall_last_said > 5.0):
                                    one.stall_last_said = loop_start
                                    extra = ("" if one.stall_count == 1 else
                                             f" ({one.stall_count} times now)")
                                    print(f"\n⚠️  ARM {one.name} GRIPPER STALLED{extra} "
                                          f"({jaw.eff:+.2f} Nm, not moving) — released to "
                                          f"{measured_jaw:.3f} so it stops pushing.")
                                    if one.stall_count > 1:
                                        print("     Something is holding the jaws and the "
                                              "command keeps pushing past it. In MIRROR that "
                                              "is the leader's jaws being squeezed while the "
                                              "follower already has hold of something.\n")
                                    else:
                                        print()
                        else:
                            one.stall_since = None
                            # ⭐ Reset the streak once the jaws are free again, so the count
                            # means "in a row" rather than "since the session began".
                            one.stall_count = 0

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

                had_finishing_sink = recording.active is None and recording.sink is not None
                recording.poll()
                if had_finishing_sink and recording.sink is None:
                    if recording.frame_error:
                        print(f"\n  {recording.frame_error}. Frames retained at {recording.frames}.\n")
                    elif recording.frame_report is not None:
                        for camera, report in recording.frame_report["per_camera"].items():
                            print(f"  📷 {camera}: {report['written']} frame(s), "
                                  f"{report['dropped']} dropped, {report['write_errors']} write error(s).")
                        print("  Camera files finished. Choose a save slot or discard.\n")
                if pending in ("take_saving", "take_discarding") and not recording.busy:
                    if recording.save_error is not None:
                        print(f"\n  Recording disk operation failed: {recording.save_error}")
                        for note in getattr(recording.save_error, "__notes__", []):
                            print(f"     {note}")
                        print("     The new take is still pending. Choose a slot to retry, "
                              "or a non-digit to discard.\n")
                        pending, replace_slot = "take_save", None
                    elif pending == "take_saving" and recording.saved is not None:
                        slot, seconds, count = saving_summary
                        saved = recording.saved
                        if saved.warning:
                            print(f"  {saved.warning}")
                        print(f"\n  ✓ recording {slot} saved: {seconds:.1f}s, "
                              f"{count} samples → {saved.path.name}"
                              + (f" + frames/{slot}/" if saved.has_frames else ""))
                        print(f"     (l then {slot} plays it back)\n")
                        pending = None
                    else:
                        print("\n  recording discarded.\n")
                        pending = None

                # ---- 3. keys ----------------------------------------------
                for k in keys.drain():
                    if recording.busy and pending in ("take_save", "take_replace", "take_saving", "take_discarding"):
                        if k == "q":
                            # Quitting still reaches the normal park/disable flow.
                            pending = None
                        else:
                            print("  Recording files are still finishing. Wait, or q to quit; "
                                  "unfinished files are retained.")
                            continue
                    # Selection controls mode changes and edits; each arm continues being driven.
                    # A pending wizard keeps its original target until finished or cancelled.
                    aimed = [one for one in arms if one.name in selection.names()]
                    edit_arm = aimed[0]
                    wizard = next((one for one in arms if one.mode == "map"), None)
                    aimed_label = "+".join(one.name for one in aimed)
                    # A pending pose/save prompt consumes its next key as an argument.
                    if pending == "save":
                        pending = None
                        if k.isdigit():
                            # Update all selected arms in one pose-file write; slot 0 changes their bases.
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

                    if pending in ("take_save", "take_replace"):
                        # An occupied slot requires the same digit twice. A different digit re-aims;
                        # a non-digit deliberately discards the new take. Keep the old slot until save commits.
                        was_replacing = pending == "take_replace"
                        pending = None
                        occupied = (describe_slot(takes_dir / f"{k}.json")
                                    if k.isdigit() else None)
                        action = save_slot_action(k, recording.pending is not None,
                                                  was_replacing, replace_slot, occupied)
                        if action[0] == "ask":
                            replace_slot = action[1]
                            pending = "take_replace"
                            if action[2]:
                                print(f"\n  ⭐ aiming at recording {replace_slot} instead.")
                            print(f"\n  ⚠️  recording {replace_slot} already holds "
                                  f"{occupied}.")
                            print(f"     Press {replace_slot} again to REPLACE it, or "
                                  f"another digit to aim somewhere else.")
                            print("     Any NON-digit discards the new recording and keeps "
                                  "what is there.\n")
                            continue
                        if action[0] == "discard":
                            recording.request_discard()
                            pending = "take_discarding"
                            if action[1] is not None:
                                print(f"\n  kept recording {action[1]}; the new one is "
                                      "being discarded.\n")
                            else:
                                print("\n  finishing recording discard.\n")
                            continue
                        if k.isdigit() and recording.pending is not None:
                            try:
                                saving_summary = (k, recording.pending.duration, len(recording.pending))
                                recording.request_save(takes_dir, k, saver=save_take)
                                pending = "take_saving"
                                print(f"\n  Saving recording {k}… control remains active.\n")
                            except Exception as exc:
                                pending = "take_save"
                                replace_slot = None
                                print(f"\n  Recording was not saved: {exc}")
                                print("     The new take is still pending. Choose a slot to retry, "
                                      "or a non-digit to discard.\n")
                        else:
                            recording.request_discard()
                            pending = "take_discarding"
                        continue

                    if pending == "settings":
                        # ⛔⭐⭐ max_speed AND max_lag ARE PUSHED ONTO THE LIVE ROBOTS. They
                        # are `SafeRobot` attributes read on every command, so assigning them
                        # here changes what bounds 4.3 kg **on the next cycle**. That is the
                        # point of a live editor, and it is also why this screen says so.
                        def apply_live(name: str, value: float) -> None:
                            setattr(args, name, value)
                            for one_arm in arms:
                                if name == "max_speed":
                                    one_arm.robot.max_speed = value
                                elif name == "max_lag":
                                    one_arm.robot.max_lag = value
                                elif name == "vel_ff":
                                    one_arm.robot.vel_ff = value
                            if name == "mirror_gap" and mirror_link is not None:
                                mirror_link.max_gap = value
                            if name == "mirror_catchup" and mirror_link is not None:
                                # ⭐ Reaches a RUNNING mirror, so he can watch the follower
                                # close onto the leader while holding it still. That is the
                                # whole reason this belongs on the live screen.
                                mirror_link.catchup = value

                        # Arrow keys move the settings selection; they do not modify the selected value.
                        show_all = k == "?"
                        if k in ("\x1b[A", "\x1b[B"):
                            step = -1 if k == "\x1b[A" else 1
                            here = LIVE_ORDER.index(settings_pick)
                            settings_pick = LIVE_ORDER[(here + step) % len(LIVE_ORDER)]
                            print(setting_line(settings_pick,
                                               float(getattr(args, settings_pick)),
                                               builtin=builtin_defaults))
                            continue
                        elif k == "n":
                            # ⭐ `n` CLOSES it, the way `i` toggles mirror. He pressed n inside
                            # the screen and was told it does nothing; toggling is the obvious
                            # meaning of pressing the key that opened something.
                            pending = None
                            print("\n  ⭐ SETTINGS closed. The values are live; press n then s "
                                  "to write them to the file.\n")
                            continue
                        elif k in "123456789":
                            idx = int(k) - 1
                            if idx < len(LIVE_ORDER):
                                settings_pick = LIVE_ORDER[idx]
                            print(setting_line(settings_pick,
                                               float(getattr(args, settings_pick)),
                                               builtin=builtin_defaults))
                            continue
                        elif k in "+=" or k == "-":
                            was = float(getattr(args, settings_pick))
                            apply_live(settings_pick,
                                       adjust_setting(settings_pick, was, k != "-"))
                            print(setting_line(settings_pick,
                                               float(getattr(args, settings_pick)),
                                               before=was, builtin=builtin_defaults))
                            # ⭐⭐ THE ARM'S LIVE ROW PRINTS UNDER EVERY CHANGE (item 43).
                            # He tuned mirror_catchup with 33 presses and vel_ff blind on
                            # 2026-08-18, because this screen covered the one row showing
                            # the effect. A live editor whose effect is invisible while
                            # editing is half a feature (FINDINGS §65.4).
                            for one_arm in arms:
                                print("     " + status_row(
                                    one_arm, "", args.reach, args.floor,
                                    note=(mirror_link.status(
                                        mirror_leader.robot.get_joint_pos(),
                                        one_arm.robot.get_joint_pos())
                                        if mirror_link is not None
                                        and one_arm.mode == "mirror" else "")))
                            continue
                        elif k == "0":
                            for name, value in settings_at_start.items():
                                apply_live(name, value)
                            print("\n  ⭐ back to the values this session started with.\n")
                        elif k == "s":
                            save_defaults(settings_file, effective_settings(args))
                            print(f"\n  ⭐ SAVED to {settings_file.parent.name}/"
                                  f"{settings_file.name}. Every later session starts with "
                                  f"these.\n")
                            pending = None
                            continue
                        elif k in MODE_KEYS or k in ("\r", "\n", " "):
                            pending = None
                            # ⚠️ A mode key LEAVES rather than also switching mode. Pushing
                            # it back onto the reader would need a queue the key reader does
                            # not have, and silently changing mode on the way out of a
                            # settings screen is the kind of surprise that moves an arm.
                            print("\n  ⭐ leaving SETTINGS. The values are live; nothing was "
                                  "written to the file.\n     Press n then s to make them "
                                  "permanent. Press the mode key again to change mode.\n")
                            continue
                        elif k == "q":
                            # ⭐ He pressed q HERE twice on 2026-08-18, wanting to quit, and
                            # got "(does nothing here)" both times (FINDINGS §67.10). The
                            # intent is unambiguous: close the screen and hand the key's
                            # meaning to the session's own quit flow, which holds every arm
                            # and asks before anything is released.
                            pending = None
                            print("\n  ⭐ SETTINGS closed — over to the quit menu. The values "
                                  "stay live; they were not saved.\n")
                            stop_reason = "quit requested"
                            continue
                        elif k != "?":
                            # ⚠️ Escape sequences are NAMED rather than echoed. `\x1b[C` on
                            # screen is noise; "left/right arrow" is information.
                            shown = {"\x1b[C": "right arrow", "\x1b[D": "left arrow"}.get(
                                k, repr(k) if k.isprintable() else "that key")
                            print(f"\n  ({shown} does nothing here — 1-9 or up/down to pick, "
                                  f"-/+ to change, 0 revert, s save, q quit, "
                                  f"n or t/g/h to leave)\n")
                        if show_all or k == "0":
                            for line in live_lines(
                                    {kk: getattr(args, kk) for kk in LIVE_ORDER},
                                    settings_pick, builtin_defaults):
                                print(line)
                        continue

                    if pending == "take_play":
                        pending = None
                        if not k.isdigit():
                            print("\n  play cancelled.\n")
                            continue
                        got = load_take(k)   # validation shared with composite runs
                        if got is None:
                            continue
                        loaded, layout, take_arms = got
                        playback.prepare(loaded, layout, take_arms, k, args.teleop_speed)
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
                                playback.pending.joint_speed(99), args.teleop_speed))
                            playback.speed = min(ceiling, playback.speed * 1.25)
                            hint(replay_plan_line()); continue
                        if k == "-":
                            playback.speed = max(0.05, playback.speed / 1.25)
                            hint(replay_plan_line()); continue
                        pending = None
                        if k == "j" and playback.pending is not None:
                            # ⭐ SCRUB (item 13): the same park-to-start safety flow as
                            # Enter, but once the recording begins the PUCK is the clock.
                            # A mode entered on purpose, never the default (ROADMAP §7.6's
                            # design caution: a long unattended playback must not need a
                            # held hand — this one is FOR the hand).
                            playback.scrub = True
                            park_to_take_start()
                            print("     then the PUCK scrubs it: push forward to play, pull "
                                  "back to rewind,\n     let go to freeze. h or t ends it.\n")
                            continue
                        if k in ("\r", "\n", " ") and playback.pending is not None:
                            playback.scrub = False
                            # Validate the take, park every participating arm to its first sample, then
                            # require the arrival gate before advancing the shared replay cursor.
                            park_to_take_start()
                            print("     then it plays the recording. Press h or t to stop.\n")
                        else:
                            playback.cancel_pending()
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
                            # ⛔ ORDER: the class's enter_hold() sets mode="hold",
                            # so MIRROR is written AFTER it — the reverse order would
                            # leave a mirror running while the row said HOLD (§52.1).
                            mirror_follower.enter_hold()
                            mirror_follower.mode = "mirror"
                            # ⭐ THE FOLLOW SPEED IS READ FROM THE FOLLOWER'S OWN CAP, not
                            # repeated here. `MirrorLink`'s default is 1.0 because that is
                            # SafeRobot's default; if `--max-speed` raises the cap, a
                            # hardcoded 1.0 here would quietly become the binding limit and
                            # the mirror would stay slow for no visible reason.
                            mirror_link = MirrorLink(
                                mode=args.mirror, align_speed=MIRROR_ALIGN_SPEED,
                                catchup=args.mirror_catchup,
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
                            # Endpoint easing and corner blending are independent controls.
                            ease_idx = (ease_idx + 1) % len(EASINGS)
                            for one in arms:
                                one.easing = EASINGS[ease_idx]
                            hint(park_plan_line(edit_arm)); continue

                    if pending == "park":
                        if k == "w":
                            # A composite queue combines pose legs and validated trajectory legs.
                            # Keep one shared cursor and require every participating arm to reach its start.
                            park_take_next = True
                            hint(f"  park sequence: {park_seq_shown()}   "
                                 "▶ next digit names a RECORDING to play as a leg")
                            continue
                        if k.isdigit():
                            if park_take_next:
                                park_take_next = False
                                park_sequence.append("w" + k)
                            else:
                                park_sequence.append(k)
                            # ⛔ A HINT, NOT THE STATUS ROW. This used to `print(…,
                            # end="")`, which the shadowed print routes to `screen.set`
                            # — the heartbeat row. So the echo of what you were typing
                            # replaced the temperature readout and was then wiped by
                            # the next once-a-second repaint: the one piece of feedback
                            # in a modal state that drives 4.3 kg, flickering.
                            hint(f"  park sequence: {park_seq_shown()}"
                                 f"   (another digit, w+digit for a take, or Enter)")
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
                            if any(e.startswith("w") for e in wanted):
                                composite.begin(wanted, aimed)
                                continue
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
                            if any(e.startswith("w") for e in wanted):
                                composite.begin(wanted, aimed)
                                continue
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
                            # Clear the previous hint when a prompt changes so keys cannot target stale instructions.
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
                        elif shared_puck is not None:
                            # ⭐ With ONE shared puck the selection aims the DRIVING too
                            # (FINDINGS §68.8), and saying so at the switch is what makes
                            # a suddenly-still arm read as aimed-away rather than broken.
                            print(f"\n⭐ SELECTED: {selection.cycle()} — mode keys AND the "
                                  "puck apply to it. Unaimed arms hold.\n")
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
                                    one.enter_hold()
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
                    if k == "n":
                        # ⭐⭐ THE SETTINGS SCREEN. His request, 2026-08-17: *"all of these
                        # flags should be default options that can be changed in some
                        # controls mode and then should be saved."* The axis map is the
                        # precedent — edited live with keys, written to a config file.
                        #
                        # ⚠️ `n` because s, m, e, v, b and i are all taken. It shows the
                        # screen and waits; nothing changes until a key is pressed.
                        pending = "settings"
                        for line in live_lines(
                                {kk: getattr(args, kk) for kk in LIVE_ORDER},
                                settings_pick, builtin_defaults):
                            print(line)
                        continue

                    if k == "v":
                        # ⭐ Cycle which frame the puck's directions mean. Safe to do
                        # live: the twist is a VELOCITY, so a frame change alters the
                        # interpretation from the next cycle onward and leaves no
                        # stale cached state behind — unlike a mode change, which is
                        # why this does not need resync().
                        order = ["world", "tool", "camera"]
                        # Save the current frame's map before switching, then load the destination frame's map.
                        map_store.set(edit_arm.name, edit_arm.axis_map, edit_arm.frame)
                        edit_arm.frame = order[
                            (order.index(edit_arm.frame) + 1) % len(order)]
                        edit_arm.axis_map = map_store.for_arm(edit_arm.name, edit_arm.frame)
                        if edit_arm.teleop is not None:
                            edit_arm.teleop.frame = edit_arm.frame
                        # Use ArmSession.frame as the source of truth for mapping, solver and status.
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
                                wizard.enter_teleop(make_teleop)
                                print("\n⭐ MODE: TELEOP — SpaceMouse drives, all axes\n")
                            elif k == "g":
                                guide_warn = wizard.enter_guide()
                                if guide_warn:
                                    print(f"\n  ⚠️  {guide_warn}\n")
                                if wizard.mode == "guide":
                                    print("\n⭐ MODE: GUIDE — arm is weightless\n")
                            else:
                                wizard.enter_hold()
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
                            args.linear_scale = adjust_setting(
                                "linear_scale", args.linear_scale, True)
                            hint(f"linear speed {args.linear_scale:.3f} m/s"
                                 + (" (ceiling)"
                                    if args.linear_scale >= MAX_LINEAR_SCALE else ""))
                        elif k == "-":
                            args.linear_scale = adjust_setting(
                                "linear_scale", args.linear_scale, False)
                            hint(f"linear speed {args.linear_scale:.3f} m/s")
                        elif k == ".":
                            angular_scale = min(MAX_ANGULAR_SCALE,
                                                angular_scale * 1.25)
                            print(f"\n  rotation speed → {angular_scale:.2f} rad/s "
                                  f"({np.degrees(angular_scale):.0f}°/s)\n")
                        elif k == ",":
                            angular_scale = max(MIN_ANGULAR_SCALE,
                                                angular_scale / 1.25)
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
                        # CONTROLS deliberately moves one isolated axis at reduced speed.
                        # A mapping direction can only be validated against the observed arm motion.
                        edit_arm.enter_teleop(make_teleop)
                        edit_arm.mode = "map"
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
                                guide_warn = one.enter_guide()
                                if guide_warn:
                                    print(f"\n  ⚠️  {guide_warn}\n")
                        guided = "+".join(one.name for one in aimed if one.mode == "guide")
                        if guided:
                            print(f"\n⭐ MODE: GUIDE on {guided} — weightless, "
                                  "you are holding it now\n")
                    elif k == "t" and any(one.mode != "teleop" for one in aimed):
                        hint("")
                        for one in aimed:
                            if one.mode != "teleop":
                                one.enter_teleop(make_teleop)
                        print(f"\n⭐ MODE: TELEOP on {aimed_label} — each arm follows its "
                              "own SpaceMouse\n")
                    elif k == "h" and any(one.mode != "hold" for one in aimed):
                        hint("")
                        for one in aimed:
                            if one.mode != "hold":
                                one.enter_hold()
                        print(f"\n⭐ MODE: HOLD on {aimed_label}\n")
                    elif k in MODE_KEYS:
                        # A repeated mode key reports the current mode without re-entering it or resetting state.
                        hint(f"already in {MODE_KEYS[k]}")
                    # ⚠️ `w` and `l` REFUSED with two arms until 2026-08-14 night, because
                    # `Trajectory` held one arm's joints and a two-arm demonstration would have
                    # been saved as half of itself. The recorder now samples every arm into one
                    # timeline, which is ABC's own shape (ROADMAP §9.2), so the refusal is gone
                    # along with the test that pinned it.
                    elif k == "k":
                        # Labels mark good/bad intervals for export; they never alter the motion.
                        if recording.active is None:
                            print("\n  k labels a stretch while RECORDING — press w first.\n")
                        else:
                            recording.toggle_label(t)
                            if recording.label == "bad":
                                print(f"\n  ✎ BAD from {t - recording.started_at:.1f}s — press k again "
                                      "when it is good again.\n")
                            else:
                                print(f"\n  ✎ good again at {t - recording.started_at:.1f}s.\n")
                    elif k == "w":
                        # Recording is allowed in every mode and captures actual mode history.
                        if recording.active is None:
                            recording.start(t, meta={
                                # Keep the legacy first-arm fields and the full layout.
                                "arm": arms[0].name,
                                **sample_layout().to_meta(),
                                "simulated": bool(args.sim),
                                "nominal_hz": CONTROL_HZ,
                                "frames": {one.name: one.frame for one in arms},
                                "frame": arms[0].frame,
                            }, modes=[f"{one.name}:{one.mode}" for one in arms])
                            if capture is not None:
                                try:
                                    mono0 = time.monotonic_ns()
                                    frames_path = pending_frames_dir(
                                        takes_dir, f"{datetime.now():%Y%m%d_%H%M%S}_{mono0}")
                                    recording.start_frames(frames_path, capture_names, mono0,
                                                           factory=FrameSink)
                                except Exception as exc:
                                    recording.fail(exc)
                                    pending = "take_save"
                                    print(f"\n  {recording.frame_error}. "
                                          "Press a non-digit to discard this take.\n")
                                    continue
                            print("\n⏺  RECORDING " + " · ".join(
                                f"{one.name} {one.mode.upper()}" for one in arms)
                                + f"  ({sample_layout().n_joints} joints per sample)."
                                + (f"  📷 {len(capture_names)} camera(s)."
                                   if capture is not None else "")
                                + " Press w again to stop, k to mark a bad stretch.\n")
                        else:
                            # Freeze immediately at stop. Waiting for a slot must not append trailing samples.
                            recording.freeze()
                            if recording.pending.meta.get("marks"):
                                print(f"  ✎ labels: {recording.pending.bad_seconds():.1f}s of "
                                      f"{recording.pending.duration:.1f}s marked BAD "
                                      f"({len(recording.pending.meta['marks'])} mark(s)).")
                            n, secs = len(recording.pending), recording.pending.duration
                            if n < 2:
                                recording.request_discard()
                                pending = "take_discarding"
                                print("\n  nothing recorded (too short) — finishing discard.\n")
                            else:
                                pending = "take_save"
                                print(f"\n⏹  RECORDED {secs:.1f}s, {n} samples, "
                                      f"typical joint speed "
                                      f"{recording.pending.joint_speed(99):.2f} rad/s "
                                      f"(peak {recording.pending.max_joint_speed():.2f}).")
                                # ⭐ The whole shelf, once, before the first digit (FINDINGS §71.6): on 2026-08-19 every slot was occupied and the one-warning-per-digit flow cost eleven keypresses. The replace confirmation below still guards each overwrite.
                                for line in slot_overview(takes_dir):
                                    print(line)
                                print("\n     SAVE to which slot? 0-9, any other key discards.\n")
                    elif k == "l":
                        # Require Enter after the playback plan to prevent accidental motion from a stray key.
                        # Refuse a second playback prompt while a replay is active.
                        if playback.active is not None:
                            hint("")
                            print("\n  ⚠️ a playback is already running — press h or t to "
                                  "stop it, then l to pick the next one.\n")
                            continue
                        pending = "take_play"
                        # ⭐ Both folders in a --sim session, with the simulated ones
                        # marked, because "saved: 1, 2, 7" that silently mixes real
                        # demonstrations with simulated ones is the confusion the folder
                        # split exists to prevent.
                        # ⛔ `listing` and not `glob`: a macOS `._5.json` sidecar in a hand-copied
                        # recordings folder was offered here as a playable slot "._5" on the Linux
                        # station (FINDINGS §76). It is not a recording and it cannot be loaded.
                        real = [q.stem for q in listing(TAKES_DIR, "*.json")]
                        mine = ([q.stem for q in listing(takes_dir, "*.json")]
                                if takes_dir != TAKES_DIR else [])
                        have = ", ".join([*[f"{n}(sim)" for n in mine],
                                          *[n for n in real if n not in mine]])
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
                              f"   waypoints: {have or 'none'}")
                        # ⭐ FINDINGS §72.2: he pressed w HERE hoping to record the run, and got the composite take-marker instead (w+digit = play a recording as a leg, by design). Recording a run has always worked the other way round — start it FIRST — and nothing said so at the one place he reached for it.
                        print("     w then a digit plays a RECORDING as a leg. To RECORD "
                              "a run: press w BEFORE p —\n     recording runs through "
                              "parks and playbacks, w again stops it.\n")
                    elif k in "oc" and any(one.mode == "teleop" for one in aimed):
                        # ⭐ Every selected arm's jaws. `o` and `c` are commands, and BOTH
                        # selected means the operator asked for both grippers.
                        jaw_step = gripper_step if k == "o" else -gripper_step
                        for one in aimed:
                            if one.mode == "teleop":
                                one.gripper_value = clamp_gripper(one.gripper_value + jaw_step)
                    elif k in KEY_STEP_UP:
                        # Ease-ramp keys keep the same meaning across prompts and modes.
                        # Gripper step size is a separate command-line setting.
                        for one in aimed:
                            one.park_ramp = min(1.0, one.park_ramp * 1.4)
                        hint(ease_note(EASINGS[ease_idx].name, edit_arm.park_ramp))
                    elif k in KEY_STEP_DOWN:
                        for one in aimed:
                            one.park_ramp = max(
                                0.0, one.park_ramp / 1.4 if one.park_ramp > 0.03 else 0.0)
                        hint(ease_note(EASINGS[ease_idx].name, edit_arm.park_ramp))
                    elif k == "e":
                        # Report which motion setting changed; its meaning must agree across all modes.
                        ease_idx = (ease_idx + 1) % len(EASINGS)
                        # ⭐ Pushed onto the class, where step_path reads it per cycle —
                        # the script no longer owns the easing (item 23 group ④).
                        for one in arms:
                            one.easing = EASINGS[ease_idx]
                        hint(ease_note(EASINGS[ease_idx].name, edit_arm.park_ramp))
                    elif k == "r":
                        rotation = not rotation
                        hint(f"wrist rotation {'ON' if rotation else 'OFF'}")
                    elif k == ".":
                        angular_scale = min(MAX_ANGULAR_SCALE,
                                            angular_scale * 1.25)
                        hint(f"rotation speed {angular_scale:.2f} rad/s")
                    elif k == ",":
                        angular_scale = max(MIN_ANGULAR_SCALE,
                                            angular_scale / 1.25)
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
                        # ⭐ In a SCRUB they mean the full-push pace — his time-lapse dial
                        # (FINDINGS §68.5): "more than normal speed if I fully press the
                        # control forward". Safe high: a fast cursor is held back by the
                        # lag hold, so only the clock is fast, never the arm.
                        if playback.active is not None and playback.scrub:
                            args.scrub_max = adjust_setting(
                                "scrub_max", args.scrub_max, True)
                            hint(f"scrub pace: full push = {args.scrub_max:g}x the "
                                 f"recording's own speed")
                        elif any(one.mode == "park" for one in aimed):
                            for one in aimed:
                                one.park_speed = min(args.teleop_speed,
                                                     one.park_speed * 1.25)
                            hint(f"park speed {edit_arm.park_speed:.2f} rad/s")
                        else:
                            args.linear_scale = adjust_setting(
                                "linear_scale", args.linear_scale, True)
                            hint(f"linear speed {args.linear_scale:.3f} m/s"
                                 + (" (ceiling)"
                                    if args.linear_scale >= MAX_LINEAR_SCALE else ""))
                    elif k == "-":
                        if playback.active is not None and playback.scrub:
                            args.scrub_max = adjust_setting(
                                "scrub_max", args.scrub_max, False)
                            hint(f"scrub pace: full push = {args.scrub_max:g}x the "
                                 f"recording's own speed")
                        elif any(one.mode == "park" for one in aimed):
                            for one in aimed:
                                one.park_speed = max(0.05, one.park_speed / 1.25)
                            hint(f"park speed {edit_arm.park_speed:.2f} rad/s")
                        else:
                            args.linear_scale = adjust_setting(
                                "linear_scale", args.linear_scale, False)
                            hint(f"linear speed {args.linear_scale:.3f} m/s")
                    elif k == "?":
                        print(HELP)
                    elif k.isprintable() and k.strip():
                        print(f"\n  (key {k!r} does nothing — press ? for the list)\n")
                # Leaving PARK cancels the remaining composite legs and pending handovers.
                if mirror_link is not None and not any(one.mode == "mirror" for one in arms):
                    mirror_link = None
                    hint("")
                    print("  ⭐ MIRROR off — the follower left the mode.\n")

                # ⚠️ Per arm, because one arm can be parking while the other is being
                # driven. Each arm's run is abandoned by ITS OWN mode leaving `park`.
                for one in arms:
                    if one.mode == "park" or one.park_path is None:
                        continue
                    # ⭐ Through the class, so queued segments and a jaw pause in progress
                    # are dropped AND counted — clearing only `park_path` here would leave
                    # a stale queue behind and under-report what was cancelled.
                    left = one.abandon_path()
                    unfinished = left > PARK_TOLERANCE
                    if unfinished:
                        print(f"\n  ⚠️  arm {one.name}: run abandoned with {left:.2f} rad of "
                              "path left — leaving PARK cancels the rest.\n")
                        composite.abandon("a park leg was abandoned")
                    # An interrupted park cannot authorize replay. All replay arms must finish
                    # their own park-to-start before the shared cursor may advance.
                    if unfinished and playback.pending is not None:
                        playback.cancel_pending()
                        hint("")
                        print("  ⚠️  playback cancelled — it never reached the start pose.\n")
                # Leaving replay mode cancels it and its remaining composite legs.
                if playback.active is not None and any(one.mode != "replay" for one in playback.arms):
                    for a2 in playback.arms:
                        if a2.mode == "replay":
                            a2.enter_hold()
                    left = playback.active.duration - playback.cursor
                    if playback.scrub:
                        # ⭐ Leaving a SCRUB via a mode key is its normal end, not an
                        # abandonment — the scrub has no finish line of its own.
                        print(f"\n  ⭐ scrub ended at {playback.cursor:.1f}s of "
                              f"{playback.active.duration:.1f}s.\n")
                        playback.scrub = False
                    elif left > 0.05:
                        print(f"\n  ⚠️  playback abandoned with {left:.1f}s left"
                              + ("" if len(playback.arms) < 2 else
                                 " — every replay arm is HOLDING now")
                              + ".\n")
                    playback.finish()
                    composite.abandon("the playback was abandoned")
                    hint("")
                if stop_reason:
                    break

                # Sample every arm in layout order on one recording clock.
                # Record measured positions, not the requested targets.
                if recording.active is not None:
                    # Joint and camera sampling failures stop only this take.
                    # Its writers and directory remain owned through completion.
                    try:
                        recording.sample(
                            t, [v for one in arms
                                for v in np.asarray(one.robot.get_joint_pos(), dtype=float)],
                            [f"{one.name}:{one.mode}" for one in arms])
                        if recording.sink is not None and capture is not None:
                            recording.offer_frames(capture.sample())
                    except Exception as exc:  # noqa: BLE001
                        recording.fail(exc)
                        pending = "take_save"
                        print(f"\n⚠️  {recording.frame_error}")
                        print("     Recording stopped; arm modes are unchanged. "
                              "The take is retained until you discard it.\n")
                    else:
                        # ⚠️ A bound, because this grows in memory for as long as it runs and
                        # nothing else would ever stop it. 100 000 samples is ~16 minutes at
                        # 100 Hz, comfortably past the ~4.5 minutes a long-context policy
                        # wants (ROADMAP §9.3). Stopping and saying so beats running out of
                        # memory in a process that is driving an arm.
                        if len(recording.active) >= MAX_TAKE_SAMPLES:
                            # Freeze it the same way `w` does, or the limit would not be one.
                            recording.freeze()
                            pending = "take_save"
                            print(f"\n⏹  RECORDING STOPPED at the {MAX_TAKE_SAMPLES} sample "
                                  f"limit ({recording.pending.duration:.0f}s).")
                            for line in slot_overview(takes_dir):
                                print(line)
                            print("\n     SAVE to which slot? 0-9, any other key discards.\n")

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
                # ⭐⭐ ONE PUCK, N ARMS: THE PUCK FOLLOWS THE SELECTION (his design,
                # 2026-08-18, FINDINGS §68.8): a → B drives B, a → G drives G, a → BOTH
                # drives both arms at once, each from its own pose. The shared reader is
                # read ONCE per cycle — two arms draining one HID queue would split the
                # event stream between them — and unaimed arms read as centred.
                shared_axes: list[float] | None = None
                shared_buttons = 0
                if shared_puck is not None:
                    try:
                        shared_axes = shared_puck.read()
                        shared_buttons = getattr(shared_puck, "buttons", 0)
                    except Exception as exc:  # noqa: BLE001
                        # Same graceful stop as the per-arm guard below (FINDINGS §68.2).
                        shared_axes = [0.0] * 6
                        if not stop_reason:
                            stop_reason = (f"the shared SpaceMouse stopped answering "
                                           f"({type(exc).__name__}) — unplugged?")
                            print(f"\n⛔ {stop_reason}")
                            print("   Treating it as centred and parking safely.\n")
                for one in arms:
                    if shared_axes is not None:
                        # ⭐ The puck follows the selection; unaimed arms read centred.
                        aimed_now = one.name in selection.names()
                        one.raw_axes = list(shared_axes) if aimed_now else [0.0] * 6
                        buttons = shared_buttons if aimed_now else 0
                    else:
                        try:
                            one.raw_axes = one.reader.read()
                        except Exception as exc:  # noqa: BLE001
                            # A puck read failure requests the controlled stop path rather than escaping
                            # the loop directly and disabling raised arms.
                            one.raw_axes = [0.0] * 6
                            if not stop_reason:
                                stop_reason = (f"arm {one.name}'s SpaceMouse stopped "
                                               f"answering ({type(exc).__name__}) — "
                                               f"unplugged?")
                                print(f"\n⛔ {stop_reason}")
                                print("   Treating that puck as centred and parking "
                                      "safely.\n")
                            continue
                        buttons = getattr(one.reader, "buttons", 0)
                    pressed = buttons & ~one.buttons_prev              # rising edge only
                    one.buttons_prev = buttons

                    if one.learn_button is not None and pending is not None:
                        # Only one prompt may own input; opening a new one cancels the previous prompt.
                        one.learn_button = None
                        print(f"\n  ⚠️ the gripper-button learning on arm {one.name} is "
                              f"CANCELLED — another prompt opened. Press b to restart it.\n")
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

                # Step each arm's mode in requested layout order, independent of key selection.
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

                        # The workspace bound is fixed to the base: radial reach plus a floor.
                        # Do not recenter it on TELEOP entry; that would move the permitted workspace.
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
                            # ⛔ Same latch in TELEOP. His log shows the stall firing four
                            # more times there, because holding a puck button re-commands
                            # the jaws every cycle exactly as MIRROR does.
                            full[N_ARM] = clamp_gripper(one.hold_jaw(one.gripper_value))
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
                            # When mirror stops itself, stop follower commands and report its measured reason.
                            joint_name = ""
                            if mirror_link.stop_joint is not None:
                                joint_name = YAM_JOINTS.get(
                                    mirror_link.stop_joint + 1, ("joint",))[0]
                            print(f"\n⛔ MIRROR STOPPED — {mirror_link.stop_reason}"
                                  + (f", {joint_name}" if joint_name else ""))
                            print(f"     {mirror_link.stop_detail}")
                            # Use measured leader speed and follower lag when diagnosing mirror tracking.
                            clipped = getattr(one.robot, "limited_cycles", 0) - mirror_clipped_at
                            if clipped > 0:
                                print(f"     ⚠️ SafeRobot held the command back on {clipped} "
                                      f"cycle(s) (its "
                                      f"{getattr(one.robot, 'max_lag', 0.25):.2f} rad "
                                      "following-error limit).")
                            if mirror_link.stop_cause == "follow_limit":
                                # ⛔⭐⭐ NAME BOTH FLAGS, AND NAME THE ONE THAT ACTUALLY
                                # FIRED FIRST. This branch used to say only *"That
                                # allowance is `--max-speed`. Raise it one step."*
                                #
                                # ⛔ On 2026-08-17 Julien raised `--max-lag` from 0.25 to
                                # 0.4 to 1.0 across three sessions chasing this message,
                                # and **none of it could ever have helped**: the stop is
                                # triggered by the gap passing `--mirror-gap`, which was
                                # sitting at its 0.35 default because he had not set it.
                                # His own earlier run with `--mirror-gap 0.6` is the one he
                                # described as working *"much better"*.
                                #
                                # ⚠️ Third time a speed-layer confusion has cost him a
                                # session ([FINDINGS §58.3](../docs/FINDINGS.md)). The
                                # message named the limit's VALUE ("limit 0.35") and never
                                # named the FLAG that sets it, so the number was unusable.
                                #
                                # ⭐ Two independent routes out, and both are stated,
                                # because they do different things: a wider tolerance means
                                # it does not stop, a faster follower means the gap does not
                                # grow.
                                suggest = max(3.0, round(mirror_link.stop_leader_speed
                                                         * 1.5 + 0.4, 1))
                                print(f"     ⭐ TWO WAYS OUT, and the first is the limit "
                                      f"that actually fired:")
                                print(f"       1. `--mirror-gap {mirror_link.max_gap * 2:.2f}`"
                                      f"  (now {mirror_link.max_gap:.2f}) — how far behind "
                                      f"is TOLERATED before stopping.")
                                print(f"       2. `--max-speed {suggest:g}`"
                                      f"  (now {args.max_speed:.2f}) — how fast the follower "
                                      f"may move. You moved the leader at "
                                      f"{mirror_link.stop_leader_speed:.2f} rad/s.")
                                # Report this run's measured leader speed; historical recordings are not its evidence.
                                print(f"     ⚠️ `--max-lag` does NOT affect this stop.")
                            elif mirror_link.stop_cause == "tracking":
                                print("     ⭐ More `--max-speed` will NOT help: the arm, not "
                                      "the software, is the limit.")
                                print(f"     Either guide the leader more slowly, or loosen "
                                      f"the tolerance with --mirror-gap "
                                      f"{mirror_link.max_gap * 2:.2f}.")
                            print("     Press i then Enter to engage it again.\n")
                            one.enter_hold(); hint("")
                            mirror_link = None
                        else:
                            full = np.asarray(cmd, dtype=float).copy()
                            # ⛔ The jaws go through the clamp, never straight from the leader.
                            # A leader whose jaws rest on a stop would otherwise drive the
                            # follower's onto its own stop and HOLD there, which is stall
                            # torque and is how motor 7 was cooked three times (FINDINGS §4).
                            if one.robot.num_dofs() > N_ARM and len(full) > N_ARM:
                                # ⛔⭐ THROUGH THE LATCH, so a stalled follower stops being
                                # pushed further closed by the leader every cycle. Opening
                                # clears it, so letting go of the leader's jaws frees the
                                # follower's immediately.
                                full[N_ARM] = clamp_gripper(
                                    one.hold_jaw(float(full[N_ARM])))
                            one.robot.command_joint_pos(full)
                            one.prev_q = full[:N_ARM].copy()

                    elif one.mode == "park" and one.park_path is not None and one.park_target is not None:
                        # ⭐⭐ item 23 group ④ (2026-08-18): the CLASS advances the park.
                        # `ArmSession.step_path` owns the cursor, the lag hold, the easing,
                        # the stall guard and the verdict — 48 tests. This branch only
                        # narrates the ParkStep it returns and performs the handovers,
                        # which is the split the whole restructure was for: the class
                        # decides, the script narrates.
                        ps = one.step_path(t, dt)
                        if ps.verdict == "moving":
                            if ps.leg_passed is not None:
                                # ⭐ Time each waypoint. Julien: *"you can't really see how
                                # long each parking section took."* The class stamps the
                                # leg clock; this line only prints it.
                                print(f"  ⭐ slot {ps.leg_passed} in {ps.leg_seconds:.1f}s"
                                      + (f" → next {ps.next_leg}" if ps.next_leg else ""))
                            elif t >= next_park_report:
                                next_park_report = t + 1.0
                                # ⛔ A HINT, NOT THE STATUS ROW — routed through screen.set
                                # this once painted over the temperature heartbeat during
                                # the exact motion where it matters most.
                                hint(f"  moving… {ps.remaining:.2f} rad of path "
                                     f"left, {ps.err:.3f} to the final pose, "
                                     f"{ps.lag:.3f} behind")
                        elif ps.verdict in ("arrived", "settled"):
                            extra = ("" if ps.verdict == "arrived" else
                                     " — as close as the arm holds itself under load")
                            one.enter_hold()
                            hint("")    # the progress readout has nothing left to say
                            # ⭐ Total and settling answer different questions: the total
                            # is what speed/corner/ease tuning changes, and the settling
                            # is how long the arm closed the last gap after the commanded
                            # path ran out. The class stamps both from the right clocks —
                            # park_start_t, NOT park_leg_t, for the total (FINDINGS §34.3).
                            tail = (f", {ps.settling_seconds:.1f}s of that settling"
                                    if 0.05 < ps.settling_seconds < ps.total_seconds - 0.05
                                    else "")
                            print(f"⭐ PARK reached in {ps.total_seconds:.1f}s{tail} "
                                  f"({ps.err:.3f} rad off{extra}) → HOLD")
                            # ⛔⭐⭐ THE ARRIVAL CLEARS ITS OWN PATH — the fix for the bug
                            # that killed the first two-arm playback. The generic "leaving
                            # PARK abandons the run" block fires for any arm whose mode is
                            # no longer `park` while `park_path` is still set; an ARRIVAL
                            # used to leave the path in place, so with two arms the FIRST
                            # arrival (which waits for the second) had its pending playback
                            # cancelled as "abandoned". FINDINGS §57.1.
                            one.park_path, one.park_marks = None, []
                            # Capture the completed park's purpose before advancing the composite queue.
                            # A previous leg's arrival cannot satisfy the next leg's park-to-start gate.
                            arrived_purpose = park_purpose.pop(one.name, "operator")
                            # ⭐ Composite (ROADMAP §6.6.1a): a pose-leg's park arrived.
                            # The leg is done when EVERY awaited arm has arrived; only
                            # then does the queue advance — in the ARRIVAL branch, never
                            # a key branch, which is the §57.1 rule.
                            composite.arrived(one.name)
                            # ⭐ The handover from "drive to the start pose" to "play the
                            # recording" lives HERE, in the arrival branch, so a park that
                            # was blocked or interrupted can never roll into a playback:
                            # only a park that actually arrived does — and only a park that
                            # was FOR the playback (`arrived_purpose`), never a pose leg's.
                            if (playback.pending is not None and one in playback.arms
                                    and arrived_purpose == "replay"):
                                # ⛔⭐ EVERY ARM MUST ARRIVE BEFORE ANY ARM PLAYS. Each one
                                # parks a different distance and finishes at a different
                                # moment; starting on the first arrival would have the
                                # second arm still parking while the recording ran.
                                waiting = playback.credit_arrival(one.name, arrived_purpose)
                                off_start = []
                                if not waiting:
                                    # Independently compare every arm's measured pose with its replay start before playback.
                                    want_all = list(playback.pending.start_pose() or ())
                                    for a in playback.arms:
                                        sl = playback.layout.slice_for(a.name)
                                        want = np.asarray(want_all[sl], dtype=float)
                                        have = np.asarray(a.robot.get_joint_pos(),
                                                          dtype=float)
                                        n_cmp = min(N_ARM, len(want), len(have))
                                        err = float(np.max(np.abs(have[:n_cmp]
                                                                  - want[:n_cmp])))
                                        if err > REPLAY_START_TOLERANCE:
                                            off_start.append((a.name, err))
                                if waiting:
                                    print(f"     arm {one.name} is at the start pose; "
                                          f"waiting for {', '.join(waiting)}.")
                                elif off_start:
                                    where = ", ".join(f"{n} is {e:.2f} rad off"
                                                      for n, e in off_start)
                                    print(f"\n  ⛔ NOT playing: {where} — a playback must "
                                          "begin at the recording's own start pose "
                                          "(FINDINGS §57.1), and something moved the arm "
                                          "after its park arrived. Nothing plays; press "
                                          "l (or retype the run) to park and retry.\n")
                                    playback.cancel_pending()
                                    composite.abandon("a playback almost began away "
                                                      "from its start pose")
                                else:
                                    playback.start(t)
                                    scrub_ref_t, scrub_ref_s, scrub_ref_h = t, 0.0, 0.0
                                    for a in playback.arms:
                                        a.mode = "replay"
                                    if playback.scrub:
                                        print(f"\n▶  SCRUB: {playback.active.duration:.1f}s of "
                                              f"recorded movement on "
                                              f"{'+'.join(a.name for a in playback.arms)}."
                                              f" The puck is the clock — push forward "
                                              f"to play, pull back to rewind, let go to "
                                              f"freeze.\n     Full push = "
                                              f"{args.scrub_max:g}x the recording "
                                              f"(-/+ changes it). h or t ends it.\n")
                                    else:
                                        print(f"\n▶  PLAYING {playback.active.duration:.1f}s of "
                                              f"recorded movement on "
                                              f"{'+'.join(a.name for a in playback.arms)} "
                                              f"at {playback.speed:.2f}x. "
                                              f"Press h or t to stop.\n")
                        elif ps.verdict == "jaws":
                            # ⭐⭐ THE JAW PAUSE (items 3 + 10): the run split at a
                            # waypoint where only the jaws move. The class holds the arm,
                            # drives the jaws and measures when they are done; this
                            # branch only says what is happening, so a pause never reads
                            # as a stall.
                            if ps.jaw_started:
                                hint("")
                                # ⭐ The settled offset is the miss-diagnosis number: at
                                # the friction floor (~0.02-0.04 rad) a missed grab means
                                # the POSE was taught off; well above it, the arm never
                                # got there (FINDINGS §70.5).
                                print(f"\n  ⏸ {ps.jaw_name}: only the jaws move (arm "
                                      f"settled {ps.jaw_arm_off:.3f} rad off) — going "
                                      f"to {ps.jaw_target:.2f} and waiting for them to "
                                      "stop.")
                            elif ps.jaw_done:
                                note = (" ⚠️ timed out still moving — continuing anyway"
                                        if ps.jaw_timed_out else "")
                                print(f"  ⭐ jaws done in {ps.jaw_seconds:.1f}s{note}"
                                      + (f" → next {ps.next_leg}" if ps.next_leg else ""))
                                # ⭐ item 10: `check_grasp` grades a CLOSING leg from
                                # where the jaws stopped. It stays silent when it cannot
                                # know (an opening leg, a timeout) — `confident` is the
                                # gate, and printing a guess would be the §0 pattern.
                                if ps.grasp is not None and ps.grasp.confident:
                                    if ps.grasp.holding:
                                        print("     ✋ holding something — the jaws "
                                              f"stopped {ps.grasp.gap:.3f} of the stroke "
                                              "short of closed.")
                                    else:
                                        print("     ∅ the jaws closed onto themselves — "
                                              "nothing gripped.")
                            elif t >= next_park_report:
                                next_park_report = t + 1.0
                                hint(f"  ⏸ waiting for the jaws… {ps.jaw_seconds:.1f}s")
                        else:
                            # ⛔ BLOCKED. Never spin silently: say so and hold. The wording
                            # keeps the old two shapes — mid-path (the arm stopped
                            # following) and at the end (it stopped closing) — decided by
                            # the remaining path, which the ParkStep carries.
                            one.enter_hold()
                            hint("")
                            composite.abandon("a park leg was blocked")
                            if ps.remaining > 1e-9:
                                print(f"\n⛔ PARK BLOCKED — the arm stopped following "
                                      f"{ps.lag:.3f} rad behind the path, no progress "
                                      f"for {PARK_STALL_SECONDS:.0f}s. Now HOLDING.\n")
                            else:
                                print(f"\n⛔ PARK BLOCKED — {ps.err:.3f} rad still to go "
                                      f"and no progress for {PARK_STALL_SECONDS:.0f}s.")
                                print(f"   The command ran {ps.lag:.3f} rad ahead of the "
                                      f"arm; SafeRobot limited "
                                      f"{getattr(one.robot, 'limited_cycles', 0)} cycles.")
                                print("   Something is blocking it, or the pose is "
                                      "unreachable. Now HOLDING.\n")

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
                if playback.active is not None and playback.layout is not None and all(
                        a.mode == "replay" for a in playback.arms):
                    # ⛔ MEASURED IN THE RECORDING'S ARM ORDER, never the session's. The
                    # layout came out of the file, so the slices line up with the samples
                    # even if `--arms` was given the other way round.
                    measured = np.concatenate([
                        np.asarray(a.robot.get_joint_pos(), dtype=float) for a in playback.arms])
                    # ⛔ The grippers are left out of the "is it keeping up" check by INDEX,
                    # because with two arms the first gripper sits in the middle of the
                    # vector. Jaws legitimately sit far from their commanded value while
                    # closing on an object, and counting that as lag would stall every
                    # playback that grips anything.
                    defl = 0.0
                    if playback.scrub:
                        # ⭐ SCRUB: the puck is the clock. EITHER puck works — during
                        # playback nobody's hand is driving an arm, so whichever hand is
                        # free is the deadman. The forward/back axis (index 1) is the
                        # natural "push to play" gesture; largest deflection wins.
                        for a2 in arms:
                            ax = getattr(a2, "raw_axes", None)
                            if ax and abs(ax[1]) > abs(defl):
                                defl = float(ax[1])
                    rs = playback.advance(measured, real_dt, n_arm=N_ARM,
                                          max_lag=MAX_CURSOR_LAG, deflection=defl,
                                          scrub_max=args.scrub_max)
                    for a in playback.arms:
                        piece = np.asarray(rs.target[playback.layout.slice_for(a.name)],
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
                    playback.observe(rs, measured, t, real_dt)
                    if rs.finished:
                        for a in playback.arms:
                            a.enter_hold()
                        hint("")
                        # ⭐⭐ SAY WHERE THE EXTRA TIME WENT. Julien's first playbacks ran
                        # 2.3 s longer than the recording and the old message reported only
                        # the total, so it read as a bug with no explanation. The whole
                        # difference is the loop holding the clock while the arm catches up,
                        # which is a decision this code makes on purpose. A readout has to
                        # show what can go wrong, not only what looks tidy — the same lesson
                        # as showing the jaw temperature separately (FINDINGS §11).
                        planned = playback.active.duration / playback.speed
                        elapsed = t - playback.started_at
                        print(f"⭐ PLAYBACK finished in {elapsed:.1f}s → HOLD")
                        print(f"     {planned:.1f}s of movement at {playback.speed:.2f}x, "
                              f"plus {playback.held_seconds:.1f}s waiting for the arm to catch up.")
                        # ⛔ THE TWO NUMBERS MUST RECONCILE, and on 2026-08-13 they did not:
                        # a 3.6 s recording reported 3.6 + 0.4 and finished in 4.6. The gap
                        # was the loop running below 100 Hz while the cursor advanced in
                        # nominal time. That is fixed, and this check stays so a future
                        # version cannot reintroduce it silently.
                        unaccounted = elapsed - planned - playback.held_seconds
                        if abs(unaccounted) > 0.15 + 0.05 * elapsed:
                            print(f"     ⚠️  {unaccounted:+.1f}s is unaccounted for. The loop "
                                  f"averaged {loop_hz:.0f} Hz against {CONTROL_HZ:.0f}.")
                            # ⭐ The average explains the drift; the worst pass is what a stall looks like, and it is the number PERFORMANCE.md §2 was missing.
                            print(f"     {loop_timer.line()}")
                        print(f"     worst it fell behind: {playback.worst_lag:.3f} rad "
                              f"(the loop holds the clock past {MAX_CURSOR_LAG:.2f}).")
                        if playback.held_seconds > 0.15 * planned:
                            print(f"     ⚠️  it spent {100 * playback.held_seconds / (planned + playback.held_seconds):.0f}% "
                                  f"of the run waiting. Try a lower speed for a faithful replay.\n")
                        else:
                            print()
                        if playback.tracking is not None and playback.tracking.cycles > 20:
                            # ⚠️ MEASURED, so read it as such. The playback holds its clock
                            # once the arm falls behind, so the speeds here are not an even
                            # sweep, and load changes with the arm's pose. It is the cheap
                            # first answer; ROADMAP §7.5 has the active sweep if this is
                            # ambiguous.
                            # ⭐⭐ ONE list of names for BOTH the printed table and the
                            # saved file, and they DISAGREED until 2026-08-17. The saved
                            # JSON below already labelled every row with its arm, and its
                            # comment says exactly why that is necessary — while this
                            # print used `YAM_JOINTS.get(i + 1)` on a **flat** index. With
                            # two arms that names arm B's seven joints correctly and then
                            # labels every one of arm G's simply "joint", because the flat
                            # indices 7-13 become keys 8-14 and `YAM_JOINTS` only holds
                            # 1-7.
                            #
                            # ⛔ Julien's 2026-08-17 playback log is the evidence: six
                            # named rows, six anonymous ones, and **nothing saying which
                            # arm any row belonged to**. The six named rows read as "the
                            # arm" when they were only arm B.
                            #
                            # ⚠️ Same family as the `label_verdict` defect: code written
                            # for one arm that produces confident, plausible, wrong output
                            # with two, and raises nothing. Building the list once is the
                            # actual fix, because two copies is what let them drift.
                            names = flat_joint_names(
                                [a.name for a in playback.arms],
                                playback.layout.per_arm, playback.tracking.n_joints)
                            print("     how well each joint kept up "
                                  f"(the loop holds past {MAX_CURSOR_LAG:.2f} rad):")
                            for line in tracking_table(playback.tracking.rows(), names,
                                                       MAX_CURSOR_LAG):
                                print(line)
                            # Save the tracking measurements with provenance for later comparison.
                            try:
                                # Share joint names between the printed table and its saved measurements.
                                rec = playback.tracking.to_dict(names)
                                rec["meta"] = {
                                    "arms": [a.name for a in playback.arms],
                                    "joints_per_arm": playback.layout.per_arm,
                                    "slot": playback.slot,
                                    "played_at": dt_now(),
                                    "commit": git_commit(),
                                    "speed": round(playback.speed, 3),
                                    "taught_speed_p99": round(playback.active.joint_speed(99), 4),
                                    "recording_duration_s": round(playback.active.duration, 3),
                                    "recording_meta": dict(playback.active.meta),
                                    "elapsed_s": round(elapsed, 3),
                                    "held_s": round(playback.held_seconds, 3),
                                    "worst_lag_rad": round(playback.worst_lag, 5),
                                    "loop_hz": round(loop_hz, 1),
                                    "loop_timing": loop_timer.to_dict(),
                                    "max_cursor_lag": MAX_CURSOR_LAG,
                                    "max_planned_joint_speed": args.teleop_speed,
                                    "safe_max_speed": args.max_speed,
                                }
                                tracking_dir.mkdir(parents=True, exist_ok=True)
                                stamp = dt_now().replace(":", "-")
                                out = tracking_dir / f"{playback.slot}_{stamp}.json"
                                out.write_text(json.dumps(rec, indent=1) + "\n")
                                print(f"     ⭐ saved this table → "
                                      f"{out.relative_to(REPO)}")
                            except Exception as exc:  # noqa: BLE001
                                print(f"     ⚠️  could not save the tracking table: "
                                      f"{type(exc).__name__}: {exc}")
                        playback.finish()
                        composite.leg_done()
                    elif t - playback.progress_at > PARK_STALL_SECONDS:
                        # ⛔ NEVER WAIT FOR EVER. Holding the clock is right for a moment
                        # and wrong for ever: an arm that cannot catch up is blocked, and a
                        # playback that sits silently holding its clock is the treadmill
                        # bug again (FINDINGS §24). Same patience the park uses.
                        for a in playback.arms:
                            a.enter_hold()
                        hint("")
                        print("\n⛔ PLAYBACK BLOCKED — "
                              f"{'+'.join(a.name for a in playback.arms)} stopped following "
                              f"{rs.lag:.3f} rad behind the recording, no progress for "
                              f"{PARK_STALL_SECONDS:.0f}s. Now HOLDING.\n")
                        playback.finish()
                        composite.abandon("the playback was blocked")
                    elif t >= next_park_report:
                        next_park_report = t + 1.0
                        # ⭐ A scrub is a POSITION, never a countdown — it goes both ways,
                        # and "playing… Xs left" over a scrub read as a stuck playback
                        # (his 2026-08-18 report, FINDINGS §68.4).
                        if playback.scrub:
                            window = max(1e-6, t - scrub_ref_t)
                            eff = abs(playback.cursor - scrub_ref_s) / window
                            held_in_window = playback.held_seconds - scrub_ref_h
                            scrub_ref_t, scrub_ref_s = t, playback.cursor
                            scrub_ref_h = playback.held_seconds
                            # ⭐ Named ONLY when the hold actually bit this window, so a
                            # released puck never gets blamed on the arm.
                            why = (f" — the ARM binds, raise max_speed (n, 1) to scrub "
                                   f"faster" if held_in_window > 0.3 * window else "")
                            hint(f"  scrubbing… at {playback.cursor:.1f}s of "
                                 f"{playback.active.duration:.1f}s, effective {eff:.2f}x of the "
                                 f"recording{why}")
                        else:
                            hint(f"  playing… {playback.active.duration - playback.cursor:.1f}s left, "
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
                    # Render every arm; only the first row carries the shared clock and recording state.
                    lead = f" t={t:6.1f}s"
                    # ⭐ RECORDING HAS TO BE VISIBLE ON THE HEARTBEAT, not only in the
                    # message that started it. A session where recording is silently still
                    # running produces a demonstration full of whatever happened next, and
                    # the operator finds out at training time.
                    if recording.active is not None:
                        lead += f"  ⏺ REC {t - recording.started_at:5.1f}s"
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
            # ⭐ Printed for every session, not only a playback, because the worst pass is the one number about this loop that nothing recorded until 2026-08-20. ⚠️ On `--sim` it is Python-side jitter only: a fake arm answers in microseconds and a real one is 14 motors over two USB adapters ([FINDINGS §76.12](../docs/FINDINGS.md)).
            if loop_timer.count:
                print(f"⭐ {loop_timer.line()}")

            # Controlled interrupt/fault stops attempt a guarded park for each live arm.
            # A dead CAN chain cannot park. Failed parks return to the operator consent flow.
            # Successful automatic parks disable afterward; planned q keeps the quit menu.
            # Exceptions escaping this flow still reach the outer cleanup handler.
            planned_quit = bool(stop_reason) and "quit requested" in (stop_reason or "")
            unplanned = not planned_quit
            if unplanned:
                exit_code = 130 if interrupted else 1
            auto_parked = False
            # ⚠️ `any(...)` rather than `all(...)`: if one chain has died the other arm can
            # still be parked, and parking it is better than leaving it holding. `park_arms`
            # skips the dead one and names it.
            live = [one for one in arms if one.alive()]
            if live and (interrupted or unplanned) and any(
                    one.base_pose is not None for one in live):
                for one in live:
                    one.enter_hold()
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
                        one.enter_hold()
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
                        # The quit-menu q option parks all live arms and disables only after successful arrival.
                        outcome = park_arms([one for one in arms if one.alive()],
                                            keys, clamp_gripper)
                        if outcome == "arrived":
                            print("\n   Parked. Disabling the motors now.\n")
                            break
                        for one in arms:
                            if one.alive():
                                one.enter_hold()
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
                                one.enter_hold()
                        print("   q = park+disable    p = park again    g = weightless    d = disable")
                    elif k == "g":
                        # ⛔ EVERY arm, and with two that is 8.6 kg going weightless at once.
                        # The operator asked for it at the quit menu, where the alternative is
                        # disabling, so it is the safer of the two. The banner says how much.
                        for one in arms:
                            if one.alive():
                                guide_warn = one.enter_guide()
                                if guide_warn:
                                    print(f"\n  ⚠️  {guide_warn}\n")
                        guided = "+".join(one.name for one in arms
                                          if one.alive() and one.mode == "guide")
                        if guided:
                            print(f"\n⭐ weightless: {guided}"
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
        exit_code = 130
        stop_reason = "interrupted during startup or shutdown"
        print("\ninterrupted.")
    except Exception as exc:  # noqa: BLE001
        exit_code = 1
        stop_reason = f"{type(exc).__name__}: {exc}"
        print(f"\n⛔ {stop_reason}")
        for note in getattr(exc, "__notes__", []):
            print(f"   {note}")
    finally:
        # Snapshot initialized sessions before disabling their underlying handles.
        # The cleanup list also includes robots whose session never initialized.
        alive_at_teardown = {one.name: _safe_fact(lambda one=one: bool(one.alive()))
                             for one in arms}
        if robots:
            _SHUTTING_DOWN["yes"] = True
        for name, robot in robots.items():
            try:
                disabled = shutdown_robot(robot)
                print(f"\narm {name} motors confirmed disabled: {disabled}")
                missing = sorted(set(range(1, n_motors + 1)) - set(disabled))
                if missing:
                    exit_code = 1
                    print(f"\n⛔ arm {name}: could not confirm motors {missing} disabled.")
                    print("   Treat that arm as live. Support it and cut the mains.")
            except Exception as exc:  # noqa: BLE001
                exit_code = 1
                print(f"\n⛔ arm {name}: could not confirm the motors are disabled: "
                      f"{type(exc).__name__}: {exc}")
                print("   Treat that arm as live. Support it and cut the mains.")
        # Peripheral cleanup may wait for camera frames or disk writes. Motors
        # have already been processed; one peripheral failure must not skip another.
        for one_puck in pucks.values():
            handle = one_puck.get("handle")
            if handle is not None:
                try:
                    handle.close()
                except Exception as exc:  # noqa: BLE001
                    exit_code = 1
                    print(f"\n⚠️ could not close SpaceMouse: {exc}")
        for label, cleanup in (
            ("recording", recording.shutdown),
            ("camera readers", lambda: capture.stop() if capture is not None else None),
        ):
            try:
                cleanup()
            except Exception as exc:  # noqa: BLE001
                exit_code = 1
                print(f"\n⚠️ could not clean up {label}: {exc}")
        threading.excepthook = previous_thread_hook
        try:
            signal.signal(signal.SIGINT, previous_sigint)
        except Exception:  # noqa: BLE001
            pass
        # Persist maps only after an intentional change. Read-only sessions must not
        # rewrite config or replace the backup of the previous map.
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
        if robots:
            # Write incident facts after attempting device shutdown. Capture cached state
            # with guarded reads so incomplete startup or a dead chain cannot hide the original fault.
            try:
                bad_stop = exit_code != 0 or (
                    bool(stop_reason) and "quit requested" not in (stop_reason or ""))
                # Use the last available cycle values. Fresh reads from a failed device can raise
                # and would erase the evidence of the state before shutdown.
                facts = {} if not bad_stop else {
                    "stop_reason": stop_reason,
                    "arms": [one.name for one in arms] or arm_names,
                    "acquired_robots": list(robots),
                    "reach_limit": args.reach,
                    "floor_limit": args.floor,
                    "loop_hz": _safe_fact(lambda: round(loop_hz, 1)),
                    # ⛔ A LAMBDA, not `_safe_fact(loop_timer.to_dict)`. `_safe_fact`'s own docstring says a local here may be unbound if the loop never ran a cycle, and the attribute access in the shorter form happens OUTSIDE its try, so on the failed-build path it would raise and take the whole incident file with it.
                    "loop_timing": _safe_fact(lambda: loop_timer.to_dict()),
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
                            # ⭐ Read at the top of the teardown, never here: after the
                            # disable loop above, `one.alive()` is False on every path.
                            # The key is renamed on purpose, so old incident files (whose
                            # `chain_alive` was always the meaningless post-shutdown read)
                            # cannot be confused with files carrying the real measurement.
                            "chain_alive_at_teardown": alive_at_teardown.get(one.name, False),
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

    # Print summaries only for initialized sessions; report partial startup without
    # assuming an arm or a control-loop sample exists.
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
        # ⛔ BOTH lines print in the SAME frame, and the frame is NAMED. This summary once
        # printed the current map in the frame the arm ENDED in (camera) against a `was:`
        # line in world labels — different motion names for the same store — and it read
        # as a scrambled, saved map. Verifying that nothing was actually written cost real
        # bench time, twice (FINDINGS §66.2, ROADMAP §8.2 item 45).
        print(f"axis map {one.name} ({one.frame} frame): {one.axis_map.one_line(one.frame)}")
        if one.axis_map != one.axis_map_at_start:
            print(f"     was: {one.axis_map_at_start.one_line(one.frame)}")
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
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
