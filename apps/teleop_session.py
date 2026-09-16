#!/usr/bin/env python3
"""Interactive YAM operator session.

Run ./teleop --help for flags and docs/COMMANDS.md for keys. Without --yes
the program prints its plan and opens no devices. --sim uses fake arms and
still pucks. Physical operation needs a clear workspace and an operator.

This application coordinates acquisition, the shared control loop and prompts.
ArmSession owns per-arm modes; RecordingSession owns take completion;
PlaybackSession and CompositeRun own replay state and sequencing. The controlled
stop interaction lives in yam.lifecycle; SessionResources owns acquired handles.
Command-line definitions live in yam.teleop_cli; incident fields in yam.incident.
Historical source notes are in docs/archive/teleop-source-notes.md.
"""

from __future__ import annotations

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
from yam.incident import describe, write_incident, session_facts, safe_fact as _safe_fact  # noqa: E402
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
from yam.teleop_cli import build_parser  # noqa: E402
from yam.session_health import SessionHealth  # noqa: E402
from yam.session_input import SessionInput  # noqa: E402
from yam.session_resources import SessionResources  # noqa: E402
from yam.recording_session import RecordingSession  # noqa: E402
from yam.lifecycle import StopCause, StopRequest, controlled_stop  # noqa: E402
from yam.composite import CompositeRun  # noqa: E402
from yam.mirror_session import MirrorSession  # noqa: E402
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
from yam.ui.recording_prompt import RecordingPrompt, save_slot_action  # noqa: E402
from yam.ui.controls_editor import edit_controls  # noqa: E402
from yam.ui.playback_prompt import PlaybackChoice, PlaybackPrompt  # noqa: E402
from yam.ui.park_prompt import ParkAction, ParkPrompt  # noqa: E402
from yam.ui.session_plan import session_plan_lines  # noqa: E402
from yam.ui.settings_panel import SettingsAction, SettingsPanel  # noqa: E402
from yam.settings import (  # noqa: E402
    LIVE_ORDER,
    adjust as adjust_setting,
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
                          (1=X 2=Y 3=UP 4=ROLL 5=PITCH 6=YAW). Neither is left unbound.
                          The message names the key that swaps this pair back
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


def main() -> int:  # noqa: PLR0915
    ap = build_parser(linear_scale=LINEAR_SCALE, gripper_step=GRIPPER_STEP,
                      planned_speed=MAX_PLANNED_JOINT_SPEED)

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
    # Display the effective scrub rate over the last reporting window.
    scrub_ref_t = scrub_ref_s = scrub_ref_h = 0.0
    # Capture each arrival's purpose before advancing a composite leg (FINDINGS §72.1).
    park_purpose: dict[str, str] = {}
    health = SessionHealth(n_arm=N_ARM, stall_torque=GRIPPER_STALL_TORQUE,
                           stall_velocity=GRIPPER_STALL_VEL, stall_seconds=GRIPPER_STALL_SECONDS,
                           emit=print)
    mirror = MirrorSession(align_speed=MIRROR_ALIGN_SPEED, n_arm=N_ARM, emit=print, hint=hint)
    # A pending `s` or `p` waiting for its digit, and the sequence being typed after `p`.
    pending: str | None = None
    park_prompt = ParkPrompt()
    angular_scale = ANGULAR_SCALE
    gripper_step = args.gripper_step
    # The active control and button binding belong to the selected ArmSession.

    for line in session_plan_lines(
            args, arm_names, map_store, saved_slots, angular_scale=ANGULAR_SCALE,
            base_slot=BASE_SLOT, planned_speed_default=MAX_PLANNED_JOINT_SPEED,
            temp_warn=TEMP_WARN, temp_stop=TEMP_STOP):
        print(line)
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
    resources = SessionResources(recording, shutdown_robot=shutdown_robot, emit=print)
    stop: StopRequest | None = None
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
            resources.own_capture(capture)

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
            resources.own_puck(handle)
            pucks[name] = {"path": info["path"], "handle": handle}
            handle.set_nonblocking(True)
            pucks[name]["reader"] = TwistReader(handle)

        # Initialized ArmSessions drive the loop; resources owns every acquired handle,
        # including robots whose session failed to initialize.
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
        recording_prompt = RecordingPrompt(recording, takes_dir, saver=save_take, emit=print)
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
            resources.own_robot(name, robot, motors=n_motors)
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
            seq = park_prompt.shown if park_prompt.entries else "0"
            name, radius = BLEND_MODES[blend_idx]
            # ⭐ A grab is visible BEFORE Enter (ROADMAP §6.6.2 item 4): a leg where only
            # the jaws move splits the run and pauses it, and the count says so here,
            # while the sequence is still being typed.
            # ⚠️ Take legs (`w<digit>`, ROADMAP §6.6.1a) are counted separately: their
            # jaw motion is whatever the hand taught, so no stop-counting applies.
            poses = [e for e in (park_prompt.entries or ["0"]) if not e.startswith("w")]
            takes = [e[1:] for e in park_prompt.entries if e.startswith("w")]
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

        inputs = SessionInput(shared_reader=shared_puck, selection=selection, n_arm=N_ARM,
                              button_rate=GRIPPER_BUTTON_RATE, clamp=clamp_gripper, emit=print)
        playback_prompt = PlaybackPrompt(playback, load_take=load_take, hint=hint, emit=print)

        composite = CompositeRun(load_take=load_take, begin_path=begin_path,
                                 start_take=start_take, emit=print,
                                 clear_hint=lambda: hint(""))

        # The panel owns selection and reset values; live-device application stays here.
        def apply_live(name: str, value: float) -> None:
            setattr(args, name, value)
            for one_arm in arms:
                if name == "max_speed":
                    one_arm.robot.max_speed = value
                elif name == "max_lag":
                    one_arm.robot.max_lag = value
                elif name == "vel_ff":
                    one_arm.robot.vel_ff = value
            if name == "mirror_gap" and mirror.link is not None:
                mirror.link.max_gap = value
            if name == "mirror_catchup" and mirror.link is not None:
                mirror.link.catchup = value

        def show_settings_status() -> None:
            for one_arm in arms:
                print("     " + status_row(
                    one_arm, "", args.reach, args.floor,
                    note=(mirror.link.status(
                        mirror.leader.robot.get_joint_pos(),
                        one_arm.robot.get_joint_pos())
                        if mirror.link is not None
                        and one_arm.mode == "mirror" else "")))

        settings_panel = SettingsPanel(
            values=lambda: {name: getattr(args, name) for name in LIVE_ORDER},
            apply=apply_live, save=lambda: save_defaults(settings_file, effective_settings(args)),
            show_status=show_settings_status, emit=print, builtin=builtin_defaults,
            settings_file=settings_file, mode_keys=MODE_KEYS)

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
                    stop = StopRequest(
                        StopCause.INTERRUPT,
                        "Ctrl-C — the loop is stopping, and the arm is NOT "
                        "released until you choose below (Ctrl-C again forces it)")
                    break

                stop = health.check(arms, loop_start, stop=stop)
                if stop:
                    break

                recording_prompt.poll()

                # ---- 3. keys ----------------------------------------------
                for k in keys.drain():
                    if recording_prompt.handle(k):
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

                    if pending == "settings":
                        action = settings_panel.handle(k)
                        if action is not SettingsAction.STAY:
                            pending = None
                        if action is SettingsAction.QUIT:
                            stop = StopRequest(StopCause.QUIT, "quit requested")
                        continue

                    if pending == "playback":
                        choice = playback_prompt.handle(k, args.teleop_speed)
                        if choice is not PlaybackChoice.STAY:
                            pending = None
                        if choice is PlaybackChoice.START:
                            park_to_take_start()
                            if playback.scrub:
                                print("     then the PUCK scrubs it: push forward to play, pull "
                                      "back to rewind,\n     let go to freeze. h or t ends it.\n")
                            else:
                                print("     then it plays the recording. Press h or t to stop.\n")
                        continue

                    if pending == "mirror_go":
                        keep_prompt, args.mirror = mirror.confirm(
                            k, mode=args.mirror, catchup=args.mirror_catchup,
                            max_gap=args.mirror_gap, max_speed=args.max_speed)
                        if not keep_prompt:
                            pending = None
                        continue

                    if pending == "park":
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
                            # Adjust ease length independently of its shape while choosing a park.
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
                        choice = park_prompt.handle(k)
                        if choice.action is ParkAction.TAKE_DIGIT:
                            hint(f"  park sequence: {park_prompt.shown}   "
                                 "▶ next digit names a RECORDING to play as a leg")
                        elif choice.action is ParkAction.UPDATE:
                            hint(f"  park sequence: {park_prompt.shown}"
                                 "   (another digit, w+digit for a take, or Enter)")
                        elif choice.action is ParkAction.CONFIRM:
                            hint(park_plan_line(edit_arm))
                        else:
                            pending = None
                            if choice.action is ParkAction.CANCEL:
                                hint("")
                                print("\n  run cancelled.\n" if choice.confirmed
                                      else "\n  park cancelled.\n")
                            elif any(entry.startswith("w") for entry in choice.entries):
                                composite.begin(choice.entries, aimed)
                            else:
                                # Resolve each arm's own slots; an empty slot never substitutes base.
                                ran = False
                                for one in aimed:
                                    legs, missing = resolve_park_legs(
                                        choice.entries, one.base_pose, one.slots)
                                    if missing:
                                        if choice.confirmed:
                                            print(f"\n  ⚠️  arm {one.name}: skipping empty slot(s) "
                                                  f"{', '.join(missing)}.\n")
                                        else:
                                            print(f"\n  ⚠️  arm {one.name}: nothing saved in slot "
                                                  f"{', '.join(missing)} — press s then that digit "
                                                  "to record one.\n")
                                    if legs:
                                        label = (" → ".join(n for n, _ in legs) if choice.confirmed
                                                 else f"slot {legs[0][0]}")
                                        begin_path(one, legs, label)
                                        ran = True
                                if not ran:
                                    print("\n  nothing to park to.\n")
                        continue

                    # Button assignment belongs to the device and works in every mode.
                    if k == "b":
                        edit_arm.learn_button = "open"
                        print(f"\n⭐ LEARNING THE GRIPPER BUTTONS for arm {edit_arm.name}.")
                        print("   Press the puck button you want for OPEN …")
                        print("   (learned by pressing, never assumed — which physical button")
                        print("    sets which HID bit has never been measured on this unit)\n")
                        continue
                    if k == "a":
                        # Selection determines which arms mode keys affect and remains available in every mode.
                        if any(other.mode == "map" for other in arms):
                            # Keep the controls wizard bound to its original arm until it finishes.
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
                        # Mirror engagement moves the follower; show the plan and require a second confirmation.
                        if mirror.link is not None:
                            mirror.turn_off(arms)
                            continue
                        if mirror.preview(arms, selection.names(), args.mirror):
                            pending = "mirror_go"
                        continue
                    if k == "n":
                        pending = "settings"
                        settings_panel.show()
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

                    # CONTROLS owns the keyboard: 1-6 exchange mappings here, rather than flipping rotation.
                    # Puck deflection still drives the isolated motion at reduced speed; keys edit the map.
                    if wizard is not None:
                        # ⛔ EVERY EDIT IN THIS BRANCH IS KEY-DRIVEN. Moving the puck
                        # must never change the map — see FINDINGS §11 for what
                        # happened when it did.
                        if k == "q":
                            stop = StopRequest(StopCause.QUIT, "quit requested")
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
                        elif edit_controls(wizard, k, emit=print):
                            pass
                        elif k == "?":
                            print(map_reference(wizard.frame))
                            print(MAP_HELP)
                            print(wizard.axis_map.describe(wizard.frame) + "\n")
                        # Allow both linear and angular speed adjustments in CONTROLS; display both scales.
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
                        stop = StopRequest(StopCause.QUIT, "quit requested")
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
                                    pending = None
                                    recording_prompt.open()
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
                                recording_prompt.discard()
                                pending = None
                                print("\n  nothing recorded (too short) — finishing discard.\n")
                            else:
                                pending = None
                                recording_prompt.open()
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
                        pending = "playback"
                        playback_prompt.open()
                        # Show live and simulated shelves distinctly. listing() filters macOS sidecars.
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
                        park_prompt.open()
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
                        # Flip the requested robot motion, independent of the current puck-axis permutation.
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
                        # Speed keys adjust park speed in PARK and cursor pace during scrub.
                        # Scrub remains subject to replay lag gating and robot command limits.
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
                mirror.observe_modes(arms)

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
                if stop:
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
                        pending = None
                        recording_prompt.open()
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
                            pending = None
                            recording_prompt.open()
                            print(f"\n⏹  RECORDING STOPPED at the {MAX_TAKE_SAMPLES} sample "
                                  f"limit ({recording.pending.duration:.0f}s).")
                            for line in slot_overview(takes_dir):
                                print(line)
                            print("\n     SAVE to which slot? 0-9, any other key discards.\n")

                stop = inputs.poll(
                    arms, dt, prompt_open=pending is not None or recording_prompt.active, stop=stop)

                # Step each arm's mode in requested layout order, independent of key selection.
                for one in arms:
                    if one.mode in ("teleop", "map") and one.teleop is not None:

                        if one.mode == "map":
                            # CONTROLS drives only the strongest puck direction at reduced speed.
                            # Deflection observes the mapping; only explicit keys edit it.
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

                    elif one.mode == "mirror" and mirror.link is not None:
                        # ⭐⭐ ONE ARM FOLLOWS THE OTHER. Every decision is `MirrorLink`'s
                        # (18 tests, no robot handle); this branch reads the two poses,
                        # carries the command out, and narrates. Same split as `replay_step`
                        # and `ArmSession` — the code that commands an arm is the code that
                        # cannot be tested without one, so it is kept as thin as possible.
                        lead_q = np.asarray(mirror.leader.robot.get_joint_pos(), dtype=float)
                        follow_q = np.asarray(one.robot.get_joint_pos(), dtype=float)
                        cmd = mirror.link.step(lead_q, follow_q, real_dt)
                        if cmd is None:
                            # When mirror stops itself, stop follower commands and report its measured reason.
                            mirror.report_stop(one, args.max_speed)
                            one.enter_hold(); hint("")
                            mirror.clear()
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
                        # ArmSession.step_path owns path progress and its verdict; this branch handles feedback and handovers.
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
                            # Clear an arrived path so the generic PARK-abandonment check cannot cancel a pending replay.
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

                # Advance one shared playback clock per cycle for every recorded arm.
                # Follow recorded time; lag gating may pause that clock while arms catch up.
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
                        # Separate elapsed replay time from clock-hold time in the completion report.
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
                            # Measured tracking data is not a controlled speed sweep: pose and lag holds affect it.
                            # Use one arm-qualified joint-name list for both the printed table and saved report.
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


                # CONTROLS renders continuously and owns the display; other arms' rows are hidden until it closes.
                # Normal status renders once per second (FINDINGS §54.2).
                wizard = next((one for one in arms if one.mode == "map"), None)
                if wizard is not None:
                    # Keep the wizard reference separate from acquisition state. Display both speed scales.
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
                                   note=(mirror.link.status(
                                       mirror.leader.robot.get_joint_pos(),
                                       one.robot.get_joint_pos())
                                       if mirror.link is not None and one.mode == "mirror"
                                       else ""))
                        for i, one in enumerate(arms)])

                time.sleep(max(0.0, dt - (time.perf_counter() - loop_start)))

            # ---- controlled shutdown -----------------------------------------
            screen.done()
            print(f"\n⛔ stopping: {stop}")
            # ⭐ Printed for every session, not only a playback, because the worst pass is the one number about this loop that nothing recorded until 2026-08-20. ⚠️ On `--sim` it is Python-side jitter only: a fake arm answers in microseconds and a real one is 14 motors over two USB adapters ([FINDINGS §76.12](../docs/FINDINGS.md)).
            if loop_timer.count:
                print(f"⭐ {loop_timer.line()}")

            if stop is None:
                stop = StopRequest(StopCause.FAULT, "control loop ended without a stop cause")
            exit_code = stop.exit_code
            controlled_stop(stop, arms, keys,
                            lambda live: park_arms(live, keys, clamp_gripper), emit=print)

    except KeyboardInterrupt:
        exit_code = 130
        stop = StopRequest(StopCause.INTERRUPT, "interrupted during startup or shutdown")
        print("\ninterrupted.")
    except Exception as exc:  # noqa: BLE001
        exit_code = 1
        stop = StopRequest(StopCause.FAULT, f"{type(exc).__name__}: {exc}")
        print(f"\n⛔ {stop}")
        for note in getattr(exc, "__notes__", []):
            print(f"   {note}")
    finally:
        # Snapshot initialized sessions before disabling their underlying handles.
        # The cleanup list also includes robots whose session never initialized.
        alive_at_teardown = {one.name: _safe_fact(lambda one=one: bool(one.alive()))
                             for one in arms}
        if resources.robot_names:
            _SHUTTING_DOWN["yes"] = True
        if not resources.close():
            exit_code = 1
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
        if resources.robot_names:
            # Write incident facts after attempting device shutdown. Capture cached state
            # with guarded reads so incomplete startup or a dead chain cannot hide the original fault.
            try:
                bad_stop = exit_code != 0 or (stop is not None and stop.cause is not StopCause.QUIT)
                # Use the last available cycle values. Fresh reads from a failed device can raise
                # and would erase the evidence of the state before shutdown.
                if bad_stop:
                    facts = session_facts(
                        arms, stop=stop, arm_names=arm_names,
                        acquired_robots=resources.robot_names, reach=args.reach, floor=args.floor,
                        loop_hz=lambda: round(loop_hz, 1), loop_timing=lambda: loop_timer.to_dict(),
                        alive_at_teardown=alive_at_teardown)
                    print("\n" + describe(write_incident(str(stop) if stop is not None else "unknown", facts)))
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
