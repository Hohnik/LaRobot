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
   `q` moves to HOLD and waits for an explicit second key.

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
import sys
import time
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
from teleop import CartesianTeleop  # noqa: E402
from yam_can import ARM_SERIALS, DEFAULT_ARM, YAM_JOINTS  # noqa: E402
from yam_robot import (  # noqa: E402
    advance_park_command,
    build_robot,
    park_target_from,
    shutdown_robot,
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
# Judged on the MEASURED pose, so it must allow for a position controller's
# steady-state error. 0.02 rad is 1.1°, which is "arrived" for parking.
PARK_TOLERANCE = 0.02
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

HELP = """
  MODES     g GUIDE (weightless)   t TELEOP   h HOLD   p PARK   s save park pose
  DIRECTION x y z  flip translation axis      1 2 3  flip rotation axis (roll/pitch/yaw)
  CONTROLS  m  set up the mouse — the arm MOVES, one isolated axis, half speed
  SPEED     - / +  linear             , / .  rotation          [ / ]  gripper step
  GRIPPER   o open   c close          r  wrist rotation on/off
  OTHER     ?  this help              q  QUIT (asks before releasing the arm)
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


def map_reference() -> str:
    """What the six motions physically are. Measured in simulation, not assumed —
    see `src/axis_map.py` for the numbers and for why "forward" is not claimed."""
    lines = ["  the six motions, in the WORLD frame (they do not change when the wrist turns):"]
    for i, m in enumerate(ROBOT_MOTIONS):
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


def main() -> int:  # noqa: PLR0915
    ap = argparse.ArgumentParser(description="Interactive YAM session: guide, teleop, park.")
    ap.add_argument("--yes", action="store_true", help="actually energise the arm")
    ap.add_argument("--arm", default=DEFAULT_ARM, choices=sorted(ARM_SERIALS))
    ap.add_argument("--start-mode", default="guide", choices=["guide", "hold", "teleop"])
    ap.add_argument("--no-gripper", action="store_true",
                    help="run the 6 arm joints only and leave motor 7 free — the escape hatch if the "
                         "gripper misbehaves again")
    ap.add_argument("--no-rotation", action="store_true",
                    help="start with wrist rotation disabled (toggle live with r)")
    ap.add_argument("--linear-scale", type=float, default=LINEAR_SCALE)
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
    rotation = not args.no_rotation
    # ⛔ The store decides WHICH map this arm uses — its own override if it has one,
    # otherwise the shared one. Editing a shared map changes both arms, so the scope
    # is printed in the plan and again at exit. Never leave that implicit.
    map_store = AxisMapStore.load(MAP_FILE)
    if args.fork_map:
        map_store.fork(args.arm)
    elif args.share_map:
        map_store.unfork(args.arm)
    axis_map = map_store.for_arm(args.arm)
    map_store_at_start = map_store.copy()
    axis_map_at_start = axis_map.copy()
    park = load_json(PARK_FILE, {}).get(args.arm)
    angular_scale = ANGULAR_SCALE
    gripper_step = GRIPPER_STEP
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
    print(f"  axis map    : {axis_map.one_line()}   (m to change it live)")
    print(f"  map scope   : {map_store.scope_note(args.arm)}")
    if axis_map.unbound():
        names = ", ".join(ROBOT_MOTIONS[i]["short"] for i in axis_map.unbound())
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
    max_temp_seen = 0.0
    max_jaw_temp_seen = 0.0
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
            teleop = CartesianTeleop()
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
        t0 = time.perf_counter()
        next_report = 1.0

        with KeyReader() as keys:
            if not keys.enabled:
                print("⚠️  stdin is not a terminal — keys will not work. Ctrl-C still does.\n")
            print(f"⭐ MODE: {mode.upper()}\n")

            while True:
                loop_start = time.perf_counter()
                t = loop_start - t0

                # ---- 1. is the robot still there? -------------------------
                if not chain_alive(robot):
                    stop_reason = (
                        "the motor chain STOPPED — I2RT's control thread exited, almost "
                        "certainly on a motor fault. Commands are no longer reaching the arm."
                    )
                    break

                # ---- 2. temperatures --------------------------------------
                try:
                    states = chain.read_states()
                    temps = [max(getattr(s, "temp_mos", 0) or 0, getattr(s, "temp_rotor", 0) or 0)
                             for s in states]
                    hottest = max(temps)
                    max_temp_seen = max(max_temp_seen, hottest)
                    # ⭐ The GRIPPER'S OWN temperature, reported separately and not
                    # folded into `hottest`. Motors 2/3 carry the arm's 4.3 kg and sit
                    # at 41-42 °C in normal equilibrium, while an idle motor 7 is
                    # 31-36 °C — so a gripper climbing 33 → 41 °C is entirely hidden
                    # behind the shoulder in a max(). The whole point of the 2π frame
                    # fix is that motor 7 no longer heats, and a test that cannot see
                    # motor 7 cannot falsify that. FINDINGS §0: prefer the test that
                    # could disagree with you.
                    jaw_temp = temps[N_ARM] if len(temps) > N_ARM else None
                    if jaw_temp is not None:
                        max_jaw_temp_seen = max(max_jaw_temp_seen, jaw_temp)
                    if hottest >= TEMP_STOP:
                        stop_reason = (
                            f"motor {temps.index(hottest) + 1} reached {hottest:.0f}°C "
                            f"(limit {TEMP_STOP}°C) — stopping before the firmware trips"
                        )
                        break
                    # ---- gripper stall guard ------------------------------
                    # ⚠️ With NO_GRIPPER the chain has 6 motors, so states[6] would
                    # IndexError -- and the surrounding try/except would swallow it,
                    # silently killing temperature monitoring too. Guard explicitly.
                    jaw = states[N_ARM] if len(states) > N_ARM else None
                    if jaw is None:
                        stall_since = None
                        raise StopIteration
                    if (abs(getattr(jaw, "eff", 0.0)) > GRIPPER_STALL_TORQUE
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
                except StopIteration:
                    pass          # no gripper in the chain; temperatures already read
                except Exception:  # noqa: BLE001
                    temps, hottest, jaw_temp = [], 0.0, None

                # ---- 3. keys ----------------------------------------------
                for k in keys.drain():
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
                            print(axis_map.describe())
                            if k == "t":
                                mode = "teleop"; enter_teleop()
                                print("\n⭐ MODE: TELEOP — SpaceMouse drives, all axes\n")
                            elif k == "g":
                                mode = "guide"; enter_guide()
                                print("\n⭐ MODE: GUIDE — arm is weightless\n")
                            else:
                                mode = "hold"; enter_hold()
                                print("\n⭐ MODE: HOLD\n")
                        elif k == "b":
                            learn_button = "open"
                            print("\n⭐ LEARNING THE GRIPPER BUTTONS.")
                            print("   Press the puck button you want for OPEN …")
                            print("   (the masks are learned by pressing, never assumed — which")
                            print("    physical button sets which bit has never been measured)\n")
                        elif k == "f" and last_input_kind == "button":
                            # ⭐ Same key, same meaning: reverse the control just used.
                            axis_map.swap_buttons()
                            print("\n  ↔ SWAPPED the gripper buttons")
                            print(axis_map.buttons_row() + "\n")
                        elif k == "f":
                            if active is None:
                                print("\n  push the puck first — f reverses the control you just used.\n")
                            elif driven is None:
                                print(f"\n  puck {PUCK_AXES[active]} drives nothing, so there is no "
                                      f"direction to reverse. Press 1-6 to give it a motion.\n")
                            else:
                                axis_map.flip(driven)
                                print(f"\n  ↔ REVERSED → {axis_map.row(driven).strip()}"
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
                                    print(f"\n  ⇄ SWAPPED {ROBOT_MOTIONS[driven]['short']} ↔ "
                                          f"{ROBOT_MOTIONS[target]['short']}")
                                    print(f"      {axis_map.row(target).strip()}")
                                    print(f"      {axis_map.row(driven).strip()}")
                                    print("      (press the same key again to swap back)\n")
                                else:
                                    # The active control drove nothing, so there is nothing
                                    # to exchange with. The direction he was last pushing
                                    # becomes this motion's positive sense.
                                    displaced = axis_map.bind(target, active, last_active_value)
                                    print(f"\n  ✓ puck {PUCK_AXES[active]} now drives "
                                          f"{ROBOT_MOTIONS[target]['short']} → "
                                          f"{axis_map.row(target).strip()}")
                                    if displaced is not None:
                                        print(f"  ⚠️  {ROBOT_MOTIONS[displaced]['short']} was using that "
                                              f"control and is now UNBOUND — it will not move.")
                                    print()
                        elif k == "u":
                            if driven is None:
                                print("\n  that control already drives nothing.\n")
                            else:
                                axis_map.unbind(driven)
                                print(f"\n  unbound {ROBOT_MOTIONS[driven]['short']} — it will not move\n")
                        elif k == "0":
                            axis_map = axis_map_at_start.copy()
                            print("\n  reverted to the controls this session started with:")
                            print(axis_map.describe() + "\n")
                        elif k == "?":
                            print(map_reference())
                            print(MAP_HELP)
                            print(axis_map.describe() + "\n")
                        # ⚠️ The rotation pair was MISSING here while the linear pair was
                        # present, so in CONTROLS mode roll/pitch/yaw could not be sped up
                        # or slowed down at all — Julien found it on the arm. The keys were
                        # copied from the drive-mode handler and the second pair was
                        # dropped. Both scales are also printed in the status line now, so
                        # a key that silently does nothing is visible rather than inferred.
                        elif k in "+=":
                            args.linear_scale *= 1.25
                            print(f"\n  linear speed → {args.linear_scale:.3f} m/s\n")
                        elif k == "-":
                            args.linear_scale /= 1.25
                            print(f"\n  linear speed → {args.linear_scale:.3f} m/s\n")
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
                        print(map_reference())
                        print(MAP_HELP)
                        print(axis_map.describe())
                        print("\n  Push the puck one way at a time and watch the arm. If a direction is")
                        print("  wrong, press f. If a control should do something else, press 1-6.\n")
                        if not rotation:
                            print("  ⚠️  wrist rotation is OFF (r toggles) — ROLL/PITCH/YAW will not move.\n")
                    elif k == "g" and mode != "guide":
                        mode = "guide"; enter_guide(); print("\n⭐ MODE: GUIDE — arm is weightless\n")
                    elif k == "t" and mode != "teleop":
                        mode = "teleop"; enter_teleop(); print("\n⭐ MODE: TELEOP — SpaceMouse drives\n")
                    elif k == "h" and mode != "hold":
                        mode = "hold"; enter_hold(); print("\n⭐ MODE: HOLD\n")
                    elif k == "s":
                        q = np.asarray(robot.get_joint_pos(), dtype=float)
                        data = load_json(PARK_FILE, {}); data[args.arm] = q.tolist()
                        save_json(PARK_FILE, data); park = q.tolist()
                        print(f"\n  park pose saved: {np.round(q[:N_ARM], 3)}\n")
                    elif k == "p":
                        if park is None:
                            print("\n  no park pose saved — press s first\n")
                        else:
                            # ⛔ Build the target from the MEASURED pose and overlay only
                            # the joints the saved pose actually carries.
                            #
                            # A pose saved in a --no-gripper session has 6 entries; this
                            # robot may have 7 (or the reverse). `park_target - q` on
                            # mismatched lengths raises ValueError, and that exception
                            # escaped the loop entirely: it skipped the "the arm is
                            # HOLDING, press g or d" consent flow and went straight to
                            # `finally`, which DISABLES THE MOTORS — dropping a raised
                            # arm. And `--no-gripper` is exactly the escape hatch the
                            # gripper instructions tell you to fall back to, so the
                            # fallback path was the broken one. Found by reading the
                            # code, not by dropping an arm.
                            # ⛔ PARK was also the ONE path that bypassed clamp_gripper,
                            # so a pose saved with the jaws on a stop would be driven
                            # back onto it and HELD. Both defects are fixed inside
                            # park_target_from(), which is pure and has tests:
                            #   uv run scripts/test_park_target.py
                            park_target, warn = park_target_from(
                                robot.get_joint_pos(), park,
                                gripper_index=N_ARM, clamp=clamp_gripper,
                            )
                            if warn:
                                print(f"\n  ⚠️  {warn}.")
                            mode = "park"
                            enter_hold()
                            # Seed the trajectory at the arm's real pose, then let it run
                            # ahead. enter_hold() has just resynced SafeRobot, so the
                            # rate limiter starts anchored here too.
                            park_cmd = np.asarray(robot.get_joint_pos(), dtype=float)
                            park_best_err = float(np.max(np.abs(park_target - park_cmd)))
                            park_progress_t = t
                            print(f"\n⭐ MODE: PARK — driving to {np.round(park_target[:N_ARM], 2)} "
                                  f"at {PARK_SPEED} rad/s. Press h or t to stop.\n")
                    elif k == "o" and mode == "teleop":
                        gripper_value = clamp_gripper(gripper_value + gripper_step)
                    elif k == "c" and mode == "teleop":
                        gripper_value = clamp_gripper(gripper_value - gripper_step)
                    elif k == "]":
                        gripper_step = min(0.20, gripper_step * 1.5)
                        print(f"\n  gripper step → {gripper_step:.3f} per press\n")
                    elif k == "[":
                        gripper_step = max(0.002, gripper_step / 1.5)
                        print(f"\n  gripper step → {gripper_step:.3f} per press\n")
                    elif k == "r":
                        rotation = not rotation
                        print(f"\n  wrist rotation {'ON' if rotation else 'OFF'}\n")
                    elif k == ".":
                        angular_scale *= 1.25
                        print(f"\n  rotation speed → {angular_scale:.2f} rad/s\n")
                    elif k == ",":
                        angular_scale /= 1.25
                        print(f"\n  rotation speed → {angular_scale:.2f} rad/s\n")
                    elif k in "xyz":
                        # ⚠️ These now flip a ROBOT MOTION, not a puck axis. Under the
                        # identity map that is the same arithmetic, which is why the
                        # hand-dialled file still means what it meant. Under a
                        # permutation it is the only reading that stays useful: when
                        # Julien presses x he means "the gripper goes the wrong way",
                        # which is a statement about the arm, not about the device.
                        idx = "xyz".index(k)
                        axis_map.flip(idx)
                        print(f"\n  {ROBOT_MOTIONS[idx]['short']} flipped → "
                              f"{axis_map.row(idx).strip()}\n")
                    elif k in "123":
                        # Rotation motions: 1 roll, 2 pitch, 3 yaw. Digits because every
                        # sensible letter was taken, and because they read as an
                        # ordered triple the way x/y/z do.
                        idx = 3 + "123".index(k)
                        axis_map.flip(idx)
                        print(f"\n  {ROBOT_MOTIONS[idx]['short']} flipped → "
                              f"{axis_map.row(idx).strip()}\n")
                    elif k == "+" or k == "=":
                        args.linear_scale *= 1.25
                        print(f"\n  linear speed → {args.linear_scale:.3f} m/s\n")
                    elif k == "-":
                        args.linear_scale /= 1.25
                        print(f"\n  linear speed → {args.linear_scale:.3f} m/s\n")
                    elif k == "?":
                        print(HELP)
                    elif k.isprintable() and k.strip():
                        print(f"\n  (key {k!r} does nothing — press ? for the list)\n")
                if stop_reason:
                    break

                # ---- 4. act on the mode -----------------------------------
                if mode in ("teleop", "map") and teleop is not None:
                    raw_axes = reader.read()

                    # ---- puck buttons ------------------------------------
                    # ⚠️ reader.read() must be called first: it is what drains the
                    # HID reports, and the button state arrives on report 0x03
                    # inside that same drain.
                    buttons = getattr(reader, "buttons", 0)
                    pressed = buttons & ~buttons_prev          # rising edge only
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
                        # A button press counts as "the control you just used", so f
                        # reverses it — but ONLY the keys edit the map, exactly as
                        # for the axes. Pressing a button never rebinds anything.
                        last_input_kind = "button"
                        if axis_map.button_action(pressed) is None:
                            print(f"\n  button 0x{pressed:02x} is not assigned — press b to "
                                  f"set the gripper buttons\n")

                    if learn_button is None and robot.num_dofs() > N_ARM:
                        action = axis_map.button_action(buttons)
                        if action == "open":
                            gripper_value = clamp_gripper(gripper_value + GRIPPER_BUTTON_RATE * dt)
                        elif action == "close":
                            gripper_value = clamp_gripper(gripper_value - GRIPPER_BUTTON_RATE * dt)

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

                elif mode == "park" and park_target is not None and park_cmd is not None:
                    q = np.asarray(robot.get_joint_pos(), dtype=float)
                    # ⭐ Completion is judged from the MEASURED pose, never from the
                    # command — the command always arrives first, so testing it would
                    # declare success while the arm was still travelling.
                    err = float(np.max(np.abs(park_target - q)))
                    if err < PARK_TOLERANCE:
                        mode = "hold"; enter_hold()
                        print(f"\n⭐ PARK reached ({err:.3f} rad off) → HOLD\n")
                    elif t - park_progress_t > PARK_STALL_SECONDS:
                        # ⛔ Never spin silently. If the arm has stopped closing the gap
                        # the honest thing is to say so and hold, not to keep printing a
                        # number that is not changing — which is exactly how the old
                        # treadmill bug hid for two sessions.
                        mode = "hold"; enter_hold()
                        print(f"\n⛔ PARK STALLED — {err:.3f} rad still to go and no progress "
                              f"for {PARK_STALL_SECONDS:.0f}s.")
                        print(f"   The command ran {float(np.max(np.abs(park_cmd - q))):.3f} rad ahead of "
                              f"the arm; SafeRobot limited {getattr(robot, 'limited_cycles', 0)} cycles.")
                        print("   Something is blocking it, or the pose is unreachable. Now HOLDING.\n")
                    else:
                        # ⛔ Advance the COMMAND, not the measurement. See
                        # yam_robot.advance_park_command() for why this is the whole bug.
                        park_cmd = advance_park_command(park_cmd, park_target, PARK_SPEED * dt)
                        robot.command_joint_pos(park_cmd)
                        if err < park_best_err - PARK_PROGRESS_EPS:
                            park_best_err, park_progress_t = err, t
                        if t >= next_park_report:
                            next_park_report = t + 1.0
                            lead = float(np.max(np.abs(park_cmd - q)))
                            print(f"\r  parking… {err:.3f} rad to go, command {lead:.3f} ahead   ",
                                  end="", flush=True)

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
                            doing = f"→ {ROBOT_MOTIONS[drv]['short']} {unit}"
                        print(f"\r[CONTROLS] puck {PUCK_AXES[last_active_axis]:<5} "
                              f"{last_active_value:+.2f}  {doing:<28} {speeds}"
                              f"{' ' * 6}", end="", flush=True)
                elif t >= next_report:
                    next_report += 1.0
                    q = np.asarray(robot.get_joint_pos(), dtype=float)
                    extra = ""
                    if mode == "teleop" and teleop is not None:
                        extra = f"  EE {np.round(teleop.ee_position(), 3)}"
                    # ⭐ `jaw` is shown separately from `hottest` on purpose — see the
                    # comment where it is read. Watching this number plateau is the
                    # actual test of the 2π frame fix; watching `hottest` is not,
                    # because the shoulder sits hotter than the gripper all session.
                    therm = f"hottest {hottest:4.0f}°C"
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
                    print(f"\r[{'CONTROLS' if mode == 'map' else mode.upper():8}] t={t:6.1f}s  {therm}"
                          f"  q {np.round(q[:N_ARM], 2)}{extra}   ", end="", flush=True)

                time.sleep(max(0.0, dt - (time.perf_counter() - loop_start)))

            # ---- controlled shutdown -----------------------------------------
            print(f"\n\n⛔ stopping: {stop_reason}")
            if chain_alive(robot):
                enter_hold()
                print("\nThe arm is HOLDING its pose. It will not be released until you choose.")
                print("   g = go weightless so you can park it by hand")
                print("   d = disable now (⚠️ a raised arm will sag)")
                while True:
                    k = keys.get()
                    if k == "g":
                        enter_guide()
                        print("\n⭐ weightless — park the arm, then press d to disable.")
                    elif k == "d":
                        break
                    time.sleep(0.05)
                    if not chain_alive(robot):
                        print("\n⚠️  the chain died while waiting — disabling now.")
                        break
            else:
                print("⚠️  the chain is already dead, so the arm is NOT being commanded.")
                print("   It will be sagging under gravity. Support it now if it is raised.")

    except KeyboardInterrupt:
        print("\ninterrupted.")
    except Exception as exc:  # noqa: BLE001
        print(f"\n⛔ {type(exc).__name__}: {exc}")
    finally:
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
        map_store.set(args.arm, axis_map)
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
            disabled = shutdown_robot(robot)
            print(f"\nmotors confirmed disabled: {disabled}")

    print(f"\nhottest motor seen this session: {max_temp_seen:.0f}°C")
    if max_jaw_temp_seen:
        # The number that decides whether the gripper frame fix held. A plateau near
        # idle (31-36 °C) is the pass; a steady climb is the failure, and it is
        # invisible in `hottest` because the shoulder runs hotter all session.
        print(f"hottest the GRIPPER (motor 7) got: {max_jaw_temp_seen:.0f}°C")
    print(f"axis map: {axis_map.one_line()}")
    if axis_map != axis_map_at_start:
        print(f"     was: {axis_map_at_start.one_line()}")
    else:
        print("     unchanged — nothing was written.")
    if axis_map.unbound():
        names = ", ".join(ROBOT_MOTIONS[i]["short"] for i in axis_map.unbound())
        print(f"  ⚠️  UNBOUND, the arm will not perform these: {names}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
