"""Teleop command-line definitions; no device acquisition or saved-default writes.

The application installs saved defaults and validates cross-option constraints
before acquiring any devices. Keep help text and accepted flags compatible.
"""
from __future__ import annotations

import argparse

from yam.can import ARM_SERIALS, DEFAULT_ARM
from yam.mirror import DEFAULT_CATCHUP, DEFAULT_MAX_GAP
from yam.recording import SCRUB_MAX_RATE
from yam.robot import SAFE_MAX_LAG, SAFE_MAX_SPEED
from yam.teleop import FLOOR_LIMIT, FRAMES, REACH_LIMIT


def build_parser(*, linear_scale: float, gripper_step: float,
                 planned_speed: float) -> argparse.ArgumentParser:
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
    ap.add_argument("--linear-scale", type=float, default=linear_scale)
    ap.add_argument("--gripper-step", type=float, default=gripper_step,
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
    ap.add_argument("--teleop-speed", type=float, default=planned_speed,
                    help=f"the ceiling on a PLANNED joint speed, rad/s (default "
                         f"{planned_speed}). ⛔ A SAFETY LIMIT, and a DIFFERENT one "
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
    return ap
