"""Saved session defaults, so the flags do not have to be typed every time.

⭐⭐ WHY THIS EXISTS. Julien, 2026-08-17: *"all of these flags should be default options
that can be changed in some controls mode and then should be saved so that I don't always
have to run with all of the flags, and I can change the default mode."*

⛔ The concrete cost he was paying. His working command had grown to:

    uv run scripts/teleop_session.py --yes --arms B,G --start-mode hold \\
        --max-speed 2 --teleop-speed 2 --mirror-gap 0.6

⚠️ **And the flag he omitted is the one that cost him three sessions.** On 2026-08-17 he
raised `--max-lag` three times chasing a mirror that kept stopping, while `--mirror-gap` sat
at its built-in 0.35 because it was not in the command he had pasted. **A long command line
is not just tedious; it is a place for a critical setting to go missing in silence.**

⭐ HOW IT WORKS, and the precedence is the whole design:

    built-in constant   →   config/session_defaults.json   →   command-line flag

Each layer overrides the one before it. So a saved default replaces the constant, and an
explicit flag still wins over the saved default for one run. ⛔ **That order matters for
safety**: a flag typed deliberately must never be silently overridden by a file.

⭐ Writing them: run the session once with the flags you want, adding `--save-defaults`, and
those values become the new baseline. ⚠️ **It saves the EFFECTIVE values**, which is what
the session actually ran with, so what gets saved is what was just proven to work.

⛔⭐ SAFETY LIMITS ARE STILL HIS TO SET, AND SAVING ONE IS A DELIBERATE ACT. `--max-speed`,
`--max-lag`, `--reach` and `--floor` bound how fast and how far 4.3 kg may move. Nothing
here changes a default on its own; the file only ever holds what he explicitly asked to
save. ⚠️ A saved value that is *looser* than the built-in constant is flagged in the
session plan, so a permanent loosening can never become invisible.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

#: The settings that may be saved, keyed by their argparse `dest`.
#:
#: ⛔ DELIBERATELY NOT EVERYTHING. `--yes` is absent because energising the motors must be a
#: conscious act on every single run, and a saved `yes: true` would turn a dry run into a
#: live one for anyone who forgot the file existed. `--arms` is absent because which arms
#: are plugged in changes between sessions and a stale saved value would try to build an arm
#: that is not there. `--save-defaults` and `--sim` are absent because they describe one run.
#:
#: ⚠️ Each entry is `(type, is_a_safety_limit, "looser means")`. The third field says which
#: DIRECTION is the permissive one, so the plan can warn about a saved loosening. `None`
#: means the setting is a preference rather than a bound.
TUNABLE: dict[str, tuple[type, bool, str | None]] = {
    "start_mode": (str, False, None),
    "frame": (str, False, None),
    "max_speed": (float, True, "up"),
    "teleop_speed": (float, True, "up"),
    "max_lag": (float, True, "up"),
    "mirror_gap": (float, True, "up"),
    "reach": (float, True, "up"),
    "floor": (float, True, "down"),
    "linear_scale": (float, False, None),
    "gripper_step": (float, False, None),
    "no_rotation": (bool, False, None),
    "no_smooth": (bool, False, None),
    "no_gripper": (bool, False, None),
    #: ⭐ Not a bound but a control gain, so `is_a_safety_limit` is False. ⚠️ It still changes
    #: what the arm does, which is why it defaults to OFF rather than to a value.
    "mirror_catchup": (float, False, None),
    #: ⭐ Velocity feedforward gain, 0..1 (item 44): the motors receive this fraction of the
    #: rate-limited command's own derivative as their velocity setpoint. Same family as
    #: `mirror_catchup`: a gain, defaults OFF, and 0 must stay reachable by key.
    "vel_ff": (float, False, None),
    #: ⭐ The scrub's full-push pace, in recording-seconds per second — his time-lapse dial
    #: (2026-08-18). Adjusted with -/+ DURING a scrub; not on the `n` screen (its keys are
    #: full, and the dial only means anything while a scrub is running). Safe high: a fast
    #: cursor is held back by the lag hold, so only the clock is fast, never the arm.
    "scrub_max": (float, False, None),
}


def defaults_path(repo: Path) -> Path:
    return repo / "config" / "session_defaults.json"


def load_defaults(path: Path) -> dict[str, Any]:
    """The saved settings, or `{}`. Never raises.

    ⛔⭐ A BROKEN OR MISSING FILE MUST NOT STOP A SESSION, and must not silently become an
    empty set of settings either. An unreadable file returns `{}` and the caller says so in
    the plan, so the arm still runs on its built-in constants and the operator is told the
    file was ignored rather than left to assume it applied.

    ⚠️ Unknown keys are dropped rather than passed through. A typo in the file would
    otherwise become an argparse `dest` that nothing reads, and the setting would look saved
    while doing nothing.
    """
    try:
        raw = json.loads(path.read_text())
    except (OSError, ValueError):
        return {}
    if not isinstance(raw, dict):
        return {}
    out: dict[str, Any] = {}
    for key, (kind, _, _) in TUNABLE.items():
        if key not in raw:
            continue
        value = raw[key]
        # ⚠️ Checked rather than coerced. `float("fast")` raises and `bool("false")` is
        # True, so a wrong type in the file must be rejected, not converted.
        if kind is bool and isinstance(value, bool):
            out[key] = value
        elif kind is float and isinstance(value, (int, float)) \
                and not isinstance(value, bool):
            out[key] = float(value)
        elif kind is str and isinstance(value, str):
            out[key] = value
    return out


def rejected_keys(path: Path) -> list[str]:
    """Keys in the file that `load_defaults` refused, so the plan can say so.

    ⭐ Silence here is the failure mode this exists to prevent: a misspelled or wrongly
    typed setting looks saved and does nothing.
    """
    try:
        raw = json.loads(path.read_text())
    except (OSError, ValueError):
        return []
    if not isinstance(raw, dict):
        return []
    good = load_defaults(path)
    return sorted(k for k in raw if k not in good)


def looser_than_builtin(saved: dict[str, Any], builtin: dict[str, Any]) -> list[str]:
    """Which SAVED safety limits are more permissive than the built-in constant.

    ⛔⭐⭐ THE POINT OF THIS FUNCTION IS THAT A PERMANENT LOOSENING CANNOT BE INVISIBLE. A
    flag typed on the command line is visible in the shell history and on screen. A saved
    default is not, so without this a session could run at three times the built-in speed
    limit with nothing on screen saying why.
    """
    out = []
    for key, (kind, is_limit, looser) in TUNABLE.items():
        if not is_limit or key not in saved or key not in builtin or kind is not float:
            continue
        if looser == "up" and saved[key] > builtin[key]:
            out.append(key)
        elif looser == "down" and saved[key] < builtin[key]:
            out.append(key)
    return sorted(out)


def effective(args: Any) -> dict[str, Any]:
    """The values this run actually used, ready to be saved."""
    return {key: getattr(args, key) for key in TUNABLE if hasattr(args, key)}


def save_defaults(path: Path, values: dict[str, Any]) -> None:
    """Write the settings, sorted, with a note for whoever opens the file by hand."""
    path.parent.mkdir(parents=True, exist_ok=True)
    body = {
        "_comment": ("Session defaults for scripts/teleop_session.py. A command-line flag "
                     "still overrides anything here. Written by --save-defaults; safe to "
                     "edit by hand. Delete a key to go back to the built-in constant, or "
                     "delete the file for all of them."),
        **{k: values[k] for k in sorted(values)},
    }
    path.write_text(json.dumps(body, indent=2) + "\n")


def describe(saved: dict[str, Any], rejected: list[str], loose: list[str],
             path: Path, builtin: dict[str, Any] | None = None) -> list[str]:
    """Plan lines for the session's startup banner. Empty when no file is in play.

    ⭐ ONLY THE SETTINGS THAT ACTUALLY DIFFER FROM THE BUILT-IN CONSTANT ARE LISTED. The
    first version printed all thirteen, which came to a 280-character line reporting things
    like `frame=world` when `world` is the built-in value anyway. ⚠️ **A banner that lists
    everything is a banner nobody reads**, and the whole purpose of this line is that a
    saved change cannot hide.
    """
    if not saved and not rejected:
        return []
    changed = {k: v for k, v in saved.items()
               if builtin is None or k not in builtin or v != builtin[k]}
    body = (", ".join(f"{k}={changed[k]}" for k in sorted(changed)) if changed
            else "all matching the built-in values")
    out = [f"  defaults    : {path.parent.name}/{path.name} — {body}"]
    if loose:
        out.append("                ⚠️ SAVED LOOSER THAN BUILT-IN: "
                   + ", ".join(loose)
                   + "  (a permanent loosening — --save-defaults again to change it)")
    if rejected:
        out.append("                ⛔ IGNORED (unknown or wrong type): "
                   + ", ".join(rejected))
    return out


# ---------------------------------------------------------------- the live editor

#: ⭐⭐ THE SIX SETTINGS THE `n` KEY EDITS, in the order the digits 1-6 select them.
#:
#: Julien, 2026-08-17, on why a live editor is the right shape: *"How would the live key
#: change be one that I didn't type myself? That does not make any sense to me if I'm in a
#: control mode changing the key. And mainly, this is for saving which default value I wanna
#: change, because if I change controls, normally they always save in the live program."*
#:
#: ⭐ He is right, and the axis map is the precedent: it is edited live with keys and written
#: to `config/spacemouse_map.json`. A keypress he makes IS him typing the value.
#:
#: ⚠️ Only these six. The booleans and the frame have their own keys already (`r`, `v`), and
#: `start_mode` cannot be changed retroactively for a session that has started.
#: ⚠️⭐ `linear_scale` IS ON THIS LIST BECAUSE HE ASKED WHY IT WAS NOT. His words, 2026-08-17:
#: *"it seemed like the teleop speed was defined once in the settings here, but then also
#: defined in the normal mode where I did the linear speed, which doesn't make a lot of sense
#: because if it's limited in the linear normal mode, then why would it not allow me to change
#: it in the control panel area?"* ⭐ He is right: it is one of the three limits in series, and
#: leaving it off this screen made it the invisible one.
LIVE_ORDER = ("max_speed", "teleop_speed", "max_lag", "mirror_gap", "reach", "floor",
              "mirror_catchup", "linear_scale", "vel_ff")

#: ⛔⭐ BOUNDS FOR THE LIVE EDITOR, and every one is a backstop rather than a policy. A key
#: that repeats when held reached `lin 19.852 m/s` on 2026-08-17 because the linear-speed
#: keys had no ceiling at all. ⚠️ Generous on purpose: these must never be the reason a
#: setting will not go where he wants it, only the reason a stuck key cannot run away.
LIVE_BOUNDS: dict[str, tuple[float, float]] = {
    "max_speed": (0.1, 20.0),
    "teleop_speed": (0.1, 20.0),
    "max_lag": (0.05, 3.0),
    "mirror_gap": (0.05, 6.0),
    "reach": (0.15, 1.2),
    #: ⚠️ The floor may legitimately go BELOW zero — Julien uses `--floor -0.005` for a flat
    #: object on the desk — so its lower bound is negative rather than a small positive.
    "floor": (-0.15, 0.5),
    #: ⭐ 0 is OFF and must stay reachable, so this is the one setting whose floor is zero and
    #: whose step has to be able to get there. `adjust` special-cases it for that reason.
    "mirror_catchup": (0.0, 20.0),
    "linear_scale": (0.03, 15.0),
    #: ⭐ 1.0 sends exactly the command's own derivative, the physically-motivated value.
    #: ⚠️ The headroom above 1.0 is EXAGGERATION, added at Julien's ask on 2026-08-18
    #: (*"exaggerate the numbers so that I can actually see what's happening"*) after 0.25
    #: felt real but subtle. Up there the motor is told the target moves faster than it
    #: does — expect overshoot; the position command stays rate-limited underneath.
    #: ⛔ Must equal `yam_robot.VEL_FF_CEILING`; a test pins the sync.
    "vel_ff": (0.0, 3.0),
    "scrub_max": (0.25, 8.0),
}

#: ⛔⭐⭐⭐ A LADDER OF ROUND NUMBERS, BECAUSE THE RATIO STEP PRODUCED UNUSABLE VALUES.
#:
#: The first version multiplied by 1.25 per press, on the reasoning that 0.1 → 0.125 and
#: 8 → 10 are the same *felt* step. That reasoning is fine and the result was not. Julien's
#: 2026-08-17 session shows what he actually got by holding `+`:
#:
#:     1.000 → 1.250 → 1.562 → 1.953 → 2.441 → 3.052 → 3.815 → 4.768 → 5.960 → 7.451 → 9.313
#:
#: ⛔ **He can never get back to 2, or 4, or 10.** Every value he had been running by flag was
#: unreachable by key, and the numbers then leaked into messages as
#: `limit 1.33514404296875` and `--max-speed 16 (now 7.45058)`.
#:
#: ⭐ So the keys walk a ladder instead. Each press moves one rung, every rung is a number a
#: person would choose, and the flag values he has actually used (1, 1.5, 2, 4, 10) are all on
#: it. ⚠️ A value set by flag that is BETWEEN rungs still works: the first press moves to the
#: nearest rung in the direction pressed, so nothing is ever stuck.
LADDERS: dict[str, tuple[float, ...]] = {
    "max_speed": (0.25, 0.5, 1.0, 1.5, 2.0, 3.0, 4.0, 6.0, 8.0, 10.0, 15.0, 20.0),
    "teleop_speed": (0.25, 0.5, 1.0, 1.5, 2.0, 3.0, 4.0, 6.0, 8.0, 10.0, 15.0, 20.0),
    "max_lag": (0.1, 0.15, 0.25, 0.4, 0.6, 0.8, 1.0, 1.5, 2.0, 3.0),
    "mirror_gap": (0.1, 0.2, 0.35, 0.5, 0.75, 1.0, 1.5, 2.0, 3.0, 4.0, 6.0),
    "reach": (0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.2),
    "floor": (-0.1, -0.05, -0.02, -0.01, -0.005, 0.0, 0.005, 0.01, 0.02, 0.05, 0.1),
    "mirror_catchup": (0.0, 1.0, 2.0, 3.0, 5.0, 8.0, 12.0, 20.0),
    "vel_ff": (0.0, 0.25, 0.5, 0.75, 1.0, 1.5, 2.0, 3.0),
    #: ⭐ 1.0 = the recording's own pace; 8.0 = an 8x time lapse for skimming to a moment.
    "scrub_max": (0.25, 0.5, 1.0, 1.5, 2.0, 3.0, 5.0, 8.0),
    #: ⭐ The CARTESIAN speed a full puck deflection asks for. Its default is 0.12 m/s and it
    #: spans two orders of magnitude, so the low rungs are fine and the high ones are coarse.
    "linear_scale": (0.03, 0.06, 0.12, 0.2, 0.3, 0.5, 0.8, 1.2, 2.0, 3.0, 5.0, 8.0, 12.0, 15.0),
}


def adjust(name: str, value: float, up: bool) -> float:
    """One press of `+` or `-` on setting `name`: the next rung of its ladder.

    ⭐ A value that sits BETWEEN rungs — because it came from a flag — moves to the nearest
    rung in the direction pressed rather than jumping past it. ⚠️ Without that, a flag value
    of 7 would step to 8 on `+` and to 6 on `-`, which is fine, but a flag value of 0.9 with a
    naive "first rung greater than this" rule would step DOWN to 0.5 on `-` and skip 0.8
    entirely.
    """
    rungs = LADDERS[name]
    low, high = LIVE_BOUNDS[name]
    eps = 1e-9
    # ⛔ No rung left in the pressed direction → the value stays EXACTLY where it is.
    # The old fallback (`rungs[-1]` / `rungs[0]`) moved a value the WRONG WAY when it sat
    # outside the ladder's ends, which flags and saved defaults can legitimately do:
    # `+` on floor 0.5 dropped it to 0.1, `-` on max_speed 0.1 raised it to 0.25. A press
    # up must never lower a limit and a press down must never raise one — on a rig with
    # no e-stop, "the key moved a safety limit the way the operator did not ask" is the
    # exact class of surprise this editor must not produce. `at_bound` already tells the
    # operator why the press did nothing.
    if up:
        nxt = next((r for r in rungs if r > value + eps), None)
    else:
        nxt = next((r for r in reversed(rungs) if r < value - eps), None)
    if nxt is None:
        return float(value)
    return float(min(high, max(low, nxt)))


def at_bound(name: str, value: float) -> str:
    """`"ceiling"`, `"floor"` or `""` — so the operator is told a press did nothing.

    ⚠️ Judged against the LADDER's ends as well as the numeric bound, because the top rung is
    what a press can actually reach. A value between the top rung and the hard bound would
    otherwise report nothing while `+` did nothing.
    """
    rungs = LADDERS[name]
    low, high = LIVE_BOUNDS[name]
    if value >= min(high, rungs[-1]) - 1e-9:
        return "ceiling"
    if value <= max(low, rungs[0]) + 1e-9:
        return "floor"
    return ""


def live_lines(values: dict[str, Any], selected: str | None,
               builtin: dict[str, Any] | None = None) -> list[str]:
    """The settings screen. `selected` is the one the `-`/`+` keys will move."""
    out = ["", "  ⭐ SETTINGS — the speed and safety limits, live.", ""]
    for i, name in enumerate(LIVE_ORDER, start=1):
        value = values.get(name)
        if value is None:
            continue
        mark = "▸" if name == selected else " "
        base = "" if builtin is None or name not in builtin else \
            ("" if value == builtin[name] else f"  (built-in {builtin[name]:g})")
        edge = at_bound(name, float(value))
        out.append(f"   {mark} {i}  {name:<14} {float(value):7.3f}"
                   f"{base}{'  ⚠️ ' + edge if edge else ''}")
    out += [
        "",
        "   1-9 pick a setting   - / +  change it   0  back to how this session started",
        "   s   SAVE these to config/session_defaults.json for every later session",
        "   t / g / h  leave        q  leave and quit the session   ?  this help",
        "",
        "  ⚠️ max_speed and max_lag take effect on the arms IMMEDIATELY. They are the two",
        "     limits that bound how fast 4.3 kg may move, so a change here is a real change.",
        "",
    ]
    return out


def one_line(name: str, value: float, before: float | None = None,
             builtin: dict[str, Any] | None = None) -> str:
    """One line for a single setting, for use after the screen has already been shown.

    ⛔⭐⭐ WHY THIS EXISTS. The first version reprinted the whole fifteen-line screen after
    **every** keypress. Julien's first use of it produced **thirteen copies** in one session,
    which buries everything else that happened and makes the scrollback useless for the very
    thing he uses it for: reading back what a session did.

    ⭐ The rest of the session already works this way — a transient one-liner for a change, the
    full block only when asked. This makes the settings screen match.
    """
    edge = at_bound(name, value)
    base = ""
    if builtin is not None and name in builtin and value != builtin[name]:
        base = f"  (built-in {builtin[name]:g})"
    if before is None or before == value:
        return f"   ▸ {name} {value:.3f}{base}{'  ⚠️ ' + edge if edge else ''}"
    return (f"   ▸ {name} {before:.3f} → {value:.3f}{base}"
            f"{'  ⚠️ at the ' + edge if edge else ''}")
