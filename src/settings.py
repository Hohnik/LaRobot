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
