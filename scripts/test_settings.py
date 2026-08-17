#!/usr/bin/env python3
"""Tests for saved session defaults, `src/settings.py`. No hardware.

    uv run scripts/test_settings.py

⭐⭐ WHY THIS MATTERS MORE THAN A PREFERENCES FILE USUALLY WOULD. Four of the settings it
saves are **safety limits** that bound how fast and how far 4.3 kg may move: `max_speed`,
`max_lag`, `reach`, `floor`. A preferences file that quietly loosens one of those is a worse
outcome than no preferences file at all, because a flag typed on the command line is visible
in the shell history and on screen while a saved default is not.

⛔ So the tests here are mostly about the ways this could go WRONG rather than the happy
path: a wrong type in the file, an unreadable file, a key nobody reads, a loosening that
does not announce itself, and `--yes` ever being saveable.
"""

from __future__ import annotations

import json
import sys
import tempfile
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "src"))

from settings import (  # noqa: E402
    LIVE_BOUNDS,
    LIVE_ORDER,
    TUNABLE,
    adjust,
    at_bound,
    live_lines,
    describe,
    load_defaults,
    looser_than_builtin,
    rejected_keys,
    save_defaults,
)

BUILTIN = {"max_speed": 1.0, "teleop_speed": 1.5, "max_lag": 0.25, "mirror_gap": 0.35,
           "reach": 0.6, "floor": 0.0, "start_mode": "guide", "frame": "world",
           "linear_scale": 0.12, "gripper_step": 0.02,
           "no_rotation": False, "no_smooth": False, "no_gripper": False}


def wrote(data) -> Path:  # noqa: ANN001
    """A settings file holding `data`, in a temp dir that outlives the call."""
    tmp = Path(tempfile.mkdtemp())
    path = tmp / "session_defaults.json"
    path.write_text(json.dumps(data) if not isinstance(data, str) else data)
    return path


# ------------------------------------------------------------------- the round trip


def test_saving_then_loading_gives_the_same_values_back() -> None:
    path = Path(tempfile.mkdtemp()) / "nested" / "session_defaults.json"
    values = {"max_speed": 4.0, "mirror_gap": 0.6, "start_mode": "hold",
              "no_rotation": True}
    save_defaults(path, values)
    assert load_defaults(path) == values


def test_saving_creates_the_directory() -> None:
    """⚠️ `config/` exists in the repo, but a fresh clone or a different root must not make
    saving fail with a traceback."""
    path = Path(tempfile.mkdtemp()) / "a" / "b" / "session_defaults.json"
    save_defaults(path, {"max_speed": 2.0})
    assert path.is_file()


def test_the_saved_file_explains_itself_to_a_human() -> None:
    """⭐ He will open this by hand. It has to say what overrides what and how to undo it."""
    path = Path(tempfile.mkdtemp()) / "session_defaults.json"
    save_defaults(path, {"max_speed": 2.0})
    note = json.loads(path.read_text())["_comment"]
    assert "flag" in note and "override" in note.lower()
    assert "delete" in note.lower(), "it must say how to get back to the built-in values"


def test_the_comment_key_is_not_read_back_as_a_setting() -> None:
    path = Path(tempfile.mkdtemp()) / "session_defaults.json"
    save_defaults(path, {"max_speed": 2.0})
    assert load_defaults(path) == {"max_speed": 2.0}


# ------------------------------------------------------------------- the refusals


def test_a_MISSING_file_is_no_settings_rather_than_a_crash() -> None:
    assert load_defaults(Path("/nonexistent/session_defaults.json")) == {}
    assert rejected_keys(Path("/nonexistent/session_defaults.json")) == []


def test_UNREADABLE_JSON_is_no_settings_rather_than_a_crash() -> None:
    """⛔ The arm must still run on its built-in constants. A diagnostic file breaking a
    session is the wrong trade."""
    assert load_defaults(wrote("{ this is not json")) == {}


def test_a_JSON_list_instead_of_an_object_is_refused() -> None:
    assert load_defaults(wrote([1, 2, 3])) == {}


def test_a_WRONG_TYPE_is_refused_rather_than_coerced() -> None:
    """⛔⭐⭐ THE MOST DANGEROUS ENTRY IN THIS FILE WOULD BE A SILENT COERCION. `float("fast")`
    raises, and `bool("false")` is **True** — so a string "false" coerced to a bool would
    switch a setting ON while reading as off. Nothing here coerces."""
    assert load_defaults(wrote({"max_speed": "fast"})) == {}
    assert load_defaults(wrote({"max_speed": None})) == {}
    assert load_defaults(wrote({"no_rotation": "false"})) == {}
    assert load_defaults(wrote({"start_mode": 3})) == {}


def test_a_bool_is_NOT_accepted_where_a_float_belongs() -> None:
    """⚠️ In Python `True` is an instance of `int`, so a naive numeric check would accept
    `max_speed: true` and run the arm at 1.0 rad/s for a reason nobody could see."""
    assert load_defaults(wrote({"max_speed": True})) == {}


def test_an_int_IS_accepted_where_a_float_belongs() -> None:
    """⭐ He will hand-edit this file and write `4`, not `4.0`."""
    assert load_defaults(wrote({"max_speed": 4})) == {"max_speed": 4.0}


def test_an_UNKNOWN_key_is_dropped_AND_reported() -> None:
    """⛔ Silence is the failure this prevents: a misspelled setting looks saved and does
    nothing. Dropping it is right; dropping it quietly is not."""
    path = wrote({"max_speeed": 4.0, "max_speed": 2.0})
    assert load_defaults(path) == {"max_speed": 2.0}
    assert rejected_keys(path) == ["max_speeed"]


def test_a_wrongly_typed_key_is_also_REPORTED_not_just_dropped() -> None:
    path = wrote({"max_speed": "fast"})
    assert load_defaults(path) == {}
    assert rejected_keys(path) == ["max_speed"]


# -------------------------------------------------------------- what may be saved


def test_YES_can_NEVER_be_saved() -> None:
    """⛔⭐⭐ ENERGISING THE MOTORS MUST BE A CONSCIOUS ACT ON EVERY RUN. A saved `yes: true`
    would turn a dry run into a live one for anyone who had forgotten the file existed, and
    the dry run is the main safety habit this project has."""
    assert "yes" not in TUNABLE
    assert load_defaults(wrote({"yes": True})) == {}
    assert rejected_keys(wrote({"yes": True})) == ["yes"]


def test_ARMS_can_never_be_saved() -> None:
    """⚠️ Which arms are plugged in changes between sessions, and arm G is shared with a
    colleague. A stale saved value would try to build an arm that is not there."""
    assert "arms" not in TUNABLE and "arm" not in TUNABLE


def test_SIM_can_never_be_saved() -> None:
    """⛔ A saved `sim: true` would mean a session that looks normal and drives nothing, or
    worse, someone believing a simulated run was real."""
    assert "sim" not in TUNABLE
    assert "save_defaults" not in TUNABLE


def test_every_safety_limit_declares_which_direction_is_LOOSER() -> None:
    """⚠️ Without that field the plan cannot tell a tightening from a loosening, and would
    either warn about both or neither."""
    for key, (_, is_limit, looser) in TUNABLE.items():
        if is_limit:
            assert looser in ("up", "down"), f"{key} is a limit with no direction"
        else:
            assert looser is None, f"{key} is not a limit but declares a direction"


# ------------------------------------------------------ announcing a loosening


def test_a_saved_HIGHER_speed_limit_is_reported_as_looser() -> None:
    assert looser_than_builtin({"max_speed": 4.0}, BUILTIN) == ["max_speed"]


def test_a_saved_LOWER_speed_limit_is_NOT_reported() -> None:
    """⭐ Tightening a limit needs no warning. Warning about both would make the line noise,
    and then it gets skipped on the run where it matters."""
    assert looser_than_builtin({"max_speed": 0.5}, BUILTIN) == []


def test_the_FLOOR_is_looser_DOWNWARDS() -> None:
    """⛔ The floor is the one limit where a SMALLER number is more permissive: it is how far
    down the tip may go. A rule that only understood "bigger is looser" would miss the one
    setting that lets the arm drive into the desk."""
    assert looser_than_builtin({"floor": -0.05}, BUILTIN) == ["floor"]
    assert looser_than_builtin({"floor": 0.05}, BUILTIN) == []


def test_a_preference_is_never_reported_as_a_loosening() -> None:
    assert looser_than_builtin({"linear_scale": 0.9, "frame": "tool"}, BUILTIN) == []


def test_several_loosenings_are_all_named() -> None:
    loose = looser_than_builtin(
        {"max_speed": 4.0, "max_lag": 0.4, "mirror_gap": 0.6, "reach": 0.9}, BUILTIN)
    assert loose == ["max_lag", "max_speed", "mirror_gap", "reach"]


# ------------------------------------------------------------------ the plan lines


def test_no_file_means_no_plan_lines_at_all() -> None:
    assert describe({}, [], [], Path("config/session_defaults.json")) == []


def test_only_the_settings_that_DIFFER_are_listed() -> None:
    """⭐ The first version listed all thirteen, a 280-character line reporting `frame=world`
    when `world` is the built-in value. A banner that lists everything is a banner nobody
    reads, and the entire purpose of this line is that a saved change cannot hide."""
    lines = describe({"max_speed": 4.0, "frame": "world"}, [], ["max_speed"],
                     Path("config/session_defaults.json"), BUILTIN)
    assert "max_speed=4.0" in lines[0]
    assert "frame" not in lines[0], f"an unchanged setting is being listed: {lines[0]!r}"


def test_a_loosening_gets_its_OWN_line_and_says_it_is_permanent() -> None:
    lines = describe({"max_speed": 4.0}, [], ["max_speed"],
                     Path("config/session_defaults.json"), BUILTIN)
    loud = [ln for ln in lines if "LOOSER" in ln]
    assert len(loud) == 1, f"the loosening is not announced: {lines}"
    assert "max_speed" in loud[0]


def test_ignored_keys_get_their_own_line() -> None:
    lines = describe({}, ["max_speeed"], [], Path("config/session_defaults.json"), BUILTIN)
    assert any("IGNORED" in ln and "max_speeed" in ln for ln in lines)


def test_a_file_whose_values_all_match_the_builtins_still_says_it_was_read() -> None:
    """⚠️ Otherwise the file looks like it was ignored, and he goes looking for why."""
    lines = describe({"frame": "world"}, [], [], Path("config/session_defaults.json"),
                     BUILTIN)
    assert lines and "session_defaults.json" in lines[0]


# ------------------------------------------------------------- the live editor


def test_a_press_moves_a_speed_by_a_RATIO_not_a_fixed_amount() -> None:
    """⭐ 0.1 → 0.125 and 8 → 10 are the same FELT step. A fixed increment cannot be both,
    and a setting that spans two orders of magnitude needs the ratio."""
    assert adjust("max_speed", 4.0, True) == 5.0
    assert adjust("max_speed", 4.0, False) == 3.2
    assert adjust("max_speed", 0.4, True) == 0.5


def test_the_FLOOR_steps_additively_because_it_crosses_zero() -> None:
    """⛔ A ratio cannot move 0.0 at all, and the floor's default IS 0.0. It would have been
    the one setting the editor silently could not change."""
    assert adjust("floor", 0.0, False) == -0.005
    assert adjust("floor", 0.0, True) == 0.005


def test_the_floor_may_go_BELOW_zero() -> None:
    """⚠️ Julien uses `--floor -0.005` to pick a flat object off the desk, so a lower bound of
    zero would have removed a case he actually uses."""
    assert adjust("floor", -0.01, False) < 0.0
    assert LIVE_BOUNDS["floor"][0] < 0.0


def test_every_setting_STOPS_at_its_ceiling() -> None:
    """⛔⭐ THE REASON BOUNDS EXIST AT ALL. The linear-speed key had none and a held press
    reached 19.852 m/s, 165x its default. A repeating key must not be able to run away."""
    for name, (low, high) in LIVE_BOUNDS.items():
        value = high
        for _ in range(50):
            value = adjust(name, value, True)
        assert value == high, f"{name} climbed past its ceiling to {value}"


def test_every_setting_STOPS_at_its_floor() -> None:
    for name, (low, high) in LIVE_BOUNDS.items():
        value = low
        for _ in range(50):
            value = adjust(name, value, False)
        assert value == low, f"{name} fell below its floor to {value}"


def test_being_AT_a_bound_is_reported_so_a_dead_press_is_visible() -> None:
    """⚠️ Otherwise a key that does nothing reads as a key that is broken."""
    assert at_bound("max_speed", LIVE_BOUNDS["max_speed"][1]) == "ceiling"
    assert at_bound("max_speed", LIVE_BOUNDS["max_speed"][0]) == "floor"
    assert at_bound("max_speed", 4.0) == ""


def test_the_bounds_are_GENEROUS_enough_not_to_get_in_his_way() -> None:
    """⭐ These are backstops, never policy. Every value he has actually run must sit inside
    them: max_speed 10, max_lag 1.0, mirror_gap 2.0 are all from his 2026-08-17 sessions."""
    for name, used in (("max_speed", 10.0), ("teleop_speed", 10.0),
                       ("max_lag", 1.0), ("mirror_gap", 2.0), ("reach", 0.6)):
        low, high = LIVE_BOUNDS[name]
        assert low <= used <= high, f"{name}={used} is outside the editor's bounds"


def test_every_live_setting_is_also_SAVEABLE() -> None:
    """⛔ A setting he can change live but not save would be the exact complaint he raised,
    reintroduced one level down."""
    for name in LIVE_ORDER:
        assert name in TUNABLE, f"{name} is editable live but cannot be saved"


def test_every_live_setting_has_a_bound() -> None:
    for name in LIVE_ORDER:
        assert name in LIVE_BOUNDS, f"{name} is editable with no bound"


def test_the_screen_marks_which_setting_the_keys_will_MOVE() -> None:
    """⚠️ Six numbers and two keys is ambiguous without it."""
    lines = live_lines({k: 1.0 for k in LIVE_ORDER}, "max_lag")
    picked = [ln for ln in lines if "▸" in ln]
    assert len(picked) == 1 and "max_lag" in picked[0]


def test_the_screen_shows_the_BUILT_IN_value_when_it_differs() -> None:
    """⭐ So he can see how far from the shipped default he has pushed a safety limit."""
    lines = live_lines({"max_speed": 4.0}, "max_speed", {"max_speed": 1.0})
    assert any("built-in 1" in ln for ln in lines)
    same = live_lines({"max_speed": 1.0}, "max_speed", {"max_speed": 1.0})
    assert not any("built-in" in ln for ln in same), "it should stay quiet when unchanged"


def test_the_screen_WARNS_that_two_of_them_are_live_limits() -> None:
    """⛔ max_speed and max_lag take effect on the next cycle. A screen that let him change
    those without saying so would be the worst kind of convenience."""
    text = "\n".join(live_lines({k: 1.0 for k in LIVE_ORDER}, None))
    assert "IMMEDIATELY" in text and "4.3 kg" in text


def main() -> int:
    tests = [(n, f) for n, f in sorted(globals().items())
             if n.startswith("test_") and callable(f)]
    failed = []
    for name, fn in tests:
        try:
            fn()
            print(f"  ✓ {name}")
        except Exception as exc:  # noqa: BLE001
            failed.append(name)
            print(f"  ✗ {name}\n      {type(exc).__name__}: {exc}")
    print(f"\n{len(tests) - len(failed)}/{len(tests)} passed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
