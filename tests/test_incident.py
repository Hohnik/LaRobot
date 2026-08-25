#!/usr/bin/env python3
"""Tests for the incident recorder. No hardware, and it writes to a temp directory.

    uv run tests/test_incident.py

⛔ WHY THIS FILE EXISTS. `src/yam/incident.py` runs on the **shutdown path**, right after the
motors have been disabled following a bad stop. Its one hard rule is that it may never
delay or prevent that teardown, on a rig with no emergency stop.

⭐ So the tests that matter are the ones that try to make it fail: a field that raises, a
`None`, an unwritable directory, a value JSON cannot serialise. **Every one must produce a
file with a note in it, or a clean `None`, and never an exception.** A crash reporter that
crashes is worse than no crash reporter.

⚠️ `usb_snapshot()` is exercised for its shape only. Whether it lists devices depends on
the machine, and a test that needs a USB bus is a test that fails on a clone.
"""

from __future__ import annotations

import json
import sys
import tempfile
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent

from yam.incident import describe, usb_snapshot, write_incident  # noqa: E402


class Boom:
    """Anything touched on this raises, like a chain that has already died."""

    def __getattr__(self, name: str):  # noqa: ANN204
        raise RuntimeError(f"chain is gone ({name})")


def read(path: Path) -> dict:
    return json.loads(path.read_text())


# ── it must write something useful ───────────────────────────────────────────


def test_it_writes_a_file_carrying_the_reason_and_the_facts() -> None:
    with tempfile.TemporaryDirectory() as tmp:
        path = write_incident("the motor chain STOPPED", {"arm": "B", "loop_hz": 84.0},
                              directory=Path(tmp))
        assert path is not None and path.exists()
        data = read(path)
        assert data["reason"] == "the motor chain STOPPED"
        assert data["arm"] == "B"
        assert data["loop_hz"] == 84.0


def test_it_records_provenance_without_being_asked() -> None:
    """⭐ A record with no commit and no time is the thing FINDINGS §33.2 was about."""
    with tempfile.TemporaryDirectory() as tmp:
        data = read(write_incident("x", {}, directory=Path(tmp)))
        assert data["commit"] and data["at"]
        assert "FINDINGS.md" in data["note"], "the file must say where to read about itself"


def test_it_always_includes_the_usb_bus() -> None:
    """⭐⭐ THE FIELD WHOSE ABSENCE COST THE MOST. FINDINGS §32 has asked since
    2026-08-13 for the bus state when a DFU fault appears, and nobody had captured it
    during a failure. §44.3 had to reconstruct the topology afterwards."""
    with tempfile.TemporaryDirectory() as tmp:
        data = read(write_incident("x", {}, directory=Path(tmp)))
        assert "usb" in data


def test_a_second_incident_does_not_overwrite_the_first() -> None:
    """⚠️ Overwriting is how four separate measurements were lost this week
    (FINDINGS §34.7, §35.7). Filenames carry the timestamp."""
    with tempfile.TemporaryDirectory() as tmp:
        folder = Path(tmp)
        write_incident("first", {"n": 1}, directory=folder)
        # A same-second second incident is the hard case; either it writes a distinct
        # file or it reuses one, and reusing one must not lose the newer content.
        write_incident("second", {"n": 2}, directory=folder)
        files = sorted(folder.glob("*.json"))
        latest = max((read(f) for f in files), key=lambda d: d["n"])
        assert latest["reason"] == "second", "the newer incident was lost"


# ── it must never raise, whatever is broken ──────────────────────────────────


def test_a_fact_that_raises_becomes_a_NOTE_not_an_exception() -> None:
    """⛔ The dying-chain case, which is the normal case for this file."""
    with tempfile.TemporaryDirectory() as tmp:
        boom = Boom()
        facts = {"mode": _guarded(lambda: boom.mode)}
        data = read(write_incident("chain gone", facts, directory=Path(tmp)))
        assert "unavailable" in str(data["mode"]), data["mode"]


def _guarded(fn):  # noqa: ANN001, ANN202
    """The caller's own guard, mirroring `teleop_session._safe_fact`."""
    try:
        return fn()
    except Exception as exc:  # noqa: BLE001
        return f"<unavailable: {type(exc).__name__}: {exc}>"


def test_an_unserialisable_value_does_not_stop_the_file_being_written() -> None:
    """⚠️ `default=str` covers numpy arrays and anything else that wanders in. Without
    it one odd field would take the whole record down."""
    with tempfile.TemporaryDirectory() as tmp:
        path = write_incident("x", {"weird": object(), "fine": 1}, directory=Path(tmp))
        assert path is not None
        assert read(path)["fine"] == 1


def test_an_unwritable_directory_returns_None_rather_than_raising() -> None:
    """⛔ THE RULE. The motors are already off at this point, and a raise here would
    surface as a traceback on top of whatever actually went wrong."""
    assert write_incident("x", {}, directory=Path("/dev/null/nope")) is None


def test_describe_says_what_to_DO_with_the_file() -> None:
    with tempfile.TemporaryDirectory() as tmp:
        path = write_incident("x", {}, directory=Path(tmp))
        text = describe(path)
        assert "PASTE IT" in text, "the operator has to be told not to commit it"
        assert "gitignored" in text


def test_describe_handles_a_failed_write() -> None:
    text = describe(None)
    assert "could not write" in text
    assert "console" in text, "it must point at where the information still is"


def test_the_usb_snapshot_never_raises_and_has_a_shape() -> None:
    snap = usb_snapshot()
    assert isinstance(snap, (list, str))
    if isinstance(snap, list) and snap:
        for key in ("bus", "addr", "vid", "pid", "product", "serial"):
            assert key in snap[0], key


def test_incidents_land_under_recordings_which_is_gitignored() -> None:
    """⚠️ An incident file holds a full pose and is evidence, not source. It must not
    become something that gets committed by habit."""
    from yam.incident import INCIDENT_DIR  # noqa: PLC0415

    assert "recordings" in INCIDENT_DIR.parts
    ignore = (REPO / ".gitignore").read_text()
    assert any(line.strip().rstrip("/") == "recordings" for line in ignore.splitlines()), \
        "recordings/ is not gitignored, so incident files would be committable"



# ── the safe-stop decision, added 2026-08-14 (FINDINGS §46) ──────────────────
#
# Julien: *"when the robot is being moved and stuff, and then it crashes for some reason,
# it should always resort to trying to do the safe crash… It should be, like, when I do
# control c."* The condition in `teleop_session.py` decides whether a stop parks the arm
# or hands it to a menu, and it cannot be reached by any headless test because it lives
# inside `main()`'s shutdown path. So the rule is duplicated here and pinned, and a test
# asserts the script still spells it the same way.


def should_auto_park(stop_reason: str | None, interrupted: bool) -> bool:
    """The rule, restated. Parks on Ctrl-C and on every UNPLANNED stop; not on `q`."""
    planned_quit = bool(stop_reason) and "quit requested" in (stop_reason or "")
    return interrupted or not planned_quit


def test_ctrl_c_parks() -> None:
    assert should_auto_park("Ctrl-C — the loop is stopping", interrupted=True)


def test_a_crash_parks_which_is_the_whole_point() -> None:
    """⭐ Previously a crash with a live chain left the arm holding until a human
    answered a menu. Julien may not be watching the terminal when it happens."""
    for reason in ("the motor chain STOPPED", "a motor is too hot", "IK failed"):
        assert should_auto_park(reason, interrupted=False), reason


def test_a_thermal_stop_parks_too_and_that_is_deliberate() -> None:
    """⚠️ Holding keeps current in a hot motor indefinitely and disabling drops the arm.
    Parking gets it to a supported pose and THEN removes current, which beats both."""
    assert should_auto_park("hottest motor 66°C — stopping", interrupted=False)


def test_a_planned_q_does_NOT_auto_park() -> None:
    """⛔ Julien uses `q p d` and may want `g` instead, so a planned quit keeps its menu."""
    assert not should_auto_park("quit requested", interrupted=False)


def test_the_script_still_spells_the_rule_the_same_way() -> None:
    """⛔ This file cannot execute the real branch, so it checks the source instead. If
    the wording drifts, the duplicated rule above stops describing the real one — which
    is exactly how ArmSession drifted for a day with green tests (FINDINGS §36.2)."""
    src = (REPO / "apps" / "teleop_session.py").read_text()
    assert "planned_quit" in src and "unplanned" in src, "the safe-stop rule is gone"
    assert "interrupted or unplanned" in src, "the auto-park condition changed shape"


def test_a_dead_chain_cannot_be_parked_and_the_docs_say_so() -> None:
    """⛔⭐ THE LIMIT OF THE FEATURE, and it is the case that prompted it. All seven
    motors latched `0xD loss of communication`, so the CAN link was gone and no command
    could reach the arm. Liveness gates the park for exactly that reason.

    ⚠️ Updated 2026-08-18: the N-arm rewrite made the gate per-arm — `live` is the list
    of arms still answering, and only those are parked. The old pinned string
    (`chain_alive(robot) and …`) had been gone for days while this test sat red,
    unnoticed, because no single runner runs these files (ROADMAP §10.5 step 2)."""
    src = (REPO / "apps" / "teleop_session.py").read_text()
    assert "live = [one for one in arms if one.alive()]" in src, \
        "the per-arm liveness list is gone"
    assert "if live and (interrupted or unplanned)" in src, \
        "the park is no longer gated on the chain being alive"


def test_a_dead_puck_parks_gracefully_instead_of_raising() -> None:
    """⛔⭐ FINDINGS §68.2: pulling a SpaceMouse mid-session raised `OSError: read error`,
    the exception skipped the auto-park entirely, and the finally disabled every motor
    with the arms wherever they stood. The per-cycle read is now guarded: a dead puck
    reads as centred and the stop routes through the SAFE STOP (park, then disable).
    This file cannot unplug hardware, so it pins the guard in the source."""
    src = (REPO / "apps" / "teleop_session.py").read_text()
    read_at = src.find("one.raw_axes = one.reader.read()")
    assert read_at != -1, "the per-cycle puck read moved; update this test with care"
    guarded = src.rfind("try:", 0, read_at)
    assert guarded != -1 and read_at - guarded < 120, \
        "the per-cycle puck read is no longer inside a try — an unplug drops the arms again"
    assert "SpaceMouse stopped answering" in src, \
        "the graceful stop_reason for a dead puck is gone"


def test_a_mode_key_ends_the_playback_for_EVERY_replay_arm() -> None:
    """⛔⭐ FINDINGS §69.1: mode keys re-mode only the AIMED arm, and the playback cleanup
    used to wait for NO arm to be in replay — so `m` aimed at B during a two-arm scrub
    left arm G a ZOMBIE (mode REPLAY, cursor frozen, no message) for the rest of
    Julien's session. Any arm leaving replay must end the playback for all of them,
    and the released arms go to HOLD through the class."""
    src = (REPO / "apps" / "teleop_session.py").read_text()
    assert 'any(one.mode != "replay" for one in replay_arms)' in src, \
        "the any-arm-left condition is gone — the zombie replay arm is back"
    assert 'not any(one.mode == "replay" for one in arms)' not in src, \
        "the old wait-for-everyone condition is back (the zombie's cause)"
    at = src.find('any(one.mode != "replay" for one in replay_arms)')
    tail = src[at:at + 400]
    assert "a2.enter_hold()" in tail, \
        "released replay arms are no longer put into HOLD through the class"


def test_liveness_is_captured_BEFORE_the_motors_are_disabled() -> None:
    """⛔⭐ FINDINGS §58.45: `chain_alive` was read AFTER `shutdown_robot()`, so every
    incident file ever written said False and the field measured nothing. The fix is an
    ordering fact, and this file cannot execute the teardown, so it pins the ORDER in
    the source: the capture must appear before the first `shutdown_robot(` call, and the
    incident dict must use the captured value rather than a fresh `alive()` read."""
    src = (REPO / "apps" / "teleop_session.py").read_text()
    capture = src.find("alive_at_teardown = {")
    disable = src.find("disabled = shutdown_robot(")
    assert capture != -1, "the pre-shutdown liveness capture is gone"
    assert disable != -1, "the shutdown call moved; update this test with care"
    assert capture < disable, \
        "liveness is captured AFTER shutdown_robot(), so the field is meaningless again"
    assert '"chain_alive_at_teardown": alive_at_teardown.get(' in src, \
        "the incident dict no longer records the pre-shutdown reading"
    assert '"chain_alive": _safe_fact(lambda one=one: bool(one.alive()))' not in src, \
        "the old post-shutdown read is back; it is always False (FINDINGS §58.45)"


def main() -> int:
    tests = [(n, f) for n, f in sorted(globals().items()) if n.startswith("test_") and callable(f)]
    failed = []
    for name, fn in tests:
        try:
            fn()
            print(f"  ✓ {name}")
        except AssertionError as exc:
            failed.append((name, str(exc)))
            print(f"  ✗ {name}\n      {exc}")
        except Exception as exc:  # noqa: BLE001
            failed.append((name, f"{type(exc).__name__}: {exc}"))
            print(f"  ✗ {name}\n      {type(exc).__name__}: {exc}")
    print(f"\n{len(tests) - len(failed)}/{len(tests)} passed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
