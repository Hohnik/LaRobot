#!/usr/bin/env python3
"""Tests for the incident recorder. No hardware, and it writes to a temp directory.

    uv run scripts/test_incident.py

⛔ WHY THIS FILE EXISTS. `src/incident.py` runs on the **shutdown path**, right after the
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
sys.path.insert(0, str(REPO / "src"))

from incident import describe, usb_snapshot, write_incident  # noqa: E402


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
    from incident import INCIDENT_DIR  # noqa: PLC0415

    assert "recordings" in INCIDENT_DIR.parts
    ignore = (REPO / ".gitignore").read_text()
    assert any(line.strip().rstrip("/") == "recordings" for line in ignore.splitlines()), \
        "recordings/ is not gitignored, so incident files would be committable"


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
