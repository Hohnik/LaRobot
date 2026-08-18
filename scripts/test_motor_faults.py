#!/usr/bin/env python3
"""Tests for reading a motor fault without erasing it. No hardware, no CAN bus.

    uv run scripts/test_motor_faults.py

⛔ WHY THIS FILE EXISTS. Until 2026-08-14, `ping_motors.py` documented a flag
`--attempt-error-clear` as "default off", advertised it in `--help`, and **read it
nowhere**. The vendor's `motor_on()` therefore cleared every latched motor fault on
every run, in a loop, while the root log level was forced to ERROR so both messages
naming the fault were suppressed. A faulted motor was silently repaired and reported
healthy. That is how the evidence for arm G's red flashing lights was destroyed
(FINDINGS §39).

⭐ These tests exercise the wrapper factories directly against a fake interface, so
none of the vendored driver and no bus is needed. The factories are module-level in
`src/yam/can.py` for exactly this reason.

⚠️ The one behaviour that must NEVER regress: the default policy is to clear. The
chain interface's own motor-recovery routine (`dm_driver.py:639`) calls `clean_error`
during a real session, and raising there would turn a recoverable motor into a dead
arm. `test_the_default_policy_still_clears` is the guard on that.
"""

from __future__ import annotations

import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent

from yam.can import (  # noqa: E402
    MOTOR_LED_FOR_ERROR,
    MotorFaultNotCleared,
    _wrap_clean_error,
    _wrap_parse_recv_message,
    describe_motor_error,
    do_not_clear_motor_faults,
)


class FakeMessage:
    """A CAN reply. The error lives in the high nibble of byte 0, per the vendor."""

    def __init__(self, error_code: int) -> None:
        self.data = bytes([(error_code << 4) | 0x01, 0, 0, 0, 0, 0, 0, 0])


class FakeInterface:
    """Stands in for `DMSingleMotorCanInterface`, with the two methods we wrap."""

    def __init__(self) -> None:
        self.cleared: list[int] = []
        self.parsed: list[int] = []

    def clean_error(self, motor_id: int) -> str:
        self.cleared.append(motor_id)
        return "cleared"

    def parse_recv_message(self, message: FakeMessage, motor_type: str, ignore_error: bool = False) -> str:
        self.parsed.append(message.data[0])
        if not ignore_error and (message.data[0] & 0xF0) >> 4 != 0x1:
            raise RuntimeError("Motor error detected")
        return "parsed"


def wired() -> FakeInterface:
    """A fake with both wrappers installed, exactly as the real patch installs them."""
    FakeInterface.clean_error = _wrap_clean_error(FakeInterface.__dict__["clean_error"])
    FakeInterface.parse_recv_message = _wrap_parse_recv_message(
        FakeInterface.__dict__["parse_recv_message"]
    )
    return FakeInterface()


# ── the default must not change ──────────────────────────────────────────────


def test_the_default_policy_still_clears() -> None:
    """⛔ THE GUARD THAT MATTERS. `DMChainCanInterface`'s recovery routine calls
    clean_error mid-session. If the default ever became 'refuse', a motor that is
    currently recoverable would become a dead arm instead."""
    iface = wired()
    assert iface.clean_error(3) == "cleared"
    assert iface.cleared == [3]


def test_a_healthy_reply_takes_the_unchanged_path() -> None:
    """Error 0x1 is every run anyone has ever made. It must be byte-identical."""
    iface = wired()
    assert iface.parse_recv_message(FakeMessage(0x1), "DM4310") == "parsed"
    assert not hasattr(iface, "_last_motor_fault")


# ── refusing to clear ────────────────────────────────────────────────────────


def test_inside_the_context_a_clear_is_refused() -> None:
    iface = wired()
    with do_not_clear_motor_faults():
        try:
            iface.clean_error(5)
        except MotorFaultNotCleared as exc:
            assert exc.motor_id == 5
        else:
            raise AssertionError("clean_error did not refuse")
    assert iface.cleared == [], "the fault must be left in place, not cleared"


def test_the_refusal_carries_the_fault_code_the_parser_saw() -> None:
    """⭐ The point of the parser wrapper: `clean_error` is handed only a motor id,
    so without this the refusal could say a fault existed but never which one."""
    iface = wired()
    try:
        iface.parse_recv_message(FakeMessage(0xD), "DM4310", ignore_error=True)
    except RuntimeError:
        raise AssertionError("ignore_error=True must not raise") from None
    with do_not_clear_motor_faults():
        try:
            iface.clean_error(2)
        except MotorFaultNotCleared as exc:
            assert exc.code == 0xD, f"expected 0xD, got {exc.code!r}"
            assert "loss of communication" in str(exc)
            return
    raise AssertionError("clean_error did not refuse")


def test_the_code_is_recorded_even_when_the_vendor_parser_RAISES() -> None:
    """⛔ The reason the nibble is decoded before delegating. With ignore_error
    False the vendor's parser raises, so there is no return value to inspect —
    and `motor_on` has forced the log level to ERROR, so the message that names
    the fault never reaches a handler either."""
    iface = wired()
    try:
        iface.parse_recv_message(FakeMessage(0xA), "DM4310", ignore_error=False)
    except RuntimeError:
        pass
    else:
        raise AssertionError("the fake parser should have raised")
    assert iface._last_motor_fault == 0xA


def test_the_policy_is_restored_after_an_exception_inside_the_block() -> None:
    iface = wired()
    try:
        with do_not_clear_motor_faults():
            raise ValueError("something else went wrong")
    except ValueError:
        pass
    assert iface.clean_error(1) == "cleared", "the policy leaked out of the block"


def test_nesting_restores_the_outer_policy_not_the_global_default() -> None:
    iface = wired()
    with do_not_clear_motor_faults():
        with do_not_clear_motor_faults():
            pass
        try:
            iface.clean_error(4)
        except MotorFaultNotCleared:
            return
    raise AssertionError("the inner block restored clearing while still inside the outer one")


# ── the LED table ────────────────────────────────────────────────────────────


def test_every_error_code_in_the_sdk_has_an_LED_meaning() -> None:
    """The table exists so a glance at the arm can be read. A missing code means
    a light nobody can interpret, which is the gap FINDINGS §36.0 recorded."""
    sys.path.insert(0, str(REPO / "third_party" / "i2rt"))
    from i2rt.motor_drivers.utils import MotorErrorCode  # noqa: PLC0415

    for code in MotorErrorCode.motor_error_code_dict:
        assert code in MOTOR_LED_FOR_ERROR, f"code 0x{code:X} has no LED meaning"


def test_the_two_states_that_look_alike_are_distinguished() -> None:
    """⭐⭐ THE WHOLE POINT. Red STEADY is a disabled motor and is normal for a
    powered arm nobody is commanding. Red FLASHING is a latched fault. FINDINGS
    §36.0 had guessed that blinking was the normal idle indication; the vendor
    manual says the opposite, and confusing the two is how a real fault gets
    dismissed."""
    idle = describe_motor_error(0x0)
    fault = describe_motor_error(0xD)
    assert "steady" in idle and "FLASHING" not in idle, idle
    assert "FLASHING" in fault, fault


def test_code_1_is_described_as_enabled_and_not_merely_normal() -> None:
    """⛔ The SDK calls 0x1 "normal", the manual calls it "enable mode". Reading
    0x1 after an enable frame proves the motor is on RIGHT NOW, not that it was
    healthy before something enabled it — which is exactly the inference that
    made §36.0 conclude the lights were fine."""
    assert "enabled" in describe_motor_error(0x1)


def test_an_unknown_code_says_so_rather_than_inventing_a_meaning() -> None:
    assert "unrecognised" in describe_motor_error(0x7)
    assert "unknown" in describe_motor_error(None)


# ── the flag is really wired now ─────────────────────────────────────────────


def test_ping_motors_actually_READS_its_own_flag() -> None:
    """⛔ THE DEFECT ITSELF. The flag was parsed and documented and never read, so
    this asserts on the source: the name must appear somewhere other than the
    docstring and the add_argument call."""
    source = (REPO / "scripts" / "ping_motors.py").read_text()
    uses = source.count("attempt_error_clear")
    assert uses >= 2, f"'attempt_error_clear' is referenced {uses} time(s) — it is dead again"
    assert "do_not_clear_motor_faults" in source, "the flag has nothing to switch"


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
