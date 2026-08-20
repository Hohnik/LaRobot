#!/usr/bin/env python3
"""Prove the declared interfaces in `yam/seams.py` describe code that exists.

    uv run tests/test_seams.py

⛔ WHY THIS IS THE POINT OF THE FILE IT TESTS. `src/yam/seams.py` exists because
[ROADMAP §10.6](../docs/ROADMAP.md) claimed, in the present tense, that a command-source
interface existed when none did. **An interface nobody implements is the same claim in a new
place.** So each Protocol is checked against the concrete class that is supposed to satisfy
it, and if somebody changes that class's method names the check fails here rather than being
discovered by whoever tries to plug a policy in.

⚠️ These are structural checks, so nothing inherits from anything. A class satisfies a
Protocol by having the right methods with the right names.
"""

from __future__ import annotations

import inspect
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent

from yam.cameras.capture import CaptureSet  # noqa: E402
from yam.cameras.writer import FrameSink  # noqa: E402
from yam.inputs.spacemouse import TwistReader  # noqa: E402
from yam.seams import CommandSource, FrameConsumer, FrameSource  # noqa: E402


def methods_of(protocol: type) -> list[str]:
    """The method names a Protocol requires."""
    return sorted(n for n, v in vars(protocol).items()
                  if callable(v) and not n.startswith("_"))


def test_the_spacemouse_reader_satisfies_CommandSource() -> None:
    """⭐ This is the Phase E seam. A trained policy needs this one method and nothing else."""
    for name in methods_of(CommandSource):
        assert hasattr(TwistReader, name), (
            f"TwistReader has no {name}(), so CommandSource no longer describes it")


def test_CommandSource_is_exactly_one_method_because_that_is_the_whole_point() -> None:
    """⚠️ If this grows, a policy has more to implement, and the seam is worth less. Growing it
    is a real decision, not a tidying step."""
    assert methods_of(CommandSource) == ["read"], methods_of(CommandSource)


def test_read_returns_six_numbers_and_says_so() -> None:
    """⭐ The shape is the contract: three for movement, three for rotation, each -1 to 1."""
    sig = inspect.signature(TwistReader.read)
    assert list(sig.parameters) == ["self"], "read() takes no arguments"
    doc = (CommandSource.read.__doc__ or "")
    assert "Six numbers" in doc, "the count has to be stated where somebody implementing it looks"


def test_the_capture_set_satisfies_FrameSource() -> None:
    for name in methods_of(FrameSource):
        assert hasattr(CaptureSet, name), (
            f"CaptureSet has no {name}(), so FrameSource no longer describes it")


def test_the_frame_sink_satisfies_FrameConsumer() -> None:
    for name in methods_of(FrameConsumer):
        assert hasattr(FrameSink, name), (
            f"FrameSink has no {name}(), so FrameConsumer no longer describes it")


def test_the_frame_a_source_hands_over_still_has_a_timestamp() -> None:
    """⛔ The export joins pictures to control ticks by nearest timestamp, so a picture
    without one cannot be joined to anything. The team's own `Frame` has no timestamp field
    yet, which is why this is asserted rather than assumed ([BRIDGE.md](../docs/BRIDGE.md))."""
    from yam.cameras.frame import Frame  # noqa: PLC0415

    fields = set(getattr(Frame, "__dataclass_fields__", {}))
    assert "host_timestamp_ns" in fields, fields
    assert "sequence" in fields, "without a sequence a consumer cannot tell a repeat from new"
    assert "depth" in fields, "the depth slot is the declared hole for a librealsense source"


def test_every_protocol_names_the_class_that_satisfies_it() -> None:
    """⚠️ A Protocol whose docstring does not name a real implementation is the fiction this
    file exists to prevent. The name in the docstring is what a reader checks against."""
    for proto, expect in ((CommandSource, "TwistReader"),
                          (FrameSource, "CaptureSet"),
                          (FrameConsumer, "FrameSink")):
        doc = proto.__doc__ or ""
        assert expect in doc, f"{proto.__name__} does not name {expect} in its docstring"


def main() -> int:
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failed = 0
    for fn in tests:
        try:
            fn()
        except AssertionError as e:  # noqa: PERF203
            failed += 1
            print(f"✗ {fn.__name__}: {e}")
        else:
            print(f"✓ {fn.__name__}")
    print(f"\n{len(tests) - failed}/{len(tests)} passed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
