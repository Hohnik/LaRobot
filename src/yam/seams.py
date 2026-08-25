"""The three places something new can be plugged in, declared as interfaces.

⛔⭐⭐ WHY THIS FILE EXISTS, and it starts with a claim that was not true.

[ROADMAP §10.6](../../docs/ROADMAP.md) says, in the present tense, that *"everything that produces commands sits behind one interface"*. On 2026-08-20 `grep -rn 'Protocol|abstractmethod|ABC)' src/yam/` returned **one comment and no code**. There was not a single declared interface anywhere in this package. The sentence was an intention written as a description, which is the same defect [FINDINGS §76](../../docs/FINDINGS.md) kept finding in prose, found here in a design claim.

⭐ Julien asked for the fix on 2026-08-20: *"at best the code should intelligently be designed in a way that the missing piece already has the most basic version of a defined interface where we could just integrate it into the running system."*

## ⛔ THE RULE THIS FILE FOLLOWS, and it is narrow on purpose

**A Protocol goes in here only when a concrete class in this repo ALREADY satisfies it.** So every interface below describes code that runs, and `tests/test_seams.py` asserts that it still does. A Protocol nothing implements is the same present-tense fiction in a new place, and this file exists because of that fiction.

⚠️ **These are structural interfaces (`typing.Protocol`), not base classes.** Nothing has to inherit from anything. A class satisfies one by having the right methods, so the existing classes conform without being edited, and so does anything the team writes in their own repo without importing ours.

## ⭐ The team's repo declares the matching interfaces, and the names differ

`Hohnik/LaRobot` took the opposite approach: it declares abstract base classes and leaves the bodies empty. On `feature/camera-framework` its `Camera` ABC is `is_available()`, `connect()`, `read() -> Frame`, `close()`. **That is the same seam as `FrameSource` here, one level lower**: theirs is one device, ours is the set of them with a thread in front. [BRIDGE.md](../../docs/BRIDGE.md) maps the two, and it explains why a thread has to be in front of a blocking `read()`.

⚠️ **Do not rename these to match theirs, and do not rename theirs to match these.** The shapes are genuinely different and the map is the honest way to connect them.
"""

from __future__ import annotations

from typing import Any, Protocol, runtime_checkable

__all__ = ["CommandSource", "FrameSource", "FrameConsumer"]


@runtime_checkable
class CommandSource(Protocol):
    """Anything that produces a movement request: the puck, a keyboard, a trained policy.

    ⭐ **This is the Phase E seam**, and it is one method. A policy that emits six numbers per call can drive the arms through the existing loop with no other change, and it inherits every limit in `SafeRobot` and every guard in `ArmSession` for free, because those are below this.

    ⭐ **`src/yam/inputs/spacemouse.py::TwistReader` already satisfies this**, which is what makes it a description rather than a wish.

    ⚠️ **What is NOT decided here, and it is a real open question for the team:** their `Input` ABC declares `is_available()` and says nothing about how a command is read. So the discovery half of the interface is theirs and the read half is ours, and neither side has both. [BRIDGE.md](../../docs/BRIDGE.md) lists it as a decision.
    """

    def read(self) -> list[float]:
        """Six numbers, each between -1 and 1.

        The first three are movement along the three axes. The last three are rotation about
        them. All six are fractions of the configured maximum speed, never absolute speeds,
        so the speed limits stay in one place.

        ⛔ **Must not block.** It is called once per pass of a loop that runs about 85 times a
        second. A source with nothing new to say returns zeros.

        ⛔ **A dead source returns zeros, never raises and never returns stale values.** A
        SpaceMouse that is unplugged mid-session reads as centred, so the arm holds position
        ([FINDINGS §68.2](../../docs/FINDINGS.md)). A policy that has not finished thinking
        does the same. Anything else turns a failed input into a movement.
        """
        ...


@runtime_checkable
class FrameSource(Protocol):
    """Everything that supplies camera pictures to the loop, as a set rather than one camera.

    ⭐ **`src/yam/cameras/capture.py::CaptureSet` already satisfies this.**

    ⭐ **This is the seam for two different missing pieces**, which is why it is worth declaring:

    - **Depth.** `Frame` already has a `depth` field and nothing ever fills it. A source backed by Intel's librealsense would fill it. That library needs no administrator rights and it streams depth at up to 1280x720 on the station ([FINDINGS §76.10](../../docs/FINDINGS.md)).
    - **A separate camera process**, if the loop ever needs isolating from camera work. [PERFORMANCE.md](../../docs/PERFORMANCE.md) section 5 explains why nothing needs that today.

    ⛔ **`sample()` must never block**, and that is the whole reason this interface is a set with a thread behind it instead of one camera you pull from. A blocking read called once per loop pass makes the loop run at the camera's frame rate. [FINDINGS §21](../../docs/FINDINGS.md) is the measurement from getting this wrong: **6 pictures a second**.
    """

    def names(self) -> list[str]:
        """Every camera's name, in a stable order. The name is what a recording stores."""
        ...

    def sample(self) -> dict[str, Any]:
        """Each camera's newest picture, right now. Never blocks.

        `None` for a camera whose first picture has not arrived. The returned `Frame` carries
        a `sequence` number from the reader, so a consumer can tell a repeat from a new
        picture, and a `host_timestamp_ns` taken when the picture was stored.

        ⛔ **The timestamp is not optional.** The episode export joins pictures to control
        ticks by nearest timestamp, so a picture without one cannot be joined to anything.
        """
        ...

    def stop(self) -> None:
        """Release every device. Safe to call twice."""
        ...


@runtime_checkable
class FrameConsumer(Protocol):
    """Whatever a recording's pictures are handed to. Today that is disk, one file each.

    ⭐ **`src/yam/cameras/writer.py::FrameSink` already satisfies this.**

    ⭐ Declared because it is the other half of the separate-process option, and because a
    consumer that wrote a video stream instead of one file per picture would slot in here
    unchanged. ⚠️ It would also break the export, which reads one file per picture, so that
    is a bigger change than the interface makes it look.
    """

    def offer(self, samples: dict[str, Any]) -> None:
        """Take one `FrameSource.sample()` result. Must not block the caller.

        ⛔ Forward only pictures whose sequence advanced. A slow camera's last picture is
        repeated by the source on purpose, and storing the repeats would write every picture
        three times at a 90 Hz loop.

        ⛔ **Never block, and never raise.** This is called from the control loop. A full
        queue drops the oldest picture and counts the drop, because a dropped picture is a
        number in a report and a stalled loop is an arm that stops responding.
        """
        ...

    def stop(self) -> dict[str, dict[str, Any]]:
        """Finish writing and return, per camera, what actually happened.

        ⛔ The counts are the point: how many were written, how many dropped, how many failed
        to write. `checks/check_recordings.py` compares them against the files on disk, and
        [FINDINGS §76.15](../../docs/FINDINGS.md) is what it missed while it only compared
        those two numbers to each other and never to the recording's length.
        """
        ...
