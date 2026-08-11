"""Talk to the YAM's CAN bus from macOS.

I2RT's own stack assumes Linux + SocketCAN throughout. This module is the thin
compatibility layer that makes the same driver code work here, over the CANable
in candleLight (gs_usb) mode via libusb. Import it before opening any bus.

What lives here, and nothing else:

1. ``patch_gs_usb_for_macos()``   — suppress a spurious kernel-driver detach.
2. ``patch_gs_usb_echo_filter()`` — ⛔ drop our own transmit echoes. Load-bearing.
3. ``add_i2rt_to_path()``         — put the vendored SDK on ``sys.path``.
4. ``open_raw_can_interface()``   — register reads; never enables a motor.
5. ``open_motor_interface()``     — the DM motor interface (can enable motors).

⛔ Nothing in this module transmits. Opening a bus in normal mode does make the
adapter an active node (it will acknowledge frames it hears), which is why
``listen_only`` is offered and why callers that poll motors say so explicitly.
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parent.parent
I2RT_PATH = REPO_ROOT / "third_party" / "i2rt"

YAM_BITRATE = 1_000_000  # I2RT documents 1 Mbit/s; see third_party/i2rt README

# ⛔ NEVER SELECT THE ADAPTER BY INDEX. Each arm has its own CANable, so as soon
# as the second one is plugged in, "adapter 0" silently becomes a different arm.
#
# That is not hypothetical: on 2026-08-10 every measurement was taken against
# serial 2081337C… as the only adapter present. Julien then plugged in the second
# arm, and its adapter enumerated *first*. Selecting by index at that moment would
# have commanded the wrong arm — with a motion script already written.
#
# Serial numbers are stable across replug; bus/address are not.
# ⭐ RENAMED 2026-08-11, at Julien's request: arm1 -> B, arm2 -> G. The arms are
# physically labelled B and G, so the names now match what is written on the
# hardware. "arm1"/"arm2" were an arbitrary software ordering that told you nothing
# while standing at the bench, and the whole point of a name here is to be able to
# check, at a glance, that the script is driving the arm you are looking at.
#
# ⚠️ The old names are gone deliberately rather than kept as aliases. An alias would
# let `--arm arm1` keep working while the config files had moved on to B, which is
# exactly the silent-mismatch class this file already exists to prevent.
ARM_SERIALS = {
    "B": "2081337C594E5018",  # was "arm1" — every measurement of 2026-08-10 was this one
    "G": "20593383594E5018",  # was "arm2"
}
DEFAULT_ARM = "B"

# ⭐ MEASURED on this arm, 2026-08-10, not copied from a config file.
# Re-derive at any time with:  uv run scripts/identify_arm.py --yes
# (each motor's gear_ratio register: DM43**40** reports 40.0, DM43**10** reports 10.0)
#
# ⛔ Decoding a motor with the wrong type does NOT raise — it silently mis-scales.
# Position happens to be safe (±12.5 rad on both), but:
#     velocity  DM4310 ±30  vs  DM4340 ±10   -> 3.0x over-read
#     torque    DM4310 ±10  vs  DM4340 ±28   -> 2.8x under-read
# Under-reading torque on the three heaviest joints is the dangerous direction,
# so never let a control loop assume a uniform motor type.
# Joint names and limits, READ OUT of the vendor model rather than invented:
#   third_party/i2rt/i2rt/robot_models/arm/yam/v1/yam.urdf
# The URDF names them joint1..joint8 with no semantics; the descriptions below are
# derived from each joint's rotation axis and its parent/child links. Julien's own
# words map onto two of them exactly: "base spin" = motor 1, "gripper's spin" = motor 6
# (its child link is literally `gripper`).
#
# ⚠️ Limits are in URDF joint coordinates. They are usable directly against raw motor
# positions ONLY because get_robot.py sets `motor_offsets = [0.0] * n` and yam_v1.yml
# sets every direction to +1 — so the two frames coincide on this arm, apart from a
# ±2π wrap correction applied at init. Re-check this if either ever changes.
YAM_JOINTS = {
    1: ("base_yaw", -2.61799, 3.14159),
    2: ("shoulder_pitch", 0.0, 3.66519),
    3: ("elbow_pitch", 0.0, 3.14159),
    4: ("forearm_pitch", -1.69297, 1.5708),
    5: ("wrist_roll", -1.5708, 1.5708),
    6: ("gripper_twist", -2.0944, 2.0944),
    # Motor 7 drives URDF joint7+joint8, two PRISMATIC finger tips (0…0.047 m).
    # Its rotational travel is not expressed in the URDF and `gripper_limits` is
    # null in linear_4310.yml, so there is no trustworthy limit to enforce here.
    7: ("gripper_jaws", None, None),
}

YAM_MOTOR_TYPES = {
    1: "DM4340",
    2: "DM4340",
    3: "DM4340",
    4: "DM4310",
    5: "DM4310",
    6: "DM4310",
    7: "DM4310",  # gripper
}

_patched = False


def patch_gs_usb_for_macos() -> None:
    """Stop a failed kernel-driver detach from aborting the CAN open.

    ``GsUsb.start()`` does, on every non-Windows platform::

        if self.gs_usb.is_kernel_driver_active(0):
            self.gs_usb.detach_kernel_driver(0)

    On macOS that detach fails with ``USBError errno=13 (Access denied)`` even
    though nothing is holding the device. Measured on this machine, with the
    CANable idle: ``is_kernel_driver_active(0)`` is **False** and
    ``usb.util.claim_interface(dev, 0)`` **succeeds** — so there is no kernel
    driver to detach and the call is pointless here. It is nevertheless reached
    intermittently (libusb's darwin backend reports a stale claim after another
    handle has opened the device), and when it is, the whole open dies.

    Suppressing only the ``USBError`` keeps the real behaviour on any platform
    where a driver genuinely must be detached: there the detach still runs and
    still succeeds. This is idempotent.
    """
    global _patched
    if _patched:
        return

    import usb.core

    original = usb.core.Device.detach_kernel_driver

    def detach_kernel_driver(self: Any, interface: int) -> None:
        try:
            original(self, interface)
        except usb.core.USBError:
            # No kernel driver on macOS — verified above. Carry on and claim it.
            pass

    usb.core.Device.detach_kernel_driver = detach_kernel_driver
    _patched = True


_echo_patched = False


def patch_gs_usb_echo_filter() -> None:
    """Stop the adapter's own transmit echoes being mistaken for motor replies.

    ⛔ THIS ONE IS LOAD-BEARING. Without it every read silently returns garbage
    that looks entirely plausible.

    A candleLight adapter echoes each transmitted frame back to the host as a
    send confirmation. python-can surfaces those through the normal receive path
    and only marks them ``is_rx=False``; it does not drop them. SocketCAN does
    not do this — a frame is not looped back to the socket that sent it — so
    I2RT's driver code, written for Linux, never had to filter anything and
    treats the next frame off the bus as the reply.

    Measured consequence, 2026-08-10: every register read returned 0. The read
    request is ``[motor_id, 0x00, 0x33, reg_id, 0x00, 0x00, 0x00, 0x00]`` and
    ``bytes_to_uint32`` decodes ``data[4:8]`` — the four zero bytes of our own
    request. Seven motors "replied" with a perfect set of zeros and the script
    reported success. A wrong answer that looks like an answer is far worse than
    an error, which is why this is patched at the bus layer where nothing can
    bypass it.

    Idempotent.
    """
    global _echo_patched
    if _echo_patched:
        return

    import time as _time

    from can.interfaces.gs_usb import GsUsbBus

    original = GsUsbBus._recv_internal

    def _recv_internal(self: Any, timeout: float | None):  # noqa: ANN202
        deadline = None if timeout is None else _time.time() + timeout
        while True:
            msg, already_filtered = original(self, timeout)
            if msg is None or msg.is_rx:
                return msg, already_filtered
            # Our own transmission coming back. Not a reply — keep waiting.
            if deadline is not None and _time.time() >= deadline:
                return None, already_filtered

    GsUsbBus._recv_internal = _recv_internal
    _echo_patched = True


def resolve_arm(arm: str = DEFAULT_ARM) -> tuple[int, str]:
    """Map an arm name to the gs_usb adapter *index* carrying its serial.

    Returns ``(index, serial)``. Raises rather than guessing: with two identical
    adapters attached, a wrong guess drives the wrong robot.

    The index is only used because I2RT's ``CanInterface`` calls
    ``can.interface.Bus(bustype=..., channel=..., bitrate=...)`` and gives us no
    way to pass gs_usb's ``bus``/``address`` selectors through. So we resolve the
    index here and then **verify the serial after opening** — see
    ``_verify_serial``. Never trust the index alone.
    """
    patch_gs_usb_for_macos()
    from gs_usb.gs_usb import GsUsb

    serial = ARM_SERIALS.get(arm, arm)  # allow passing a raw serial too
    devs = GsUsb.scan()
    if not devs:
        raise RuntimeError("No candleLight CAN adapter found. Is the arm's CANable plugged in?")

    serials = [d.serial_number for d in devs]
    if serial not in serials:
        known = {v: k for k, v in ARM_SERIALS.items()}
        listing = "\n".join(f"    [{i}] {s}  ({known.get(s, 'unknown adapter')})" for i, s in enumerate(serials))
        raise RuntimeError(
            f"Adapter for {arm!r} (serial {serial}) is not attached.\n"
            f"  {len(devs)} adapter(s) present:\n{listing}"
        )
    return serials.index(serial), serial


def _verify_serial(iface: Any, expected: str, arm: str) -> None:
    """Abort unless the bus we just opened really is the arm we asked for.

    The index resolved by :func:`resolve_arm` could in principle go stale between
    our scan and python-can's own scan inside ``GsUsbBus.__init__``. This closes
    that window: a mismatch shuts the bus and raises instead of quietly talking
    to the other robot.
    """
    actual = getattr(getattr(iface.bus, "gs_usb", None), "serial_number", None)
    if actual != expected:
        iface.close()
        raise RuntimeError(
            f"⛔ WRONG ADAPTER: asked for {arm} (serial {expected}) but opened serial {actual}. "
            "Bus closed without transmitting. Do not retry blindly — re-check which arm is which."
        )


_chain_patched = False
GS_USB_CHANNEL_PREFIX = "gsusb"


def chain_channel(arm: str = DEFAULT_ARM) -> str:
    """The channel string to hand `DMChainCanInterface` / `get_yam_robot()`.

    Returns e.g. ``"gsusb1"`` — the gs_usb adapter *index* for `arm`, resolved by
    serial (never by position), wrapped in a form the vendor code will accept.

    ⚠️ The name is load-bearing and must not contain the substring ``"can"``.
    `dm_driver.py:409` branches on ``if "can" in channel:`` and hands anything
    matching straight to SocketCAN, which does not exist here. ``"gsusb1"`` is
    chosen precisely because it fails that test.
    """
    index, _ = resolve_arm(arm)
    return f"{GS_USB_CHANNEL_PREFIX}{index}"


def patch_dm_driver_for_gs_usb() -> None:
    """Let the whole-arm chain reach the CANable, not just single motors.

    ⭐ WHY THIS IS NEEDED AT ALL

    `DMSingleMotorCanInterface` accepts a `bustype`, which is how every
    single-motor script here already works. But the layer above it —
    `DMChainCanInterface`, the one `get_yam_robot()` uses and the only route to
    all seven motors, gravity compensation and the gripper force limiter — does
    not expose it (`dm_driver.py:372-424`). It picks the bus itself::

        if "can" in channel:
            DMSingleMotorCanInterface(channel=channel, bustype="socketcan", ...)
        else:
            DMSingleMotorCanInterface(channel=channel, bitrate=bitrate, ...)

    Both branches end at SocketCAN — the second by falling back to the default.
    There is no argument anywhere that changes it.

    So the interception happens one level lower, at the constructor that *does*
    understand bustype: a channel named ``gsusb<N>`` is rewritten to
    ``(channel=N, bustype="gs_usb")``. Anything else is passed through untouched,
    so behaviour on Linux/SocketCAN is bit-for-bit unchanged.

    ⛔ Deliberately a patch and not a fork. `third_party/i2rt` stays a clean
    upstream checkout that can be re-pulled without merging anything.

    Idempotent.
    """
    global _chain_patched
    if _chain_patched:
        return

    add_i2rt_to_path()
    from i2rt.motor_drivers import dm_driver

    original_init = dm_driver.DMSingleMotorCanInterface.__init__

    def __init__(self: Any, *args: Any, **kwargs: Any) -> None:  # noqa: N807
        channel = kwargs.get("channel")
        if isinstance(channel, str) and channel.startswith(GS_USB_CHANNEL_PREFIX):
            suffix = channel[len(GS_USB_CHANNEL_PREFIX) :]
            if suffix.isdigit():
                kwargs["channel"] = int(suffix)
                kwargs["bustype"] = "gs_usb"
        original_init(self, *args, **kwargs)

    dm_driver.DMSingleMotorCanInterface.__init__ = __init__

    # ⛔ Drain before every enable/disable, or one late reply cascades.
    #
    # `CanInterface._send_message_get_response` retries by sending the command
    # AGAIN. So if a reply arrives a moment late and fails the arbitration-id
    # check, the retry puts a second enable on the wire — and now two replies
    # come back. One satisfies this call; the other is left in the buffer and is
    # read as the NEXT motor's reply, which mismatches, which retries, which
    # leaves another. A single hiccup snowballs down the chain.
    #
    # Measured 2026-08-10 bringing all seven motors up through
    # `DMChainCanInterface._motor_on()`: three consecutive runs failed at motor
    # 4, then motor 7, then succeeded. **The varying failure point is the
    # signature** — a genuinely dead motor fails in the same place every time.
    #
    # I2RT's 3 ms spacing between motors is ample over SocketCAN, where the
    # round trip happens in-kernel. Over libusb each transfer is ~0.45 ms and the
    # margin is thin. Draining first makes each exchange start from an empty
    # buffer, so stale frames cannot be mistaken for the current reply.
    #
    # ⚠️ Deliberately NOT applied to `set_control` — that runs in the 100 Hz
    # control loop, where a drain would cost more than the problem. Enable and
    # disable happen at init and teardown, where a few ms is free.
    for name in ("motor_on", "motor_off"):
        original = getattr(dm_driver.DMSingleMotorCanInterface, name)

        def make(fn: Any) -> Any:
            def wrapper(self: Any, *args: Any, **kwargs: Any) -> Any:
                import time as _t

                last: Exception | None = None
                for attempt in range(3):
                    try:
                        self._drain_bus(timeout_s=0.01 * (attempt + 1), idle_count=3)
                    except Exception:  # noqa: BLE001, S110
                        pass
                    try:
                        return fn(self, *args, **kwargs)
                    except AssertionError as exc:
                        # The vendor's own retry re-sends the command, which is what
                        # creates the surplus reply in the first place. Retrying at
                        # THIS level instead — after a full drain — starts the whole
                        # exchange clean and contains the cascade rather than feeding
                        # it. Only AssertionError: that is the vendor's "no matching
                        # reply" signal. A real CAN fault is a different exception and
                        # must not be swallowed.
                        last = exc
                        _t.sleep(0.02 * (attempt + 1))
                raise last  # type: ignore[misc]

            return wrapper

        setattr(dm_driver.DMSingleMotorCanInterface, name, make(original))

    patch_gs_usb_for_macos()
    patch_gs_usb_echo_filter()
    _chain_patched = True


def add_i2rt_to_path() -> Path:
    """Make the vendored I2RT SDK importable without installing it.

    Deliberately not ``uv pip install -e third_party/i2rt``: that would drag in
    mujoco, viser, rerun and ``ruckig`` (sdist-only, compiles from source) for a
    motor poll that needs none of them. The driver layer imports with just
    numpy, python-can, tyro, pydantic, packaging and crcmod.
    """
    if not I2RT_PATH.exists():
        raise FileNotFoundError(
            f"I2RT SDK not found at {I2RT_PATH}. Clone it with:\n"
            f"  git clone --depth 1 https://github.com/i2rt-robotics/i2rt.git {I2RT_PATH}"
        )
    if str(I2RT_PATH) not in sys.path:
        sys.path.insert(0, str(I2RT_PATH))
    return I2RT_PATH


def open_raw_can_interface(
    *,
    bitrate: int = YAM_BITRATE,
    arm: str = DEFAULT_ARM,
    name: str = "yam_macos_raw",
) -> Any:
    """Return an I2RT ``RawCanInterface`` bound to the CANable.

    This is the interface behind I2RT's register read/write helpers. Reading a
    register (``0x7FF`` sub-command ``0x33``) does **not** enable a motor and
    cannot command motion, which makes it a gentler first contact than
    ``motor_on()``. Writing (``0x55``) and saving (``0xAA``) change motor
    configuration and are deliberately not wrapped here.
    """
    # patch_dm_driver_for_gs_usb() also installs the macOS + echo-filter patches
    # AND the drain/retry hardening around motor_on/motor_off. Calling it here too
    # means every path gets the same robustness -- ping_motors.py was still
    # desyncing (motors 4, 6, 7 silent) purely because it took this route instead.
    patch_dm_driver_for_gs_usb()
    index, serial = resolve_arm(arm)

    from i2rt.motor_config_tool.utils import RawCanInterface

    iface = RawCanInterface(channel=index, bustype="gs_usb", bitrate=bitrate, name=name)
    _verify_serial(iface, serial, arm)
    return iface


def open_motor_interface(
    *,
    bitrate: int = YAM_BITRATE,
    arm: str = DEFAULT_ARM,
    name: str = "yam_macos",
) -> Any:
    """Return an I2RT ``DMSingleMotorCanInterface`` bound to the CANable.

    ⚠️ This opens the bus in **normal** mode. The adapter becomes an active CAN
    node and will acknowledge frames. It transmits no data frames until you call
    a method that does. Callers that poll motors must have been asked for.

    The channel is an integer, not ``"can0"``: python-can's gs_usb backend takes
    an adapter *index*. Passing a ``"can*"`` string is also what routes I2RT's
    higher-level ``DMChainCanInterface`` down its hardcoded SocketCAN branch
    (``dm_driver.py``: ``if "can" in channel``), which is precisely the path
    that does not exist on macOS.
    """
    # patch_dm_driver_for_gs_usb() also installs the macOS + echo-filter patches
    # AND the drain/retry hardening around motor_on/motor_off. Calling it here too
    # means every path gets the same robustness -- ping_motors.py was still
    # desyncing (motors 4, 6, 7 silent) purely because it took this route instead.
    patch_dm_driver_for_gs_usb()
    index, serial = resolve_arm(arm)

    from i2rt.motor_drivers.dm_driver import ControlMode, DMSingleMotorCanInterface

    iface = DMSingleMotorCanInterface(
        control_mode=ControlMode.MIT,
        channel=index,
        bustype="gs_usb",
        bitrate=bitrate,
        name=name,
    )
    _verify_serial(iface, serial, arm)
    return iface
