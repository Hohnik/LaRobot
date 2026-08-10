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
GS_USB_INDEX = 0  # python-can's gs_usb backend indexes adapters, it has no "can0"

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
    index: int = GS_USB_INDEX,
    name: str = "yam_macos_raw",
) -> Any:
    """Return an I2RT ``RawCanInterface`` bound to the CANable.

    This is the interface behind I2RT's register read/write helpers. Reading a
    register (``0x7FF`` sub-command ``0x33``) does **not** enable a motor and
    cannot command motion, which makes it a gentler first contact than
    ``motor_on()``. Writing (``0x55``) and saving (``0xAA``) change motor
    configuration and are deliberately not wrapped here.
    """
    patch_gs_usb_for_macos()
    patch_gs_usb_echo_filter()
    add_i2rt_to_path()

    from i2rt.motor_config_tool.utils import RawCanInterface

    return RawCanInterface(channel=index, bustype="gs_usb", bitrate=bitrate, name=name)


def open_motor_interface(
    *,
    bitrate: int = YAM_BITRATE,
    index: int = GS_USB_INDEX,
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
    patch_gs_usb_for_macos()
    patch_gs_usb_echo_filter()
    add_i2rt_to_path()

    from i2rt.motor_drivers.dm_driver import ControlMode, DMSingleMotorCanInterface

    return DMSingleMotorCanInterface(
        control_mode=ControlMode.MIT,
        channel=index,
        bustype="gs_usb",
        bitrate=bitrate,
        name=name,
    )
