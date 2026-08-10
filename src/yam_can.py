"""Talk to the YAM's CAN bus from macOS.

I2RT's own stack assumes Linux + SocketCAN throughout. This module is the thin
compatibility layer that makes the same driver code work here, over the CANable
in candleLight (gs_usb) mode via libusb. Import it before opening any bus.

Three things live here, and nothing else:

1. ``patch_gs_usb_for_macos()`` — suppress a spurious kernel-driver detach.
2. ``add_i2rt_to_path()``      — put the vendored SDK on ``sys.path``.
3. ``open_motor_interface()``  — build an I2RT motor interface over gs_usb.

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
    add_i2rt_to_path()

    from i2rt.motor_drivers.dm_driver import ControlMode, DMSingleMotorCanInterface

    return DMSingleMotorCanInterface(
        control_mode=ControlMode.MIT,
        channel=index,
        bustype="gs_usb",
        bitrate=bitrate,
        name=name,
    )
