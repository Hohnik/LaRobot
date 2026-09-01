import grp
import os
import select
import threading
import time
from collections.abc import Sequence
from typing import Self

import numpy as np
import numpy.typing as npt
from evdev import InputDevice, InputEvent, ecodes, list_devices

# to get standard axe indices of the spacemouse
AXES = (
    ecodes.REL_X,
    ecodes.REL_Y,
    ecodes.REL_Z,
    ecodes.REL_RX,
    ecodes.REL_RY,
    ecodes.REL_RZ,
)


def open_spacemice() -> list:
    """
    get InputDevices for all connected Spacemice
    """

    assert grp.getgrnam("input").gr_gid in os.getgroups(), (
        "add user to input group: 'sudo usermod -aG input $USER'"
    )

    devices = [InputDevice(path) for path in list_devices()]
    mice = []
    for device in devices:
        if "3dconnexion" in device.name.lower():
            mice.append(device)
    if mice == []:
        raise ConnectionError("No spacemouse found")
    return mice


class SpaceMouseReader:
    """
    perm: permutations to switch out axes of the mouse
    signs: directions of movement
    zero_timeout: after this time raw values will be set to zero, prevent getting stuck
    expo: exponent to smooth out movement speed
    scales: scaling for linear and angular movements
    """

    def __init__(
        self,
        dev: InputDevice,
        perm: Sequence[int] = (0, 1, 2, 3, 4, 5),
        signs: Sequence[float] = (1.0, -1.0, -1.0, 1.0, -1.0, -1.0),
        zero_timeout: float = 0.05,
        expo: float = 1.6,
        lin_scale: float = 0.12,
        ang_scale: float = 0.8,
    ) -> None:
        self._dev = dev
        self._idx = {code: i for i, code in enumerate(AXES)}
        self.zero_timeout = zero_timeout
        self.perm = list(perm)
        self.signs = np.asarray(signs, dtype=float)
        self.expo = expo
        self.lin_scale = lin_scale
        self.ang_scale = ang_scale

        self._buttons = [False, False]
        self._raw = np.zeros(6)
        self._t = np.full(6, time.monotonic())

        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._lock = threading.Lock()

    def __enter__(self) -> Self:
        self._thread.start()
        return self

    def __exit__(self, *exc: object) -> None:  # *exc is for conventions in threading
        self._stop.set()
        self._thread.join(timeout=1.0)
        self._dev.close()

    def _run(self) -> None:
        while not self._stop.is_set():
            if select.select([self._dev.fd], [], [], 0.05)[
                0
            ]:  # either if event is in or time passed, for stopping
                for e in self._dev.read():
                    self._handle(e)

    def _handle(self, e: InputEvent) -> None:
        """update classes raw values"""
        now = time.monotonic()  # forward passing time
        with self._lock:
            if e.type == ecodes.EV_REL:
                i = self._idx[e.code]
                self._raw[i] = e.value / 350.0  # max value
                self._t[i] = now
            elif e.type == ecodes.EV_KEY:
                self._buttons[e.code - ecodes.BTN_0] = (
                    e.value != 0
                )  # 256 to 0 and 257 to 1

    def _snapshot(self) -> tuple[npt.NDArray[np.float64], tuple[bool, ...]]:
        """get the current raw outputs of spacemouse"""
        now = time.monotonic()
        with self._lock:  # should not be read and written at the same time
            raw = self._raw.copy()
            t = self._t.copy()
            buttons = tuple(self._buttons)

        raw[(now - t) > self.zero_timeout] = 0.0
        return raw, buttons

    def get_twist(self) -> tuple[npt.NDArray[np.float64], tuple[bool, ...]]:
        """get velocity"""
        raw, buttons = self._snapshot()
        v = raw[(self.perm)] * self.signs
        v = np.clip(v, -1.0, 1.0)

        smoothed = np.sign(v) * np.abs(v) ** self.expo

        twist = np.empty(6)
        twist[:3] = smoothed[:3] * self.lin_scale
        twist[3:] = smoothed[3:] * self.ang_scale

        return twist, buttons


class CartesianTarget:
    def __init__(
        self,
        point: np.ndarray = np.zeros(3),
        rotation: np.ndarray = np.eye(3),
    ) -> None:
        self.point = point
        self.rotation = rotation
        self._steps = 0

    @classmethod
    def from_pose(cls, pose: np.ndarray) -> Self:
        return cls(point=pose[:3, 3], rotation=pose[:3, :3])

    @staticmethod
    def exp_so3(w: npt.NDArray[np.float64]) -> npt.NDArray[np.float64]:
        """
        compute the rotation matrix
        w: the last 3 values of the twist
        """
        theta = np.linalg.norm(w)
        if theta < 1e-12:
            return np.eye(3)
        k = w / theta
        K = np.cross(np.eye(3), k)
        return np.eye(3) + np.sin(theta) * K + (1 - np.cos(theta)) * (K @ K)

    def integrate(self, twist: npt.NDArray[np.float64], dt: float) -> None:
        self.point = self.point + twist[:3] * dt
        self.rotation = self.exp_so3(twist[3:] * dt) @ self.rotation

        self._steps += 1
        if self._steps % 200 == 0:  # to avoid accumulative error
            u, _, vt = np.linalg.svd(self.rotation)
            d = np.linalg.det(u @ vt)
            self.rotation = u @ np.diag([1.0, 1.0, d]) @ vt  # leave out dilution by S


if __name__ == "__main__":
    target = CartesianTarget()
    #
    # # DEBUG
    # R = target.exp_so3([0, 0, np.pi / 2])
    # print(f"sollte 1 sein: {np.linalg.det(R)}")
    # print(f"sollte einheitsmatrix sein: {R.T @ R}")
    # print(f"[0, 1, 0]: {R @ np.array([1, 0, 0])}")  # does it turn the right way?
    # print(f"[0, 0, 1]: {R @ np.array([0, 0, 1])}")
    # R = np.eye(3)
    # for _ in range(1000):
    #     R = target.exp_so3(np.array([0, 0, np.pi / 2 / 1000])) @ R
    # print(f"[0, 1, 0] {R @ np.array([1, 0, 0])}")
    # print(f"1.0: {np.linalg.det(R)}")
    #
    dev = open_spacemice()[0]
    with SpaceMouseReader(dev) as sm:
        try:
            t_prev = time.monotonic()
            while True:
                t_now = time.monotonic()
                dt = t_now - t_prev
                t_prev = t_now
                twist, btns = sm.get_twist()
                target.integrate(twist, dt)
                print(
                    target.rotation @ np.array([0, 0, 1]),
                    target.rotation @ np.array([1, 0, 0]),
                    flush=True,
                )
                time.sleep(0.02)
        except KeyboardInterrupt:
            print()
