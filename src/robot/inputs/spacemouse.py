# TODO: add evdev to uv
from evdev import InputDevice, list_devices, ecodes
import numpy as np
import threading
import select
import time

# to get standard axe indices of the spacemouse
AXES = (
    ecodes.REL_X,
    ecodes.REL_Y,
    ecodes.REL_Z,
    ecodes.REL_RX,
    ecodes.REL_RY,
    ecodes.REL_RZ,
)


# Make sure the user has read/write access (sudo usermod -aG input $USER)
def open_spacemouse() -> InputDevice:
    devices = [InputDevice(path) for path in list_devices()]
    for device in devices:
        if "3dconnexion" in device.name.lower():
            path = device.path
            break
    else:
        raise ConnectionError("SpaceMouse not found")
    return InputDevice(path)


class SpaceMouseReader:
    def __init__(
        self,
        perm=(0, 1, 2, 3, 4, 5),
        signs=(1, -1, -1, 1, -1, -1),
        zero_timeout: float = 0.05,
        expo=1.6,
        lin_scale=0.12,
        ang_scale=0.8,
    ):
        self._dev = open_spacemouse()
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

    def __enter__(self):
        self._thread.start()
        return self

    def __exit__(self, *exc):  # *exc is for conventions in threading
        self._stop.set()
        self._thread.join(timeout=1.0)
        self._dev.close()

    def _run(self):
        while not self._stop.is_set():
            if select.select([self._dev.fd], [], [], 0.05)[
                0
            ]:  # either if event is in or time passed, for stopping
                for e in self._dev.read():
                    self._handle(e)

    def _handle(self, e):
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

    def _snapshot(self):
        """get the current raw outputs of spacemouse"""
        now = time.monotonic()
        with self._lock:  # should not be read and written at the same time
            raw = self._raw.copy()
            t = self._t.copy()
            buttons = tuple(self._buttons)

        raw[(now - t) > self.zero_timeout] = 0.0
        return raw, buttons

    def get_twist(self):
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
    def __init__(self, p=(0.35, 0.0, 0.25)):
        self.p = np.asarray(
            p, dtype=float
        )  # guessed values, here arm is 35cm in front, 25cm above the base
        self.R = np.eye(3)
        self._steps = 0

    def exp_so3(self, w):
        """compute the rotation matrix"""
        theta = np.linalg.norm(w)
        if theta < 1e-12:
            return np.eye(3)
        k = w / theta
        K = np.cross(np.eye(3), k)
        return np.eye(3) + np.sin(theta) * K + (1 - np.cos(theta)) * (K @ K)

    def integrate(self, twist, dt):
        self.p = self.p + twist[:3] * dt
        self.R = self.exp_so3(twist[3:] * dt) @ self.R

        self._steps += 1
        if self._steps % 200 == 0:  # to avoid accumulative error
            u, _, vt = np.linalg.svd(self.R)
            self.R = u @ vt  # leave out dilution by S


if __name__ == "__main__":
    target = CartesianTarget()

    # DEBUG
    R = target.exp_so3([0, 0, np.pi / 2])
    print(f"sollte 1 sein: {np.linalg.det(R)}")
    print(f"sollte einheitsmatrix sein: {R.T @ R}")
    print(f"[0, 1, 0]: {R @ np.array([1, 0, 0])}")  # does it turn the right way?
    print(f"[0, 0, 1]: {R @ np.array([0, 0, 1])}")
    R = np.eye(3)
    for _ in range(1000):
        R = target.exp_so3(np.array([0, 0, np.pi / 2 / 1000])) @ R
    print(f"[0, 1, 0] {R @ np.array([1, 0, 0])}")
    print(f"1.0: {np.linalg.det(R)}")

    with SpaceMouseReader() as sm:
        try:
            t_prev = time.monotonic()
            while True:
                t_now = time.monotonic()
                dt = t_now - t_prev
                t_prev = t_now
                twist, btns = sm.get_twist()
                target.integrate(twist, dt)
                print(target.R @ np.array([0, 0, 1]), target.R @ np.array([1, 0, 0]))
                time.sleep(0.02)
        except KeyboardInterrupt:
            print()
