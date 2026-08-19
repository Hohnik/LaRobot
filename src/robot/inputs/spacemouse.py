# TODO: add evdev to uv
from evdev import InputDevice, list_devices, ecodes
import numpy as np
import threading
import select
import time

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
    def __init__(self, zero_timeout: float = 0.05):
        self._dev = open_spacemouse()
        self._idx = {code: i for i, code in enumerate(AXES)}
        self.zero_timeout = zero_timeout

        self._buttons = [False, False]
        self._raw = np.zeros(6)
        self._t = np.full(6, time.monotonic())

        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._lock = threading.Lock()

    def __enter__(self):
        self._thread.start()
        return self

    def __exit__(self, *exc):
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

    def snapshot(self):
        now = time.monotonic()
        with self._lock:  # should not be read and written at the same time
            raw = self._raw.copy()
            t = self._t.copy()
            buttons = tuple(self._buttons)

        raw[(now - t) > self.zero_timeout] = 0.0
        return raw, buttons


if __name__ == "__main__":
    with SpaceMouseReader() as sm:
        try:
            while True:
                raw, btns = sm.snapshot()
                print(
                    "\r" + " ".join(f"{v:+.2f}" for v in raw) + f" {btns}",
                    end="",
                    flush=True,
                )
                time.sleep(0.02)
        except KeyboardInterrupt:
            print()
