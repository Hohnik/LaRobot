from typing import Self, override

import numpy.typing as npt
from evdev import InputDevice, ecodes, list_devices

from robot.inputs.input import Input


def get_keyboard_devices() -> list[InputDevice]:
    """Return a list of InputDevices that are keyboards."""
    keyboards = []

    for path in list_devices():
        dev = InputDevice(path)
        keys = dev.capabilities().get(ecodes.EV_KEY, [])

        if all(key in keys for key in [ecodes.KEY_A, ecodes.KEY_Z, ecodes.KEY_SPACE]):
            keyboards.append(dev)
        else:
            dev.close()

    return keyboards


class Keyboard(Input):
    def __init__(
        self, device: InputDevice, speed: float = 0.12, turn_speed: float = 0.8
    ) -> None:
        self._device = device
        self.speed = speed
        self.turn_speed = turn_speed
        self._held: set[int] = set()

    @classmethod
    def is_available(cls) -> bool:
        return len(get_keyboard_devices()) > 0

    @override
    def __enter__(self) -> Self:
        return self

    def __exit__(self, *exc: object) -> None:
        self._device.close()

    def _read_keys(self) -> None:
        try:
            for event in self._device.read():
                if event.type != ecodes.EV_KEY:
                    continue
                if event.value == 0:  # key up
                    self._held.discard(event.code)
                else:  # key down
                    self._held.add(event.code)
        except BlockingIOError:
            pass  # no new events

    def get_velocity(self) -> tuple[npt.NDArray[np.float64], tuple[bool, ...]]: ...



if __name__ == "__main__":
    keyboards = get_keyboard_devices()
    print("Keyboards:", keyboards)
    dev = InputDevice("/dev/input/event0")

    with Keyboard(keyboards[0], speed=0.12, turn_speed=0.8) as keyboard:
        while "False":
            keyboard._read_keys()
            print(keyboard._held, flush=True)
            import time

            time.sleep(1)
