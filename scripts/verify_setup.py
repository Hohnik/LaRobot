import sys

from robot.inputs.keyboard import Keyboard
from robot.inputs.spacemouse import SpaceMouse


def main() -> int:
    """Print input-device availability; the keyboard check is unimplemented.

    Returns
    -------
    If both checks complete, 0 when both succeed and 1 otherwise.
    ```
    int
    ```
    """
    keyboard_ok = Keyboard.is_available()
    spacemouse_ok = SpaceMouse.is_available()
    print(f"{keyboard_ok=}")
    print(f"{spacemouse_ok=}")

    all_ok = all([keyboard_ok, spacemouse_ok])
    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main())
