import sys

from robot.inputs.keyboard import Keyboard


def main() -> int:
    keyboard_ok = Keyboard.is_available()
    print(f"{keyboard_ok=}")

    all_ok = all([keyboard_ok])
    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main())
