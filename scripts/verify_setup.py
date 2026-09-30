import sys

from robot.inputs.spacemouse import SpaceMouse


def main() -> int:
    """Print availability of setup components (robot, inputs, cameras).

    Returns
    -------
    0 when all checks succeed and 1 otherwise.
    ```
    int
    ```
    """
    spacemouse_ok = SpaceMouse.is_available()
    print(f"{spacemouse_ok=}")

    all_ok = all([spacemouse_ok])
    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main())
