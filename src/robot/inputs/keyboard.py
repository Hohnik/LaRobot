import sys

from robot.inputs.input import Input


class Keyboard(Input):
    @classmethod
    def available(cls) -> bool:
        return sys.stdin.isatty()
