"""See the real command limiter and cleanup owner using a blocked fake joint.

Run from the repository: .venv-teleop/bin/python examples/command_limits.py
No devices are opened. Parking and physical stop decisions belong to the operator.
"""
import time

import numpy as np

from yam.fake.arm import FakeArm
from yam.recording_session import RecordingSession
from yam.robot import SafeRobot, shutdown_robot
from yam.session_resources import SessionResources


def main() -> None:
    resources = SessionResources(RecordingSession(), shutdown_robot=shutdown_robot)
    print("FAKE ARM: joint 0 stays at zero while its requested target is 1 rad.")
    try:
        raw = FakeArm(name="demo")
        # Register the handle before any later setup can fail.
        resources.own_robot("demo", raw, motors=7)
        raw.block(0, 0.0, 0.0)
        robot = SafeRobot(raw, max_speed=1.0, max_lag=0.04)
        target = np.zeros(7)
        target[0] = 1.0

        for _ in range(12):
            robot.command_joint_pos(target)
            # SafeRobot uses the real clock even when its robot is fake.
            time.sleep(0.01)

        print(f"First sent target: {raw.commands[0][0]:.3f} rad (20 ms rate budget)")
        print(f"Last sent target:  {raw.cmd[0]:.3f} rad (0.040 rad lag limit)")
        print(f"Measured position: {raw.get_joint_pos()[0]:.3f} rad")
        print(f"Limited cycles:    {robot.limited_cycles}")
    except BaseException as failure:
        if not resources.close():
            failure.add_note("Fake resource cleanup also failed")
        raise
    else:
        if not resources.close():
            raise RuntimeError("Fake resource cleanup did not complete")
    print("All seven fake motor-off calls completed; no physical arm was used.")


if __name__ == "__main__":
    main()
