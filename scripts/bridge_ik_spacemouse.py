from pathlib import Path

import mujoco
from robot import CONTROL_HZ
from robot.environment.simulation import Simulation
from robot.inputs.spacemouse import (
    CartesianTarget,
    SpaceMouseReader,
    open_spacemice,
)
from robot.kinematics.cartesian import CartesianKinematics

ROOT = Path(__file__).parents[1]
print(ROOT)
SCENE = ROOT / "assets/put_bottles/put_bottle.xml"
LEFT_JOINTS = slice(0, 6)
model = mujoco.MjModel.from_xml_path(str(SCENE))
LAG_LIMIT = 0.05
GRIPPER_OPEN, GRIPPER_SHUT = 0.0495, 0.0  # TODO: Take a look here
DT = 1 / CONTROL_HZ


def main() -> None:
    robot = Simulation(str(SCENE), realtime=True)
    kin = CartesianKinematics(model=model)

    pose = kin.forward(robot.state[LEFT_JOINTS])
    target = CartesianTarget.from_pose(pose=pose)

    print(f"{target.point}\n{target.rotation}")

    with SpaceMouseReader(open_spacemice()[0]) as sm:
        while True:
            twist, buttons = sm.get_twist()
            target.integrate(twist, DT)

            measured = robot.state
            joints = kin.inverse(measured[LEFT_JOINTS], target.)


if __name__ == "__main__":
    main()
