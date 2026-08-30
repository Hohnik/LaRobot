from pathlib import Path

import numpy as np 

from robot.environment.simulation import Simulation
from robot.inverse_kinematics.cartesian import ARM_JOINTS, CartesianIK


ROOT = Path(__file__).resolve().parents[1]

SCENE = ROOT / "assets" / "put_bottles" / "put_bottle.xml" 

IK_MODEL = (
    ROOT
    / "assets"
    / "put_bottles"
    / "assets"
    / "i2rt_yam"
    / "yam.xml"
)
# bcs we have 12 joints + 2 grippers:
LEFT_ARM = slice(0, ARM_JOINTS) #left joints 
LEFT_GRIPPER = ARM_JOINTS # left gripper
RIGHT_SIDE = slice(ARM_JOINTS + 1, 14) # right joints + right gripper

def test_ik_moves_only_left_arm_towards_target():
    sim = Simulation(str(SCENE))
    ik = CartesianIK(IK_MODEL, site_name="grasp_site")

    initial_state = sim.state.copy() #14 Zustandswerte der Simulation [q1L, q2L,...,q6L,...,greiferL,...]
    initial_pose = ik.forward_kinematics( #räumliche Position und Ausrichtung als 4x4 Matrix
        initial_state[LEFT_ARM]
    )

    target_position = initial_pose[:3, 3].copy() #liefert x,y,z
    target_position[0] += 0.01
    target_rotation = initial_pose[:3, :3].copy() # liefert rotationsmatrix

    old_error = np.linalg.norm(
        target_position - initial_pose[:3, 3]
    )

    for _ in range(30):
        state = sim.state
        action = state.copy()

        action[LEFT_ARM] = ik.solve_target(  #Berechnung der Gelenkziele basierend auf target_position und rotation
            state[LEFT_ARM],
            target_position,
            target_rotation
        )

        sim.step(action)

    final_state = sim.state
    final_pose = ik.forward_kinematics( #Berechung von FK zur Evaluierung wo der Greifer tatsächlich angekommen ist.
        final_state[LEFT_ARM]
    )

    new_error = np.linalg.norm(
        target_position - final_pose[:3, 3]
    )

    assert new_error < old_error
    assert np.isclose(
        final_state[LEFT_GRIPPER],
        initial_state[LEFT_GRIPPER],
        atol=1e-3,
    )

    assert np.allclose(
        final_state[RIGHT_SIDE],
        initial_state[RIGHT_SIDE],
        atol=1e-3,
    )

