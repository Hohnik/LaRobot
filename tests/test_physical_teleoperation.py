"""Exercise physical control and failure cleanup without connecting hardware."""

import argparse
import importlib.util
from pathlib import Path
from unittest.mock import MagicMock

import numpy as np
import pytest

from robot.arm.physical_teleoperation import ArmState, update_arm
from robot.kinematics.cartesian_target import CartesianTarget

SCRIPT = Path(__file__).parents[1] / "scripts" / "start_phys.py"
spec = importlib.util.spec_from_file_location("start_phys", SCRIPT)
start_phys = importlib.util.module_from_spec(spec)
spec.loader.exec_module(start_phys)


def test_update_uses_feedback_limits_lag_and_keeps_gripper_target():
    kin = MagicMock()
    kin.forward.return_value = np.eye(4)
    measured = np.arange(6, dtype=float) / 10
    kin.inverse.return_value = measured
    arm = ArmState(kin, CartesianTarget(), gripper=0.4)
    velocities = np.array([1.0, 0, 0, 0, 0, 0])

    command = update_arm(arm, measured, velocities, [0, 1], dt=0.1)

    np.testing.assert_allclose(arm.target.position, [0.07, 0, 0])
    np.testing.assert_array_equal(kin.inverse.call_args.args[0], measured)
    assert kin.inverse.call_args.kwargs["dt"] == 0.1
    np.testing.assert_allclose(command, [*measured, 0.45])
    command = update_arm(arm, measured, np.zeros(6), [0, 0])
    assert command[6] == pytest.approx(0.45)


@pytest.mark.parametrize(
    ("gripper", "buttons", "expected"),
    [(0.99, [1, 0], 1.0), (0.01, [0, 1], 0.0)],
)
def test_mirrored_gripper_buttons_and_bounds(gripper, buttons, expected):
    kin = MagicMock()
    kin.forward.return_value = np.eye(4)
    kin.inverse.return_value = np.zeros(6)
    arm = ArmState(kin, CartesianTarget(), gripper, close_button=1, open_button=0)
    command = update_arm(arm, np.zeros(6), np.zeros(6), buttons)
    assert command[6] == expected


@pytest.fixture
def hardware(monkeypatch):
    monkeypatch.setattr(start_phys.socket, "if_nametoindex", lambda _: 1)
    mouse = MagicMock()
    mouse.connected_paths.return_value = ["left-mouse", "right-mouse"]
    mouse.return_value.__enter__.return_value.read.return_value = (np.zeros(6), [0, 0])
    monkeypatch.setattr(start_phys, "SpaceMouse", mouse)
    monkeypatch.setattr(start_phys.mujoco.MjModel, "from_xml_path", lambda _: None)
    kin = MagicMock()
    kin.return_value.forward.return_value = np.eye(4)
    monkeypatch.setattr(start_phys, "CartesianKinematics", kin)
    factory = MagicMock()
    monkeypatch.setattr(start_phys, "get_yam_robot", factory)
    monkeypatch.setattr("builtins.input", lambda _: "")
    args = argparse.Namespace(
        device="spacemouse",
        dual=True,
        left_channel="can-left",
        right_channel="can-right",
    )
    return args, factory, mouse


def test_second_arm_failure_cleans_up_first(hardware):
    args, factory, mouse = hardware
    left = MagicMock()
    factory.side_effect = [left, RuntimeError("right arm failed")]
    with pytest.raises(RuntimeError, match="right arm failed"):
        start_phys.main(args)
    left.enter_gravity_comp_idle.assert_called_once()
    left.close.assert_called_once()
    assert mouse.return_value.__exit__.call_count == 2


def test_missing_interface_does_not_activate_arms(hardware, monkeypatch):
    args, factory, _ = hardware
    monkeypatch.setattr(
        start_phys.socket, "if_nametoindex", MagicMock(side_effect=OSError())
    )
    with pytest.raises(ConnectionError, match="can-left"):
        start_phys.main(args)
    factory.assert_not_called()


def test_missing_mouse_does_not_activate_arms(hardware):
    args, factory, mouse = hardware
    mouse.connected_paths.return_value = ["only-one"]
    with pytest.raises(ConnectionError, match="Need 2"):
        start_phys.main(args)
    factory.assert_not_called()


def test_dual_tick_and_overrun_without_catch_up(hardware, monkeypatch):
    args, factory, _ = hardware
    left, right = MagicMock(), MagicMock()
    left.get_joint_pos.return_value = np.array([0, 0, 0, 0, 0, 0, 0.3])
    right.get_joint_pos.return_value = np.array([0, 0, 0, 0, 0, 0, 0.7])
    factory.side_effect = [left, right]
    update = MagicMock(
        side_effect=lambda arm, *_args, **_kw: np.r_[np.zeros(6), arm.gripper]
    )
    monkeypatch.setattr(start_phys, "update_arm", update)
    monkeypatch.setattr(start_phys.time, "perf_counter", MagicMock(side_effect=[0, 1]))
    sleep = MagicMock(side_effect=KeyboardInterrupt())
    monkeypatch.setattr(start_phys.time, "sleep", sleep)
    with pytest.raises(KeyboardInterrupt):
        start_phys.main(args)
    assert [call.kwargs["channel"] for call in factory.call_args_list] == [
        "can-left",
        "can-right",
    ]
    assert update.call_args_list[0].args[0].close_button == 1
    assert update.call_args_list[1].args[0].close_button == 0
    assert left.command_joint_pos.call_args.args[0][6] == 0.3
    assert right.command_joint_pos.call_args.args[0][6] == 0.7
    assert sleep.call_args.args[0] == pytest.approx(start_phys.DT)
    for robot in (left, right):
        robot.enter_gravity_comp_idle.assert_called_once()
        robot.close.assert_called_once()


def test_shutdown_attempts_all_arms_even_when_idle_and_close_fail(monkeypatch):
    left, right = MagicMock(), MagicMock()
    left.enter_gravity_comp_idle.side_effect = RuntimeError("idle failed")
    right.close.side_effect = RuntimeError("close failed")
    prompt = MagicMock(return_value="")
    monkeypatch.setattr("builtins.input", prompt)
    with pytest.raises(RuntimeError, match="close failed"):
        start_phys.shutdown_robots({"left": left, "right": right})
    right.enter_gravity_comp_idle.assert_called_once()
    prompt.assert_called_once()
    left.close.assert_called_once()


def test_prompt_eof_still_closes_arms(monkeypatch):
    robot = MagicMock()
    monkeypatch.setattr("builtins.input", MagicMock(side_effect=EOFError()))
    with pytest.raises(EOFError):
        start_phys.shutdown_robots({"left": robot})
    robot.close.assert_called_once()
