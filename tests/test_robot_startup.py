"""The real builder must release every acquired handle when verification fails."""
from contextlib import ExitStack
from types import SimpleNamespace
import sys
from unittest.mock import patch

from yam import robot as module


def build_with(*, read_error=None, jaw=0.5, wrapper_error=None, with_gripper=True, shutdown_error=None, disabled=None):
    events = []

    def read():
        events.append("read")
        if read_error:
            raise read_error
        return [0.0] * 6 + [jaw]

    raw = SimpleNamespace(get_joint_pos=read)

    def acquire(**kwargs):
        events.append("acquire")
        return raw

    def wrap(handle, **kwargs):
        assert handle is raw
        if wrapper_error:
            raise wrapper_error
        events.append("wrap")
        return SimpleNamespace(_robot=handle)

    def shutdown(handle):
        assert handle is raw
        events.append("shutdown")
        if shutdown_error:
            raise shutdown_error
        return list(range(1, 8)) if disabled is None else disabled

    modules = {
        "i2rt.robots.get_robot": SimpleNamespace(get_yam_robot=acquire),
        "i2rt.robots.utils": SimpleNamespace(
            ArmType=SimpleNamespace(YAM="YAM"),
            GripperType=SimpleNamespace(NO_GRIPPER="none", LINEAR_4310="linear")),
    }
    result, error = None, None
    with ExitStack() as stack:
        stack.enter_context(patch.dict(sys.modules, modules))
        for name in ["add_i2rt_to_path", "patch_dm_driver_for_gs_usb"]:
            stack.enter_context(patch.object(module, name, lambda: None))
        stack.enter_context(patch.object(module, "chain_channel", lambda arm: "mock"))
        stack.enter_context(patch.object(module, "load_gripper_limits", lambda arm: [0, 1]))
        stack.enter_context(patch.object(module, "read_raw_gripper_position", lambda arm: 0.5))
        stack.enter_context(patch.object(module, "SafeRobot", wrap))
        stack.enter_context(patch.object(module, "shutdown_robot", shutdown))
        try:
            result = module.build_robot("B", with_gripper=with_gripper)
        except BaseException as exc:
            error = exc
    return result, error, events


def test_good_read_transfers_ownership_without_shutdown():
    result, error, events = build_with()
    assert result is not None and error is None
    assert events == ["acquire", "read", "wrap"]


def test_failed_read_refuses_and_shuts_down():
    result, error, events = build_with(read_error=OSError("read failed"))
    assert result is None and isinstance(error, OSError)
    assert events == ["acquire", "read", "shutdown"]


def test_runtime_read_error_also_shuts_down():
    _, error, events = build_with(read_error=RuntimeError("read failed"))
    assert isinstance(error, RuntimeError) and events[-1] == "shutdown"


def test_invalid_jaw_position_shuts_down():
    _, error, events = build_with(jaw=1.5)
    assert isinstance(error, RuntimeError) and events[-1] == "shutdown"


def test_nan_is_not_a_verified_frame():
    _, error, events = build_with(jaw=float("nan"))
    assert isinstance(error, RuntimeError) and events[-1] == "shutdown"


def test_interrupt_during_verification_shuts_down():
    _, error, events = build_with(read_error=KeyboardInterrupt())
    assert isinstance(error, KeyboardInterrupt) and events[-1] == "shutdown"


def test_wrapper_failure_releases_verified_handle():
    _, error, events = build_with(wrapper_error=RuntimeError("wrapper failed"))
    assert isinstance(error, RuntimeError) and events[-1] == "shutdown"


def test_no_gripper_wrapper_failure_releases_handle():
    _, error, events = build_with(with_gripper=False, wrapper_error=RuntimeError("wrapper failed"))
    assert isinstance(error, RuntimeError) and events == ["acquire", "shutdown"]


def test_cleanup_error_does_not_replace_startup_error():
    original = OSError("read failed")
    _, error, events = build_with(read_error=original, shutdown_error=RuntimeError("close failed"))
    assert error is original and "close failed" in str(error.__notes__)


def test_incomplete_shutdown_is_attached_to_original_error():
    _, error, events = build_with(read_error=OSError("read failed"), disabled=[1, 2])
    assert "[3, 4, 5, 6, 7]" in str(error.__notes__)


def main():
    tests = [v for k, v in globals().items() if k.startswith("test_") and callable(v)]
    passed = 0
    for test in tests:
        try:
            test()
            passed += 1
            print("✓", test.__name__)
        except Exception as exc:
            print("✗", test.__name__, type(exc).__name__, str(exc))
    print(f"{passed}/{len(tests)} passed")
    return int(passed != len(tests))


if __name__ == "__main__":
    sys.exit(main())
