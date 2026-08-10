#!/usr/bin/env python3
"""
TODO: Rewrite this for the real tools used!
        this is just an example file

verify_setup.py - check the YAM + LeRobot stack layer by layer.

Run inside the venv:
    source .venv/bin/activate && python verify_setup.py

Exits non-zero if anything REQUIRED is broken. Hardware checks are advisory:
they will fail harmlessly if the arm is unplugged.
"""

from __future__ import annotations

import glob
import importlib
import os
import shutil
import subprocess
import sys

OK, WARN, FAIL = (
    "  \033[32mOK  \033[0m",
    "  \033[33mWARN\033[0m",
    "  \033[31mFAIL\033[0m",
)
failures: list[str] = []
warnings_: list[str] = []


def section(name: str) -> None:
    print(f"\n\033[1m{name}\033[0m")


def ok(msg: str) -> None:
    print(f"{OK} {msg}")


def warn(msg: str) -> None:
    print(f"{WARN} {msg}")
    warnings_.append(msg)


def fail(msg: str) -> None:
    print(f"{FAIL} {msg}")
    failures.append(msg)


def sh(cmd: list[str]) -> tuple[int, str]:
    try:
        p = subprocess.run(cmd, capture_output=True, text=True, timeout=20)
        return p.returncode, (p.stdout + p.stderr).strip()
    except Exception as e:  # noqa: BLE001
        return 1, str(e)


# --------------------------------------------------------------------- python --
def check_python() -> None:
    section("Python")
    v = sys.version_info
    if (v.major, v.minor) >= (3, 10):
        ok(f"Python {v.major}.{v.minor}.{v.micro}")
    else:
        fail(f"Python {v.major}.{v.minor} is too old; use 3.10+ (3.11 recommended)")

    if os.environ.get("VIRTUAL_ENV"):
        ok(f"venv active: {os.environ['VIRTUAL_ENV']}")
    else:
        warn("No VIRTUAL_ENV set - are you running inside the venv?")


# ---------------------------------------------------------------------- torch --
def check_torch() -> None:
    section("PyTorch / CUDA")
    try:
        import torch
    except Exception as e:  # noqa: BLE001
        fail(f"torch not importable: {e}")
        return

    ok(f"torch {torch.__version__}")
    major, minor = (int(x) for x in torch.__version__.split(".")[:2])
    if (major, minor) < (2, 7):
        fail(f"torch {torch.__version__} < 2.7, which LeRobot 0.6 requires")

    if torch.cuda.is_available():
        name = torch.cuda.get_device_name(0)
        vram = torch.cuda.get_device_properties(0).total_memory / 1e9
        ok(f"CUDA available: {name}, {vram:.1f} GB")
        if vram < 20:
            warn(
                "Under 20 GB VRAM - ACT and SmolVLA fine, pi0/GR00T need LoRA or cloud"
            )
        elif vram < 40:
            ok(
                "24 GB class: ACT, Diffusion, SmolVLA full fine-tune; pi0/GR00T via LoRA"
            )
    else:
        warn("CUDA not available - recording works, local training does not")


# -------------------------------------------------------------------- lerobot --
def check_lerobot() -> None:
    section("LeRobot")
    try:
        import lerobot
    except Exception as e:  # noqa: BLE001
        fail(f"lerobot not importable: {e}")
        return
    ok(f"lerobot {getattr(lerobot, '__version__', 'unknown')}")

    # CLI entry points introduced/renamed across 0.5 -> 0.6
    for exe in (
        "lerobot-record",
        "lerobot-train",
        "lerobot-teleoperate",
        "lerobot-eval",
        "lerobot-rollout",
        "lerobot-calibrate",
    ):
        if shutil.which(exe):
            ok(f"CLI: {exe}")
        else:
            warn(f"CLI missing: {exe} (may not exist in your version)")

    # Training deps are a separate extra in 0.6
    for mod in ("datasets", "torchvision", "einops"):
        try:
            importlib.import_module(mod)
            ok(f"training dep: {mod}")
        except Exception:  # noqa: BLE001
            fail(f"training dep missing: {mod} - install lerobot[training]")

    # Plugin discovery: LeRobot imports any installed lerobot_robot_* package
    plugins = [
        m
        for m in sys.modules.keys() | set(_installed_top_levels())
        if m.startswith(("lerobot_robot_", "lerobot_teleoperator_", "lerobot_camera_"))
    ]
    if plugins:
        ok(f"plugins discovered: {', '.join(sorted(plugins))}")
    else:
        warn(
            "No lerobot_robot_* plugin installed - '--robot.type=yam' will not resolve yet"
        )


def _installed_top_levels() -> list[str]:
    try:
        from importlib.metadata import distributions

        names = []
        for d in distributions():
            names.append((d.metadata.get("Name") or "").replace("-", "_"))
        return [n for n in names if n]
    except Exception:  # noqa: BLE001
        return []


# ----------------------------------------------------------------------- i2rt --
def check_i2rt() -> None:
    section("i2rt SDK")
    try:
        import i2rt  # noqa: F401

        ok("i2rt importable")
    except Exception as e:  # noqa: BLE001
        fail(f"i2rt not importable: {e}")
        return

    try:
        from i2rt.robots.get_robot import get_yam_robot  # noqa: F401

        ok("get_yam_robot importable")
    except Exception as e:  # noqa: BLE001
        fail(f"get_yam_robot missing: {e} - the SDK API may have changed")

    try:
        import mujoco

        ok(f"mujoco {mujoco.__version__} (needed for gravity comp + sim)")
    except Exception as e:  # noqa: BLE001
        warn(f"mujoco unavailable: {e}")

    # URDF / MJCF models
    hits = glob.glob(
        os.path.expanduser("~/robot/i2rt/i2rt/robot_models/arm/yam/*.urdf")
    )
    if hits:
        ok(f"YAM URDF found: {hits[0]}")
    else:
        warn(
            "YAM URDF not found under ~/robot/i2rt - adjust path if you cloned elsewhere"
        )


# ------------------------------------------------------------------ hardware ---
def check_can() -> None:
    section("CAN bus (advisory)")
    ifaces = sorted(os.path.basename(p) for p in glob.glob("/sys/class/net/can*"))
    if not ifaces:
        warn("No can* interfaces. Adapter unplugged, or gs_usb module not loaded.")
        return
    for i in ifaces:
        rc, out = sh(["ip", "-brief", "link", "show", i])
        state = out.split()[1] if rc == 0 and len(out.split()) > 1 else "?"
        if state == "UP":
            ok(f"{i}: UP")
        else:
            warn(f"{i}: {state} - run bringup_can.sh")

    named = [i for i in ifaces if not i[3:].isdigit()]
    generic = [i for i in ifaces if i[3:].isdigit()]
    if generic and not named:
        warn(
            f"Interfaces still generic ({', '.join(generic)}). "
            "Run 'setup_yam_lerobot.sh can-names' or you will mix up leader and follower."
        )

    if shutil.which("candump"):
        ok("candump available for bus debugging")
    else:
        warn("can-utils not installed")


def check_cameras() -> None:
    section("Cameras (advisory)")
    vids = sorted(glob.glob("/dev/video*"))
    if not vids:
        warn("No /dev/video* devices")
    else:
        ok(f"{len(vids)} video device node(s): {', '.join(vids)}")

    by_id = sorted(glob.glob("/dev/v4l/by-id/*"))
    if by_id:
        ok(f"{len(by_id)} stable by-id path(s):")
        for p in by_id:
            print(f"        {p}")
    else:
        warn(
            "No /dev/v4l/by-id paths - you will be forced to use unstable /dev/videoN indices"
        )

    if shutil.which("ffmpeg"):
        ok("ffmpeg present (required for episode video encoding)")
    else:
        fail("ffmpeg missing - LeRobot cannot encode episodes")


def check_misc() -> None:
    section("Misc")
    if shutil.which("hf") or shutil.which("huggingface-cli"):
        ok("Hugging Face CLI present")
    else:
        warn("HF CLI missing - you will not be able to push datasets to the Hub")

    total, used, free = shutil.disk_usage(os.path.expanduser("~"))
    gb = free / 1e9
    if gb >= 100:
        ok(f"{gb:.0f} GB free in $HOME")
    else:
        warn(f"Only {gb:.0f} GB free in $HOME - video datasets will fill this")

    if os.environ.get("USER") and "dialout" in sh(["id", "-nG"])[1]:
        ok("user in 'dialout' group")
    else:
        warn("user NOT in 'dialout' - log out and back in after running the installer")


# ----------------------------------------------------------------------- main --
def main() -> int:
    print("\033[1mYAM + LeRobot stack verification\033[0m")
    check_python()
    check_torch()
    check_lerobot()
    check_i2rt()
    check_can()
    check_cameras()
    check_misc()

    section("Summary")
    if failures:
        print(f"  {len(failures)} REQUIRED check(s) failed:")
        for f in failures:
            print(f"    - {f}")
    if warnings_:
        print(
            f"  {len(warnings_)} warning(s) - fine if the arm is unplugged, "
            "otherwise worth fixing before you record data."
        )
    if not failures:
        print("  \033[32mCore stack is sound.\033[0m")
        print(
            "  Next: record 5 THROWAWAY episodes and inspect them before "
            "collecting the real 100."
        )
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
