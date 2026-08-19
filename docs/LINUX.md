# Running this on the Linux PC — the connection, then the port

> **Who this is for:** Julien, setting up remote access to the station PC, and any agent working on the port afterwards. **How to use it:** §1 is yours and needs your hands and your ZeroTier account. §2 is what happens once the connection exists. §3 is the package and permission list for the PC. §4 says which parts of the Linux code are proven and which are designed-and-unverified, which matters more than it sounds.
>
> **Why Linux at all:** the vendor SDK supports Linux and not macOS, the whole walkthrough ran on macOS anyway (with one workaround, see [PLAN.md](PLAN.md) Phase A), and the real station is the Linux PC. So the port is not a rewrite. It is a handful of device-naming differences, and Linux answers most of them more simply than macOS did.

## 1. Getting a connection — your part

ZeroTier builds a private network between machines wherever they are, so the Mac can reach the PC from home as easily as from the office. Both machines join one network, you approve them once, and each gets a fixed private IP.

**On the Linux PC:**

```bash
curl -s https://install.zerotier.com | sudo bash
sudo systemctl enable --now zerotier-one
sudo apt install -y openssh-server && sudo systemctl enable --now ssh
```

**On the Mac:** install the ZeroTier package from <https://www.zerotier.com/download/> (the macOS PKG installer). It runs as a system service afterwards.

**Then, once, in a browser:** open <https://my.zerotier.com>, create a network if you do not have one, and copy its **network ID** (16 hex characters, e.g. `6722f92749d059c7`).

**Join from both machines** (same command on each):

```bash
sudo zerotier-cli join 6722f92749d059c7
```

**Approve both machines** in the ZeroTier web console: open the network, find the **Members** list, and tick the checkbox for each one. Nothing works until you do this, and a device that is joined but not approved looks exactly like a broken network.

**Read the PC's ZeroTier address** on the PC:

```bash
sudo zerotier-cli listnetworks
```

The last column holds the assigned IP, usually something like `10.147.20.x`. Also useful: `sudo zerotier-cli info` prints this node's ID and whether it is ONLINE.

**Make the login keys**, on the Mac. This creates a key pair dedicated to this connection, so nothing else you use SSH for is affected:

```bash
ssh-keygen -t ed25519 -f ~/.ssh/yam_linux -C "mac-to-yam-pc"
```

It asks for a passphrase. Your call: empty means logging in never prompts, a passphrase means it does unless you add it to the agent. Then copy the public half to the PC (this asks for your PC password once, and only you type it):

```bash
ssh-copy-id -i ~/.ssh/yam_linux.pub USERNAME@10.147.20.x
```

**Then tell me three things** and I take it from there: the PC's ZeroTier IP, your username on it, and where you want the repo to live (`~/yam-robotics` unless you say otherwise). I never handle your password, and I will not run anything that moves an arm without your word, exactly as on the Mac.

The first thing that happens with those three facts is this block, which is why they are the only three: it goes into `~/.ssh/config` on the Mac and turns every later command into `ssh yam-pc`.

```
Host yam-pc
    HostName 10.147.20.x          # your PC's ZeroTier IP
    User USERNAME                 # your login on the PC
    IdentityFile ~/.ssh/yam_linux
    ServerAliveInterval 30        # keeps a long session from dropping silently
```

Then `ssh yam-pc uname -a` proves the path end to end without touching anything on the bench.

⚠️ **If both machines sit on the same office network anyway**, plain `ssh USERNAME@<local-ip>` works with no ZeroTier at all. ZeroTier is worth it for reaching the bench from home, and it costs nothing once set up.

## 2. What happens once the connection exists

1. I add an SSH config entry so the machine has a short name (`ssh yam-pc`), and check the connection with a harmless command.
2. `git clone` the repo on the PC (from `Hohnik/LaRobot`, branch `julien/yam-teleop-wip`, which is current), then `uv sync`.
3. `uv run checks/check_platform.py --raw` — **the first real command, and the one that matters.** It prints what the machine provides and, with `--raw`, the exact text it parsed. That output either confirms the Linux device-naming code or shows precisely how the machine differs (§4).
4. `uv run checks/run_tests.py` — the whole suite runs with no hardware. If the port is sound, the total matches the Mac's.
5. Then hardware, in the order the README's bring-up checklist already gives: devices, motor health, dry run, and only then a session.

## 3. What the PC needs

**Packages:**

⛔ **Every command in this section runs ON THE PC.** They are written wrapped in `ssh -t yam-pc` so they can be pasted straight into the Mac's terminal, which is where Julien types — a bare `sudo apt` block was once pasted on the Mac and failed with `apt: command not found` ([FINDINGS §75.6](FINDINGS.md)). `-t` is what lets `sudo` prompt for the password over SSH.

```bash
ssh -t yam-pc 'sudo apt update && sudo apt install -y ffmpeg v4l-utils can-utils git curl'
```

(`uv` and `git` are already installed on this station; on a fresh machine add `curl -LsSf https://astral.sh/uv/install.sh | sh`.)

`ffmpeg` is not optional any more: the training-episode export encodes video with it, and `checks/check_dataset.py` verifies the result with `ffprobe`. `v4l-utils` and `can-utils` are for looking at cameras and CAN by hand when something is odd.

**Group membership** (log out and back in afterwards, or the change does not apply):

```bash
sudo usermod -aG video,plugdev $USER
```

Linux has no per-application camera permission, which is a real difference from macOS: being in the `video` group is the whole gate. That also means an agent on the PC can open cameras itself, so the hand-the-command-over dance that every camera measurement on the Mac needed simply disappears.

**The SpaceMouse** needs a rule before a normal user may open it. Its USB vendor is `256f` (measured on this rig: `256f:c635`, a SpaceMouse Compact):

```bash
echo 'SUBSYSTEM=="hidraw", ATTRS{idVendor}=="256f", TAG+="uaccess"
SUBSYSTEM=="usb", ATTRS{idVendor}=="256f", TAG+="uaccess"' | sudo tee /etc/udev/rules.d/70-spacemouse.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

`uaccess` gives access to whoever is logged in at the seat, which is safer than making the device world-writable.

**The CAN adapters** appear as network interfaces on Linux, and each must be brought up with the bitrate before anything can talk to a motor. This needs `sudo`, so it stays your step:

```bash
ssh -t yam-pc 'sudo ip link set can0 up type can bitrate 1000000 && sudo ip link set can1 up type can bitrate 1000000'
```

⭐ Measured on this station 2026-08-19: **`can0` is arm G and `can1` is arm B**, resolved from the adapters' USB serials in sysfs, and both come up DOWN after every boot ([FINDINGS §75.5](FINDINGS.md)).

`uv run checks/check_platform.py` names which interface belongs to which arm, by reading the adapter's USB serial out of sysfs, and it prints this command for any interface that is still down. To make it survive a reboot, a systemd-networkd file or a udev rule does the same thing permanently, and that is worth doing once the ports are settled.

**Depth**, if wanted: the D405s deliver colour only over the plain webcam protocol, on any OS ([FINDINGS §63.0](FINDINGS.md)). Depth needs Intel's own library, which on Ubuntu is a package rather than the day of compiling it would have cost on macOS:

```bash
sudo apt install -y librealsense2-utils librealsense2-dev
uv add pyrealsense2
```

`check_platform.py` reports whether it is present.

## 4. What is proven, and what is designed-and-unverified

This distinction is the point of §4 existing, and it follows this repo's own rule: never present an assumption as a measurement.

**Proven ON THIS MACHINE, 2026-08-19** (`lavita@10.64.9.60`, Ubuntu 24.04.4 — [FINDINGS §75.2](FINDINGS.md)): `uv sync` from a clean clone, the whole suite at **767/767** (the same total as the Mac), the full simulated session at **31/31**, every checker and falsifier green, the training-episode export at **17/17** on a real recording, and the `states_actions.bin` table **bit-identical to the Mac's**. Also measured rather than assumed: `perf_counter` and `monotonic` are both `CLOCK_MONOTONIC` here and differ by **40 ns**, so the camera-to-joint join is exact on this machine too.

**Still designed and unverified: the CAN and camera device paths only.** No arms or cameras are plugged into the PC yet, so `ip -details link show type can` prints nothing and `/dev/v4l/by-id` does not exist at all (udev creates it when the first camera appears). Both were handled gracefully rather than crashing. The formats in `src/yam/platform.py` come from documentation, and the fixtures in `tests/test_platform.py` say so in their own comments.

**How that gets settled in one command:** with the hardware plugged in, `uv run checks/check_platform.py --raw` prints the raw text beside the parse. Either the formats match and the port is fully confirmed, or they differ and the output shows exactly where. It also prints the `sudo ip link set canX up type can bitrate 1000000` line for each adapter, which is the one CAN step that needs root after every boot.

## 5. The differences from macOS, in one table

| what | macOS (the walkthrough) | Linux (the station) |
|---|---|---|
| CAN transport | gs_usb over libusb, adapter chosen by index and verified by serial after opening | SocketCAN (`can0`), interface resolved directly from the adapter's USB serial in sysfs |
| CAN bring-up | nothing to do | `sudo ip link set can0 up type can bitrate 1000000`, needed once per boot |
| camera identity | AVFoundation uniqueID equals locationID·VID·PID, plus one confirmed index hint per port layout | `/dev/v4l/by-id` names model and serial and links to `/dev/videoN`, which IS OpenCV's index |
| two identical D405s | needed a physical confirmation once per port arrangement | separated by serial from the symlink name, nothing physical needed |
| camera permission | granted per application; an agent can never have it | membership of the `video` group; an agent on the PC can open cameras |
| SpaceMouse | opens exclusively, no rule needed | needs a udev rule before a non-root user may open it |
| depth | would need a day of compiling; measured as colour-only over UVC | `apt install librealsense2-*`, then `pyrealsense2` |
| the clocks | `perf_counter` and `monotonic` are the same clock, measured | both documented as `CLOCK_MONOTONIC`; `check_platform.py` measures it on the first run |
| `camera_view --list` | AVFoundation modes, measured index identification | the `/dev/v4l/by-id` table, since the kernel already answers it |
| `probe_can.py` | libusb listen-only watch of the raw bus | refuses, and prints the `ip link ... listen-only on` + `candump` recipe instead |
| single-motor tools (`ping_motors`, `move_one_motor`, `teleop_gripper`, …) | gs_usb index, serial re-verified after opening | SocketCAN interface, resolved from the serial before opening (nothing left to verify) |

*Written 2026-08-19, when the connection did not exist yet. When it does, the first `check_platform.py --raw` on the real machine turns §4's second list into the first one, and this file should say so.*
