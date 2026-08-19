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

## 2. The connection, as it actually is — and how to work with it

✅ **All of this is live as of 2026-08-19** ([FINDINGS §75](FINDINGS.md)). An agent picking this up needs these six facts and nothing else:

1. **`ssh yam-pc` works from the Mac, with no password.** The host entry is in `~/.ssh/config` (HostName `10.64.9.60`, User `lavita`, IdentityFile `~/.ssh/yam_linux`). The machine calls itself **RoVita**: Ubuntu 24.04.4, 32 cores, 60 GB RAM, 3.4 TB free, **RTX 5090 32 GB**.
2. **The repo lives at `~/yam-robotics` on the PC**, with `uv sync` already done. The team's own repo is beside it at `~/LaRobot`.
3. ⭐ **Code moves Mac → PC by git BUNDLE, never by a push.** Pushing to `Hohnik/LaRobot` needs Julien's word every time ([HANDOFF §4](HANDOFF.md) rule 9), and a bundle needs nobody's. The loop, three commands from the Mac:

```bash
git bundle create /tmp/yam.bundle --all && scp /tmp/yam.bundle yam-pc:/tmp/yam.bundle && ssh yam-pc 'cd ~/yam-robotics && git fetch -q /tmp/yam.bundle "main:refs/remotes/mac/main" && git merge --ff-only refs/remotes/mac/main && git log --oneline -1'
```

   ⭐ **`git merge --ff-only`, and it used to be `git reset --hard`.** Both land the PC on the Mac's commit while the PC's clone is never edited directly, and `--ff-only` is strictly better: it REFUSES if the move is not a fast-forward, so the one case where a hard reset would silently destroy work is the one case this stops. It also passes the agent harness's destructive-command gate, which blocks `reset --hard` over SSH and correctly so ([FINDINGS §76.9](FINDINGS.md) item 1 is the session where that happened).

   ⚠️ Gitignored things (`recordings/`, `third_party/i2rt`, the venv) are untouched either way. ⛔ If `--ff-only` ever refuses, do NOT reach for `--hard` reflexively: something on the PC has diverged, and finding out what comes first.

4. ⚠️ **`third_party/i2rt` is gitignored, so it does NOT travel in the bundle.** It is cloned from upstream at the tag the Mac uses: `git clone --depth 1 --branch v1.3.1 https://github.com/i2rt-robotics/i2rt.git third_party/i2rt`. Both machines are on `1276f63`.
5. ⛔ **Use a fresh connection when group membership matters.** Julien's `~/.ssh/config` sets `ControlMaster auto` globally, so connections are shared and an old one keeps the group list it was opened with. `ssh -o ControlPath=none yam-pc …` after a `usermod` change.
6. ⛔ **`sudo` on the PC needs Julien.** Every command that needs root goes to him wrapped in `ssh -t yam-pc '…'` so it can be pasted on the Mac, which is where he types ([FINDINGS §75.6](FINDINGS.md)).

**What has been verified on the machine** ([FINDINGS §75.2](FINDINGS.md), [§75.5](FINDINGS.md), [§75.7](FINDINGS.md)): `uv sync` from a clean clone · the suite at 773/773, the same total as the Mac · the whole simulated session at 31/31 · every checker and falsifier · the two clocks the frame join depends on, 40 ns apart · the training export at 17/17 with a `states_actions.bin` **byte-identical to the Mac's** · both CAN adapters identified from their serials (`can0` = arm G, `can1` = arm B) · all three camera streams of the D405 classified by pixel format, and both cameras captured by the agent itself.

**What is still untested there, and it is only this:** anything that needs a motor to answer (the CAN links are DOWN until someone with root brings them up) and anything that needs the SpaceMouse (still on the Mac).

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

**The SpaceMouse** needs a rule before a normal user may open it, and the rule form matters more than it looks. The file is in the repo at `config/linux/70-yam-spacemouse.rules` and is already copied to `/tmp` on the station:

```bash
ssh -t yam-pc 'sudo cp /tmp/70-yam-spacemouse.rules /etc/udev/rules.d/ && sudo udevadm control --reload-rules && sudo udevadm trigger && echo INSTALLED'
```

⛔ **It uses `GROUP="plugdev"`, not `TAG+="uaccess"`, and an earlier version of this file recommended the wrong one.** `uaccess` grants access through logind's ACLs to whoever holds an **active local seat**. This station is driven over SSH, and an SSH session is not a local seat, so `uaccess` grants nothing here. A group does, and `lavita` is already in `plugdev` ([FINDINGS §75.9](FINDINGS.md)).

⚠️ Two subsystems are matched on purpose. The hidapi build in use reports libusb-style paths (`9-2:1.0`), so the `usb` rule is the one doing the work today; the `hidraw` rule covers a future build that uses the other backend. `udevadm trigger` in the command applies the new mode to the already-plugged puck, so no replug is needed.

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

## 4a. ⛔ macOS sidecar files arrive with any hand-copied folder

If you copy `recordings/` (or anything else) from the Mac by hand, expect **files named `._<original>`** to arrive with it. They are AppleDouble sidecars: 163 bytes, one extended attribute, no data anyone wants. The station's copy had **813 of them** and they caused four separate wrong answers in one session, including a checker that crashed and a frame count that came out at exactly double and then accused real data of being foreign ([FINDINGS §76.4](FINDINGS.md)).

Every listing of the project's own files goes through `yam/files.py::listing` now, so the tools ignore them, and `check_recordings.py` tells you how many it ignored. Clear them out anyway:

```bash
ssh yam-pc "find ~/yam-robotics/recordings -name '._*' -delete"
```

⚠️ The producer is not established. Plain `tar` on this Mac does not emit them, tested with real extended attributes present and with `COPYFILE_DISABLE=1` and `--no-mac-metadata`. Whatever wrote them preserved only whole-second mtimes. So: check for them after any hand copy rather than trusting one recipe.

## 5. The differences from macOS, in one table

| what | macOS (the walkthrough) | Linux (the station) |
|---|---|---|
| CAN transport | gs_usb over libusb, adapter chosen by index and verified by serial after opening | SocketCAN (`can0`), interface resolved directly from the adapter's USB serial in sysfs |
| CAN bring-up | nothing to do | `sudo ip link set can0 up type can bitrate 1000000`, needed once per boot |
| camera identity | AVFoundation uniqueID equals locationID·VID·PID, plus one confirmed index hint per port layout | `/dev/v4l/by-id` names model and serial and links to `/dev/videoN`, which IS OpenCV's index |
| two identical D405s | needed a physical confirmation once per port arrangement | separated by serial from the symlink name, nothing physical needed |
| camera permission | granted per application; an agent can never have it | membership of the `video` group; an agent on the PC can open cameras |
| ⭐ pixel format | AVFoundation ignores the MJPG request and picks well by itself; measured, it changes nothing | ⛔ **the request is obeyed and it matters.** A C920 at 1280x720 in YUYV is capped at **10.01 fps** by its own firmware, in MJPG it does **29.92**. YUYV is format `[0]`, so it wins by default ([FINDINGS §76.1](FINDINGS.md)) |
| ⭐ D405 colour sizes | 1280x720 and 640x480 give clean photographs; **848x480 is broken** (16-bit data read as 8-bit triplets) | ⚠️ **1280x720 over plain V4L2 is INTERMITTENT here**: measured dead twice and alive twice in four hours. 848x480 and below have never failed, at up to 89.88 fps. Read a real frame and step the size down; never trust `cap.get` ([FINDINGS §76.2](FINDINGS.md)) |
| SpaceMouse | opens exclusively, no rule needed | needs a udev rule before a non-root user may open it |
| depth | measured as colour-only over UVC, and no route to it | ✅ **`uv run --with pyrealsense2`, no root, nothing compiled.** z16 up to 1280x720, streaming alongside colour. It also gets 1280x720 colour at a steady 30.0 fps where plain V4L2 is intermittent ([FINDINGS §76.10](FINDINGS.md)) |
| ⛔ D405 serial | one serial | ⛔ **TWO.** `serial_number` is `260522273162` and `asic_serial_number` is `260323072846`. The USB descriptor, `/dev/v4l/by-id` and every name in this repo use the ASIC one. A librealsense backend must key on `asic_serial_number` ([FINDINGS §76.11](FINDINGS.md)) |
| the clocks | `perf_counter` and `monotonic` are the same clock, measured | both documented as `CLOCK_MONOTONIC`; `check_platform.py` measures it on the first run |
| `camera_view --list` | AVFoundation modes, measured index identification | the `/dev/v4l/by-id` table, since the kernel already answers it |
| `probe_can.py` | libusb listen-only watch of the raw bus | refuses, and prints the `ip link ... listen-only on` + `candump` recipe instead |
| single-motor tools (`ping_motors`, `move_one_motor`, `teleop_gripper`, …) | gs_usb index, serial re-verified after opening | SocketCAN interface, resolved from the serial before opening (nothing left to verify) |

*Written 2026-08-19, when the connection did not exist yet. When it does, the first `check_platform.py --raw` on the real machine turns §4's second list into the first one, and this file should say so.*
