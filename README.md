# yam-robotics — SpaceMouse teleop for a YAM arm

> **Status: 2026-08-07, session 1.** Input device **proven readable**. Arm **not yet commanded, by choice**.
> Nothing in this repo has ever transmitted on the CAN bus.

**Goal (Julien, 2026-08-07):** *"at best, I'm able to control the robot arm with the space mouse."*
This session's honest scope: find out what is actually connected, what the toolchain must be, get the
**input** end working end-to-end, and report precisely what is hard.

**The long-range plan is [`docs/Setup-Plan.md`](docs/Setup-Plan.md)** — a 382-line German plan by a friend of
Julien's for a **bimanual YAM setup** built on three papers (**ABC** from Amazon FAR, **ASPIRE**, **ENPIRE**,
optionally RoboTTT). This repo is the very first crawl step of that plan.

---

## 1. Hardware actually attached — verified, not assumed

Enumerated on 2026-08-07 via `ioreg`, `system_profiler` and `hid.enumerate()`.

| Device | ID | State |
|---|---|---|
| **SpaceMouse Compact** (3Dconnexion) | `VID 0x256f PID 0xc635` | ✅ **opens and reads from Python.** Three HID interfaces; the multi-axis one is `usage_page 0x01 / usage 0x08` |
| **HD Pro Webcam C920** (Logitech) | `VID 0x046d PID 0x08e5` | ✅ present, untested |
| **CANable 2.5 "Candlelight"** | `VID 0x1d50 PID 0x606f` | ✅ **enumerates via `gs_usb`.** This is the arm's link — YAM is CAN-based |
| ASIX AX88179A | USB→Ethernet | present; role unknown |
| Generic/Realtek hubs | — | dock |

**No USB-serial devices exist** (`/dev/cu.*` holds only Bluetooth and the debug console). The arm is
therefore **not** a serial device — it is CAN, reached through the CANable.

⚠️ **No 3Dconnexion driver is installed or running**, which is *good*: nothing claims the HID device
exclusively, so raw access works with no vendor software and no permission prompt.

## 2. The platform problem — read this before planning anything

**`docs/Setup-Plan.md` §5 specifies Ubuntu 22.04. This machine is macOS (MacBook Air, Darwin 25.5).**
That is not a detail:

- The CANable in candleLight mode is normally driven by **SocketCAN**, which **does not exist on macOS**.
- The plan assumes a Linux CAN loop at **100 Hz** and suggests a low-latency kernel.
- Any YAM vendor SDK will almost certainly assume SocketCAN.

**But the macOS path is real and was verified today:** `python-can` has a **`gs_usb`** backend that speaks to
candleLight adapters over **libusb**, cross-platform. `libusb` is already installed
(`/opt/homebrew/lib/libusb-1.0.dylib`), and `GsUsb.scan()` finds the adapter. So macOS is *viable for
experimentation*, while Linux remains right for the real rig.

> **Recommendation:** treat this Mac as the **prototyping surface for the input half** (SpaceMouse decode, IK,
> visualisation) and plan on a Linux box for the **closed loop with the arm**. Do not fight macOS for the
> 100 Hz control loop — that is a fight the plan already tells you not to pick.

## 3. What the plan says about the SpaceMouse — the key section

`docs/Setup-Plan.md` **§4** is directly about this, and it is the most important thing in the document:

- The papers teleoperate with **GELLO leader arms** — passive copies giving **joint-space** targets directly.
- A SpaceMouse gives a **cartesian 6-DoF twist** instead. **This is not joint space.**
- ⭐ **Consequence (§4.2): you need IK inside the teleop loop.**
  `SpaceMouse twist → integrate to target EE pose → IK → joint targets → command arm`
- Recommended IK: **`mink`** (MuJoCo-based, already used by ABC), with `cuRobo` later for collision-aware
  resets. Alternative: `PyRoKi`.
- **§4.3 action space** — log *everything* while collecting (SpaceMouse input, resulting EE pose, **and** the
  IK-produced joint angles) so the action space can be chosen per experiment without re-collecting.
  MVP recommendation in the plan: **joint space**, for ABC compatibility.
- **§4.5** warns ABC's DAgger trick relies on GELLO forward-kinematics and is **not** portable to a
  SpaceMouse. Irrelevant for the MVP.

## 4. What works right now

```bash
uv run scripts/probe_hardware.py    # enumerate everything, open the SpaceMouse, listen 5 s
uv run src/spacemouse_live.py       # live 6-DoF bar readout — move the device and watch
```

- `scripts/probe_hardware.py` — enumerates all 26 HID interfaces, isolates the 3Dconnexion ones, opens the
  multi-axis interface and reports whether data flows. **Verified: opens successfully.**
- `src/spacemouse_live.py` — decodes and displays all six axes plus buttons. Handles **both** report shapes
  (split `0x01`/`0x02`, or a combined 13-byte `0x01`) and prints which one this unit uses. **Written and
  compile-checked; not yet run against actual motion** — that needs a human hand on the puck.

**Both are strictly read-only. Neither can move anything.**

## 5. ⛔ Safety boundary held this session

**Nothing has been transmitted on the CAN bus.** The adapter was *enumerated* only — no channel opened, no
frame sent, no bitrate set. Reasons:

1. A wrong bitrate injects error frames onto a live bus.
2. Commanding an arm is a physical action. Julien said *"don't break anything. Don't connect it with
   anything unnecessarily."*
3. **The YAM CAN protocol is not yet known** — see §6. Sending bytes at an unknown protocol is how hardware
   gets damaged.

**Opening the bus requires his explicit go-ahead**, ideally with the arm powered down or e-stopped first.

## 6. What is genuinely unknown — the honest gaps

| Gap | Why it matters |
|---|---|
| ⭐ **The YAM CAN protocol** | Nothing can command the arm without it. Need the vendor SDK / I2RT docs. **The single biggest blocker.** |
| **Is the arm even powered and on the bus?** | Untested — would need a listen-only channel at the right bitrate |
| **Does the YAM SDK run on macOS?** | Almost certainly assumes SocketCAN → probably not |
| **A YAM MJCF/URDF model** | `mink` IK needs one. Unknown whether one is published |
| **Bitrate** | Typically 1 Mbit/s for arms, but that is a guess until documented |

## 7. Next steps, in order

1. **Julien runs `src/spacemouse_live.py` and moves the puck.** Confirms decode and reveals the report shape.
   *Zero risk.*
2. **Find the YAM SDK / CAN protocol** — vendor docs, I2RT GitHub, or whatever the friend used. Until this
   exists, step 4 cannot start.
3. **Webcam check** — trivial, and the plan needs it for data collection anyway.
4. **Listen-only CAN**, arm powered, *with his go-ahead*: confirm the arm chatters and at what bitrate.
5. **IK in simulation first** — `mink` + a YAM model, driven by the real SpaceMouse, rendered on screen.
   **This is the whole teleop loop with no hardware risk**, and it is the right next big step.
6. Only then: close the loop onto the physical arm, on Linux.

*Step 5 is the one to aim for. It makes the interesting half — twist → IK → joint targets — real and
debuggable while the arm sits still.*

## 8. Layout

```
yam-robotics/
├── README.md              # this file: state, findings, next steps
├── docs/Setup-Plan.md     # the friend's 382-line bimanual YAM plan (copy; original in ~/Downloads)
├── scripts/probe_hardware.py
├── src/spacemouse_live.py
└── pyproject.toml         # uv; hidapi · python-can · gs-usb
```
