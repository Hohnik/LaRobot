# yam-robotics — SpaceMouse teleop for a YAM arm

> **Status: session 2, Monday 2026-08-10 — in progress. Hardware is reconnected and re-verified.**
> Arm **not yet commanded**. Nothing here has ever transmitted on the CAN bus.
>
> ## ⭐ The two blockers from session 1 are gone
>
> 1. **The YAM CAN protocol is not unknown — it is published.** I2RT ship a full Python SDK at
>    **[github.com/i2rt-robotics/i2rt](https://github.com/i2rt-robotics/i2rt)**, vendored here at
>    `third_party/i2rt`. §5 of `docs/Setup-Plan.md` had named it all along. See **§6.1**.
> 2. **macOS can drive the CAN bus after all.** The SDK assumes SocketCAN, which does not exist here — but
>    its driver layer takes a `bustype` argument, and the CANable works through python-can's `gs_usb`
>    backend over libusb. **Proven today, not theorised.** See **§2.1**.
>
> **Verified live on 2026-08-10** (`ioreg`, `hid.enumerate()`, `GsUsb.scan()`): SpaceMouse Compact, CANable
> 2.5 Candlelight, C920 and the AX88179A all enumerate; the SpaceMouse **opens**; the CAN adapter **opens in
> listen-only mode at 1 Mbit/s**. Nothing was left in a broken state and no device configuration was changed.

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

### 2.1 …but first contact from macOS is solved — measured 2026-08-10

The blocker was never CAN itself, it was I2RT's SocketCAN assumption. Three facts, each verified in source
or on the wire rather than inferred:

| Fact | Where |
|---|---|
| `CanInterface.__init__(channel, bustype="socketcan", bitrate=1_000_000, …)` builds `can.interface.Bus(bustype=bustype, …)` — **`bustype` is a plain argument, not a hardcoded string** | `third_party/i2rt/i2rt/motor_drivers/can_interface.py:14,21` |
| `DMSingleMotorCanInterface` passes `bustype` straight through, so **`bustype="gs_usb"` reaches python-can untouched** | `dm_driver.py:135,142` |
| The adapter **opens listen-only at 1 Mbit/s** on this Mac: `fclk=160 MHz`, timings computed by python-can, `listen_only_granted=True` | `uv run scripts/probe_can.py` |

⚠️ **The one layer that does NOT work is the one the docs tell you to use.** `get_yam_robot()` → 
`DMChainCanInterface` selects the bus with `if "can" in channel:` and then **hardcodes `bustype="socketcan"`**
(`dm_driver.py:409-417`). There is no argument that overrides it. So the high-level robot object is Linux-only
as shipped; the motor-driver layer beneath it is not. Everything here goes through that lower layer, via
[`src/yam_can.py`](src/yam_can.py).

**Two macOS specifics worth knowing before they cost an hour:**
- python-can's gs_usb backend takes an **adapter index (an int)**, not a `"can0"` string. Passing a name
  containing `"can"` is also exactly what routes I2RT's own code down the SocketCAN branch above.
- `GsUsb.start()` calls `detach_kernel_driver(0)`, which on macOS fails with `USBError errno=13` even though
  nothing holds the device (measured: `is_kernel_driver_active(0)` is `False` and `claim_interface(0)`
  succeeds). `src/yam_can.py` suppresses only that error, on any platform where the detach genuinely matters
  it still runs.

**This does not overturn §2's recommendation.** It makes the Mac enough for bring-up, reading state and
hand-guiding. The 100 Hz closed loop still belongs on Linux — gs_usb over libusb has not been throughput-tested
here, and 7 motors × 2 frames × 100 Hz is a real load.

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
uv run scripts/probe_can.py         # listen-only CAN watch — silent transceiver, cannot even ACK
uv run scripts/ping_motors.py       # DRY RUN by default; --yes transmits (see §5)
```

- `scripts/probe_hardware.py` — enumerates all 26 HID interfaces, isolates the 3Dconnexion ones, opens the
  multi-axis interface and reports whether data flows. **Verified: opens successfully.**
- `src/spacemouse_live.py` — decodes and displays all six axes plus buttons. Handles **both** report shapes
  (split `0x01`/`0x02`, or a combined 13-byte `0x01`) and prints which one this unit uses. **Written and
  compile-checked; not yet run against actual motion** — that needs a human hand on the puck.
  - **⚠️ Device-selection bug found and fixed 2026-08-08, before it could cost bench time.** The old
    `find_device()` fell back to `cands[0]` over both accepted VIDs — and **`0x046D` is Logitech, which is
    both 3Dconnexion's legacy VID *and* the C920 webcam on this very dock**. With the SpaceMouse unplugged
    it would have opened the *webcam*, printed a plausible "Opening HD Pro Webcam C920 …", and then never
    reported motion — indistinguishable from a decode bug, and a genuinely nasty thing to debug at the
    bench. Now: a Logitech device is accepted **only** if it independently identifies as multi-axis
    (`usage_page 0x01 / usage 0x08`); blind fallback is allowed for `0x256F` alone. The script also prints
    the **VID:PID it opened** and warns explicitly when the interface is not the multi-axis one.
    **Verified against four stubbed enumerations** (both present · webcam only · axis interface hidden ·
    nothing at all) — the webcam is never selected, and "webcam only" now correctly reports no device.

- `scripts/probe_can.py` — **new 2026-08-10.** Watches the bus in `GS_CAN_MODE_LISTEN_ONLY`, so the
  transceiver drives no dominant bits at all and cannot even acknowledge a frame. It *verifies* the mode was
  granted and refuses to listen otherwise, rather than silently falling back to a mode that talks.
  **Verified: opens at 1 Mbit/s and reads 0 frames** — see the expectation note below.
- `scripts/ping_motors.py` — **new 2026-08-10, and the only script here that can transmit.** Dry run unless
  given `--yes`. macOS port of I2RT's own `motor_config_tool/ping_motors.py`; the CAN traffic is identical.
- `src/yam_can.py` — the macOS CAN layer (§2.1). Imported by both of the above.

**The first three are strictly read-only. `ping_motors.py --yes` is not — see §5.**

> ⚠️ **A quiet bus is the expected reading, not a fault.** DM motors are request/response: they answer when
> polled and say nothing otherwise. With nothing driving the bus, **zero frames is what a healthy, idle,
> correctly-wired arm looks like.** Passive listening therefore cannot confirm the arm is alive; only a poll
> can, and a poll transmits. Do not read silence as a wiring problem.

## 5. ⛔ Safety boundary held this session

**Nothing has been transmitted on the CAN bus.** The adapter was *enumerated* only — no channel opened, no
frame sent, no bitrate set. Reasons:

1. A wrong bitrate injects error frames onto a live bus.
2. Commanding an arm is a physical action. Julien said *"don't break anything. Don't connect it with
   anything unnecessarily."*
3. **The YAM CAN protocol is not yet known** — see §6. Sending bytes at an unknown protocol is how hardware
   gets damaged.

**Opening the bus requires his explicit go-ahead**, ideally with the arm powered down or e-stopped first.

**Updated 2026-08-10.** Reasons 1 and 3 have since been answered — the bitrate is documented at 1 Mbit/s and
the protocol is I2RT's own (§6.1) — and the bus *has* now been opened, but only in **listen-only** mode,
where the transceiver is electrically silent. **Still true: nothing has ever been transmitted.**

Reason 2 stands and is the live boundary. The next step, `scripts/ping_motors.py --yes`, energises motors on
a physical arm:

- It sends only enable/disable per motor and **never a torque, position or velocity setpoint**, so the arm
  should stay limp — a disabled DM motor is already back-drivable, and enabling without a command adds no
  torque.
- **But it is a physical action.** Arm clear of people and obstructions, power reachable, and a motor holding
  a stale setpoint from an earlier session could twitch.
- It stays gated behind `--yes`, and behind Julien saying so.

## 6. What is genuinely unknown — the honest gaps

**Resolved on 2026-08-10** (kept visible, because "we already looked into that" is the cheapest thing to lose):

| Was a gap | Answer |
|---|---|
| ⭐ The YAM CAN protocol | **Published.** I2RT's SDK — §6.1 |
| Does the YAM SDK run on macOS? | **The driver layer does.** Not the `get_yam_robot()` layer — §2.1 |
| A YAM MJCF/URDF model | **Ships with the SDK**, `third_party/i2rt/i2rt/robot_models/arm/` — and `mink` is already an SDK dependency |
| Bitrate | **1 Mbit/s, documented**, no longer a guess |

**Still open:**

| Gap | Why it matters |
|---|---|
| ⭐ **Is the arm powered and on the bus?** | The only way to find out is to poll, which transmits — §5 |
| **Which YAM variant and gripper** | `get_yam_robot()` needs `arm_type` + `gripper_type`; they change motor types and limits |
| **Is there a Linux machine yet?** | The 100 Hz closed loop belongs there, and the plan's whole stack assumes Ubuntu 22.04 |
| **gs_usb throughput on macOS** | Untested. Fine for bring-up; unknown at 7 motors × 2 frames × 100 Hz |

### 6.1 The SDK — what it gives us

`third_party/i2rt` (vendored, `git clone --depth 1`, **gitignored**). It is far more than a protocol spec:

- **`i2rt/motor_drivers/`** — the DM motor protocol: enable `…FC`, disable `…FD`, clear-error `…FB`,
  save-zero `…FE`; MIT and position/velocity control modes; feedback decoding.
- **`i2rt/robot_models/arm/`** — URDF/MJCF for `yam`, `yam_pro`, `yam_ultra`, `yam_ultra_2`, `big_yam`.
  **This is the model `mink` needs**, so plan step 5 (IK in sim) has its missing piece.
- **`examples/`** — `minimum_gello`, `control_with_mujoco`, `control_with_viser`, `record_replay_trajectory`.
- **Safety-relevant:** motors ship with a **400 ms command timeout**. I2RT's own warning is that without it a
  failed gravity-compensation loop can produce uncontrolled torque. **Leave it at the factory default.**

**Do not `uv pip install -e third_party/i2rt`** — it pulls mujoco, viser, rerun and `ruckig` (sdist-only,
compiles from source) for a motor poll that needs none of them. `src/yam_can.py` puts it on `sys.path`
instead; the driver layer imports with just numpy, python-can, tyro, pydantic, packaging and crcmod.
**Verified: it imports cleanly on macOS under Python 3.12**, despite the SDK's README saying 3.11
(`requires-python = ">=3.10"`).

## 6.5 Why this is its own repo and not part of Mind Understanding

Julien asked directly (2026-08-07), and the honest answer has two halves.

**What actually happened:** he said the task was *"completely unrelated"*, and his standing convention is that
standalone projects live in `~/Developer/Projects/` (`ai_book`, `AutonomousMAS`, `LearningApp`…). I followed
that convention **without deliberating hard about it at the time**. The reasoning below is partly
reconstructed — but it does hold up, and the decision is cheap to reverse if he disagrees.

**Why it holds up:**

1. **Different kind of thing.** Mind Understanding is a *learning programme* — "understand what intelligence
   actually is", organised as topics, concepts and experiments, with no deadline. This is an *engineering
   build* for a specific robotics programme with a friend and a professor: real hardware, a Linux target, a
   collaborator's plan, and milestones. A **project**, not a **topic**.
2. **Physical incompatibility.** Mind Understanding lives inside the **iCloud/Obsidian container** so its
   markdown reaches his phone. Its `state/NOW.md` §5 documents that ~1.8 GB of binaries once produced a
   ~3-hour sync backlog, which is why `.venv`, `imports/**/raw` and even `.git` are deliberately held
   *outside* it. A robotics repo accrues exactly the wrong things — virtualenvs, MCAP/rosbag logs, camera
   recordings, model checkpoints. Putting it there would attack that architecture head-on.
3. **Different lifecycle.** This may well move to a Linux machine (§2). Mind Understanding is Mac-and-phone.

**The counter-argument, which is real:** Mind Understanding's `DESIGN.md` §6.2 explicitly struck
"content-on-demand" *because* material otherwise stays scattered across old repos forever — and a new
separate repo is arguably that same failure.

**How that is resolved without merging them:** the concern in §6.2 is that things get **lost**, not that they
are stored apart. Mind Understanding now has `canon/SOURCES.md`, an index of every asset wherever it lives —
built the same day, for exactly this. **This repo is indexed there.** Knowledge that comes *out* of this work
(IK, VLA architectures, teleop as a data-collection problem) flows into Mind Understanding as topics; the
rig, drivers and logs stay here.

> **Reversible if he prefers otherwise.** Nothing here depends on the location: no absolute paths into it,
> and its own git history is self-contained. Moving it under `Mind Understanding/lab/` later is a `git mv`
> plus a `.gitignore` line — but the iCloud-binary problem in point 2 would need answering first.

## 7. Next steps, in order

✅ Done 2026-08-10: hardware re-verified · protocol found (§6.1) · macOS CAN path proven (§2.1) ·
listen-only probe written and green.

1. **Julien runs `src/spacemouse_live.py` and moves the puck.** Confirms decode and reveals which of the two
   report shapes this unit uses. *Zero risk, ~30 seconds.* **Still the one thing that has never been tested
   against real motion**, and it has been the top of this list since Friday.
2. ⭐ **`scripts/ping_motors.py --yes`** — the first transmission. Answers "is the arm alive, which motor IDs
   exist, and what are their positions". **Needs his go-ahead and a clear arm** (§5).
3. **Read joint positions continuously** — a small loop over the driver layer. Proves the full state pipeline
   before any command is ever issued.
4. **Gravity compensation / zero-gravity mode**, so the arm can be hand-guided. First time the arm actually
   holds torque; treat it as its own gated step, not a continuation of 3.
5. **IK in simulation** — `mink` + the YAM MJCF from `third_party/i2rt/i2rt/robot_models/arm/`, driven by the
   real SpaceMouse, rendered on screen. **The whole teleop loop with no hardware risk.** Both pieces now
   exist, so this is no longer blocked on anything.
6. **Webcam check** — trivial, and the plan needs it for data collection anyway.
7. Only then: close the loop onto the physical arm — and by then, ideally on Linux.

*Step 5 remains the one to aim for. It makes the interesting half — twist → IK → joint targets — real and
debuggable while the arm sits still. Steps 2–4 are what make it worth trusting when it is finally connected.*

## 8. Layout

```
yam-robotics/
├── README.md                  # this file: state, findings, next steps
├── docs/Setup-Plan.md         # the friend's 382-line bimanual YAM plan (copy; original in ~/Downloads)
├── scripts/
│   ├── probe_hardware.py      # HID enumeration + open the SpaceMouse          read-only
│   ├── probe_can.py           # listen-only CAN watch                          read-only
│   └── ping_motors.py         # ⚠️ the only script that can transmit (--yes)
├── src/
│   ├── spacemouse_live.py     # live 6-DoF readout                             read-only
│   └── yam_can.py             # the macOS CAN layer (§2.1)
├── third_party/i2rt/          # vendored I2RT SDK — GITIGNORED, clone with the command in §6.1
└── pyproject.toml             # uv; hidapi · python-can · gs-usb · numpy · tyro · pydantic · crcmod-plus
```
