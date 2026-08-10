# yam-robotics — SpaceMouse teleop for a YAM arm

> # ▶▶ NO CONTEXT? READ **[docs/HANDOFF.md](docs/HANDOFF.md)** FIRST.
> It rebuilds the whole picture — state, working rules, traps, and what to do next — without this file.
>
> **Status: session 2, Monday 2026-08-10, 09:30-14:00 CEST — closed.**
> ⭐ **A SpaceMouse drove the real arm.** Gravity compensation holds to 0.61°, hand-guiding works, and the
> cartesian teleop loop runs on hardware at 100 Hz.
> ⛔ **The gripper is deliberately NOT controlled** (`NO_GRIPPER` default) — motor 7 was cooked three times;
> the mechanism and the fix are in [docs/FINDINGS.md](docs/FINDINGS.md) §3.5.
>
> *(previous status below)*
>
> **Session 2 — in progress.**
> ⭐ **Both arms are alive and under control.** Motor 6 (`gripper_twist`) has been driven on each arm
> individually and on **both simultaneously**, from a single 100 Hz loop across two independent CAN buses.
> The arms have **only ever been commanded on that one joint** — no multi-joint motion yet, by choice.
> Full log: **§0**. Ordered plan with reasoning: **[docs/ROADMAP.md](docs/ROADMAP.md)**.
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

## 0. Session log

**Session 2 — Monday 2026-08-10, 09:30-11:30 CEST (~2 h).** Both arms went from "we have never transmitted a
CAN frame" to "both arms driven simultaneously from one control loop". Ordered as it happened; the commit for
each carries the full reasoning.

| time | what | why it mattered |
|---|---|---|
| 09:48 | **YAM protocol found** — I2RT's SDK, vendored | Session 1 called this "the single biggest blocker". It was named in `docs/Setup-Plan.md` §5 all along |
| 09:48 | **macOS drives CAN** — `bustype="gs_usb"` over libusb | The SocketCAN assumption was one argument deep, not architectural |
| 10:22 | **SpaceMouse verified, all 6 axes** | Open since Friday. Also fixed the HID seize that cost Julien control of his cursor |
| 10:27 | **All 7 motors identified** — without energising any | `gear_ratio` register: DM43**40**→40.0, DM43**10**→10.0 |
| 10:33 | **First motor energised** (gripper) — and stayed still | torque ≈ 0 confirmed that enabling adds no force |
| 10:48 | ⭐ **100 Hz measured, 3× headroom** | Refuted session 1's "do not fight macOS for the control loop". Teleop need not wait for Linux |
| 10:58 | ⛔ **Adapter selection by serial** | Arm 2 was plugged in and became index 0. Index selection would have moved the **wrong arm** |
| 11:07 | ⭐ **First visible commanded motion** — ±86°, 3 cycles | The earlier 7.6° move was real but too small to see |
| 11:16 | **Two arms proven distinct** — per-unit `inertia` fingerprint | Replaced a claim that its own evidence could not support |
| 11:22 | ⭐ **Bimanual** — both arms, one 100 Hz loop, two CAN buses | Different amplitude, speed and direction per arm, verified by eye |
| 11:30 | **`get_yam_robot(sim=True)` works on macOS** | Same API as hardware ⇒ the whole teleop stack can be built and debugged with zero risk |
| 11:45 | **Cartesian IK teleop validated in simulation** | Scripted circle traced to spec: radius = speed/0.8, period 2π/0.8 |
| 12:0x | ⭐ **Gravity compensation on hardware** | Arm held its own 4.3 kg for 12 s, worst drift **0.61°** |
| 12:2x | **Gripper limits measured once and cached** | `+0.0704 … −5.0528` (78% of declared stroke). No more slamming at every startup |
| 12:4x | ⭐⭐ **SpaceMouse drove the real arm** | Hand-posed, then EE moved 0.15 m under puck control. **The goal, reached** |
| 12:4x | ⛔ **Motor 7 over-temperature; loop kept running blind** | See [docs/FINDINGS.md](docs/FINDINGS.md) §4 — the single most important incident of the day |

> ### ⚠️ The pattern worth carrying forward: **this stack fails by lying, not by crashing.**
> ⭐ **Full catalogue, with all nine, in [docs/FINDINGS.md](docs/FINDINGS.md) §0 — read that first.**
> Six of the nine defects today produced *confident, plausible, wrong answers* rather than errors:
> transmit echoes decoded as motor replies (a flawless set of zeros from all seven motors) · `inertia` in a
> model signature making every joint look unique · `FeedbackFrameInfo` vs `MotorInfo` silently yielding `?`
> for every field · `error_code` annotated `int` but holding the string `'0x1'` · adapter-by-index quietly
> retargeting the other robot · two arms "verified" by evidence that could not distinguish them from one arm
> read twice. **None raised an exception.** Check values for plausibility, not merely for absence of errors,
> and prefer a test that could falsify the claim over one that merely agrees with it.

---

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

> ### ⭐ That recommendation was WRONG, and now it is measured rather than argued. 2026-08-10.
>
> `uv run scripts/bench_can.py --yes --cycle --samples 8000` — real 7-motor cycles against the real arm,
> sustained for 25 s:
>
> ```
> 8000/8000 cycles complete, 0 missed replies
> cycle latency (ms): mean 3.121  p50 3.116  p95 3.231  p99 3.318  p99.9 3.576  max 17.771
> sustained 320 cycles/s
> cycles that would miss a 100 Hz deadline (10 ms): 2/8000  (0.03%)
> ```
>
> **macOS clears the 100 Hz target with roughly 3× headroom.** The tail is what settles it — p99.9 is 3.58 ms
> against a 10 ms budget. The two 17 ms outliers are OS scheduling jitter, and the motors' own 400 ms firmware
> timeout makes a stall that size a non-event.
>
> **Consequence: teleop does not have to wait for the Linux machine.** Linux is still right for the final rig
> — the plan's whole stack (RealSense, cuRobo, ABC training) is Linux-first, and this measurement says nothing
> about the loop once IK, three cameras and policy inference compete for CPU. But *"macOS cannot do the control
> loop"* is refuted, not merely doubted.
>
> ⚠️ Honest caveat: measured with **register reads**, not MIT control frames. Both are one request plus one
> response, so the transport cost is identical, but firmware service time may differ. Treat it as a lower bound.

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

**Updated 2026-08-10 — this section is now history plus a new boundary.** Reasons 1 and 3 were answered: the
bitrate is documented at 1 Mbit/s and the protocol is I2RT's own (§6.1).

**What has now happened, with Julien's go-ahead and the arm clear:**

1. **Listen-only CAN** — transceiver electrically silent. 0 frames, as expected.
2. **Register reads** (`0x7FF`/`0x33`) — transmits, but cannot command motion. Identified the whole arm.
3. ⭐ **Motor 7 (gripper) enabled and immediately disabled.** First time a motor on this arm has ever been
   energised from this repo. **It did not move**: `pos=-0.0040 rad, vel=-0.0220, torque=-0.0073, err=normal`,
   `T_mos=33 °C / T_rot=30 °C`. Torque ≈ 0 confirms that enabling without a setpoint adds no force.

4. ⭐ **Motor 6 (`gripper_twist`) commanded, on both arms, together and separately.** ±86° ramped, and a
   bimanual sine at different amplitudes, speeds and directions. Peak torque measured at 0.14 Nm against a
   1.5 Nm abort limit. Both arms returned to their start positions and disabled cleanly every time.

**The boundary as it stands now: only motor 6 has ever been commanded.** `gripper_twist` changes the
gripper's orientation and never the arm's reach, so no commanded motion so far has been able to move any part
of either arm through space. **Motors 1-5 and 7 have been enabled and read, never commanded.**

⚠️ **The next boundary is the significant one: multi-joint motion.** Steps 3-4 of
[ROADMAP.md](docs/ROADMAP.md) move the *whole arm* through space for the first time — a shoulder or elbow can
reach things a gripper twist never could. That step needs a workspace check with Julien before it runs, not
just a script review.

⭐ **The safety timeout is ON and at the factory default** — the `timeout` register reads **8000** on every
motor, which is exactly what I2RT's `set_timeout.py` writes to *enable* it (it writes `0` to disable), and the
README documents the result as 400 ms. So the unit is 50 µs. A motor that stops receiving commands enters
damping mode by itself after 400 ms. **Do not "fix" this value.**

**The next boundary — actual motion.** Julien's instruction (2026-08-10): *move the gripper only, and only a
twist, so we can see how fast things move and whether anything moves at all, before turning on any other
motor.* That is the right instinct and is the plan. ⚠️ **But note before doing it:** `linear_4310.yml` has
`gripper_limits: null` and `needs_calibration: true`, so **the gripper's travel limits are not known**, and a
blind position command could drive it into a hard stop. Any first motion must be a small delta from the
*measured current position*, with low gains — not an absolute target.

## 6. What is genuinely unknown — the honest gaps

**Resolved on 2026-08-10** (kept visible, because "we already looked into that" is the cheapest thing to lose):

| Was a gap | Answer |
|---|---|
| ⭐ The YAM CAN protocol | **Published.** I2RT's SDK — §6.1 |
| Does the YAM SDK run on macOS? | **The driver layer does.** Not the `get_yam_robot()` layer — §2.1 |
| A YAM MJCF/URDF model | **Ships with the SDK**, `third_party/i2rt/i2rt/robot_models/arm/` — and `mink` is already an SDK dependency |
| Bitrate | **1 Mbit/s, documented**, no longer a guess |
| ⭐ **Is the arm powered and on the bus?** | **YES.** All 7 motors answered register reads; the gripper reported live state and `err=normal` |
| ⭐ **Which YAM variant and gripper** | **Determined over CAN, without energising anything** — see below |
| Is there a Linux machine? | **Not yet**, one is coming. Julien is on the MacBook for now — which is why §2.1 matters |

### 6.0 The arm, as measured

`uv run scripts/identify_arm.py --yes` reads each motor's `gear_ratio`, and the Damiao part number **is** the
gear ratio — DM43**40** reports 40.0, DM43**10** reports 10.0. `sw_ver` partitions identically as a cross-check.

| | motor | gear_ratio | sw_ver |
|---|---|---|---|
| joints 1-3 | **DM4340** | 40.0 | 925970741 |
| joints 4-6 | **DM4310** | 10.0 | 925970485 |
| gripper (id `0x07`) | **DM4310** | 10.0 | 925970485 |

That is `yam_v1.yml`'s layout exactly → **`yam` / `yam_pro` / `yam_ultra_v1`**, and **not** `yam_ultra_v2`
(which puts a DM4340 on joint 4). Those three share an identical motor layout and are **indistinguishable over
CAN** — separating them needs the physical label or a mass check (4.292 / 4.349 / 4.521 kg). Gripper is the
4310 family (`linear_4310` / `crank_4310` / `flexible_4310`).

Motor IDs are 1-7, master IDs 17-23 (`0x11`-`0x17`).

**Still open:**

| Gap | Why it matters |
|---|---|
| ⭐ **gs_usb throughput on macOS** | The question that decides whether the Mac is enough. Untested. 7 motors × 2 frames × 100 Hz is the target |
| **Which of `yam` / `yam_pro` / `yam_ultra_v1`** | Not resolvable over CAN. Affects the URDF/MJCF chosen for IK |
| **Gripper travel limits** | `gripper_limits: null`, `needs_calibration: true` — must be calibrated before any absolute position command |
| **Joints 1-6 live state** | Only the gripper has been polled so far |

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

✅ **Done 2026-08-10:** hardware re-verified · protocol found (§6.1) · macOS CAN path proven (§2.1) ·
listen-only probe green · **SpaceMouse verified on all six axes** · **arm identified over CAN** (§6.0) ·
**gripper polled live, healthy, did not move** (§5).

# ⭐ The ordered plan, with the reasoning for each step, is **[docs/ROADMAP.md](docs/ROADMAP.md)**.

Short form — the target is Julien's: *one arm, one SpaceMouse.*

1. **Teleop in simulation, end to end.** `get_yam_robot(sim=True)` exposes the *same API* as the hardware, so
   the whole SpaceMouse → IK → joint-targets chain is built and debugged at zero risk, then moved to hardware
   by changing one flag. First-attempt IK is always wrong; debugging it against a physical arm is how
   equipment gets damaged. **Needs no hardware.**
2. **Full-arm chain over gs_usb.** `DMChainCanInterface` hardcodes SocketCAN — the same wall as §2.1, one
   layer up. Unlocks all 7 motors at once, gravity compensation, and the gripper force limiter.
3. **Gravity compensation → hand-guiding.** Without it a 4.3 kg arm sags at gentle gains and every symptom
   reads as "the IK is wrong". Also the safest whole-arm test there is: it holds a pose, follows no trajectory.
4. ⭐ **SpaceMouse → the real arm.** With 1-3 done this is a flag plus a safety envelope, not new logic.
5. **Recorder → MCAP** in ABC's exact schema — get it right and the whole data→training half works unmodified.
6. **Second SpaceMouse → bimanual.** The hard half is already proven (`move_both_grippers.py`).
7. **Cameras.** Needed for data collection, not for teleop; nothing else depends on it.

*Why not joint-space jogging as a shortcut, and everything else deliberately excluded:
[ROADMAP §Deliberately NOT doing](docs/ROADMAP.md).*

> ### ⛔ Standing rule (Julien, 2026-08-10)
> **The agent never runs a command that physically moves the robot.** Those are handed over for him to run.
> The agent freely runs anything that cannot move it. The working line: **register reads and listen-only are
> the agent's; anything that enables a motor or sends a setpoint is Julien's.**

## 7.5 Contributing this back to `Hohnik/LaRobot` — the plan

Julien's ask (2026-08-10): this work should end up in his friend's repo,
**[github.com/Hohnik/LaRobot](https://github.com/Hohnik/LaRobot)** — *"only once everything is cleaned up and
clear… noted down and then done sensibly at the most sensible time."* So: **planned, deliberately not done yet.**

**What LaRobot is today** (checked 2026-08-10): **public**, 2 commits, `src/robot/` + `tests/`, Python +
`justfile`, README mostly `TODO`, requires **Ubuntu 24.04**, Hohnik the only contributor. It is an early-stage
skeleton — *"an environment for recording, simulating and training robot arms"*.

**Three things follow from that, and they shape the whole approach:**

1. ⛔ **Never push to `main` of someone else's repo.** Fork → feature branch → pull request. That holds even
   with write access: a PR is reviewable, a direct push is a fait accompli.
2. ⚠️ **It is public.** This repo has no secrets today, but that must be *verified at push time*, not assumed —
   and it is a standing reason never to put logs, camera frames or credentials here.
3. ⭐ **Its architecture is still his to define.** Two commits means Hohnik has not settled the structure.
   Dropping a parallel layout on him would be antisocial and would probably be rewritten anyway.
   **Ask him what shape he wants before opening the PR** — this is a social step, not a technical one.

**What is genuinely worth contributing** (roughly in descending value):

| Piece | Why it is worth having |
|---|---|
| `src/yam_can.py` | The macOS/gs_usb path **and** the transmit-echo bug. That bug bites *any* non-SocketCAN transport, so it is useful to him even on Ubuntu if he ever uses a candleLight adapter |
| `scripts/identify_arm.py` | Platform-agnostic, and **safer than I2RT's own `ping_motors.py`** — identifies the arm without energising a motor |
| `scripts/probe_can.py` | Listen-only bring-up probe; platform-agnostic |
| `src/spacemouse.py` + `spacemouse_live.py` | Teleop input, directly on LaRobot's stated path. The macOS seize behaviour is Mac-only, the decode is not |
| README §2.1 / §6.1 findings | The protocol pointer and the SocketCAN-assumption analysis |

⚠️ **Framing matters:** LaRobot targets Ubuntu, and `yam_can.py` is explicitly a macOS shim. It goes in as an
**optional platform layer**, never as the main path — otherwise it reads as "here is my OS's problem, now it
is yours."

**Sequence, in order:**

1. **Give this repo its own remote first** (still open — §4 of Mind Understanding's `NOW.md`). Julien's own
   private backup should not depend on a collaborator's repo.
2. Reach the "clean" bar below.
3. Ask Hohnik what structure he wants.
4. Fork `Hohnik/LaRobot` to Julien's account, add it as a second remote, push a feature branch, open a PR.
5. **Julien reviews the diff and says go.** Nothing is pushed to a third party's repo without that.

**The "cleaned up and clear" bar** — concretely, so it is not a matter of taste:

- [ ] The teleop chain works end to end, or at minimum: arm state reads live **and** SpaceMouse decodes.
- [ ] No dead code, no scratch scripts, no commented-out experiments.
- [ ] Every script's docstring states what it transmits, if anything.
- [ ] The safety boundary (§5) is accurate and nothing overstates what has been tested.
- [ ] Secrets check actually run against the diff, not assumed.
- [ ] The macOS-specific parts are clearly labelled as such.

## 8. Layout

```
yam-robotics/
├── README.md                  # this file: state, findings, next steps
├── docs/
│   ├── Setup-Plan.md          # the friend's 382-line bimanual YAM plan (copy; original in ~/Downloads)
│   ├── ROADMAP.md             # ⭐ the ordered plan and WHY each step comes where it does
│   ├── COMMANDS.md            # ⭐ every command, grouped by how much it can move
│   ├── HANDOFF.md             # ⭐⭐ START HERE with no context: state, rules, traps, next steps
│   └── FINDINGS.md            # ⭐⭐ the latent knowledge: every measurement, every trap, every rule
├── scripts/
│   ├── probe_hardware.py      # HID enumeration + open the SpaceMouse          read-only
│   ├── probe_can.py           # listen-only CAN watch                          read-only
│   ├── ping_motors.py         # ⚠️ enables motors (--yes); sends no setpoint
│   ├── identify_arm.py        # reads registers; identifies the arm without energising it
│   ├── bench_can.py           # CAN round-trip / control-rate measurement    read-only
│   ├── move_one_motor.py      # ⚠️ MOVES one motor on one arm (--yes)
│   └── move_both_grippers.py  # ⚠️ MOVES motor 6 on BOTH arms (--yes)
├── src/
│   ├── spacemouse_live.py     # live 6-DoF readout                             read-only
│   └── yam_can.py             # the macOS CAN layer (§2.1)
├── third_party/i2rt/          # vendored I2RT SDK — GITIGNORED, clone with the command in §6.1
└── pyproject.toml             # uv; hidapi · python-can · gs-usb · numpy · tyro · pydantic · crcmod-plus
```
