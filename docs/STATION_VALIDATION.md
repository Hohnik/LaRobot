# Station verification — September 16, 2026

## Current status and the next operator action

Julien chose physical-station verification after the local cleanup. He and a
teammate are at the station. Both arms can be made available. **For the first live
checks, Julien wants the exact commands and explanation first, then runs them in
the station terminal himself.** Do not start a motor controller remotely in place
of that agreed handoff. Software checks and preparation continue independently.

Software checks and simulation on the Linux station have passed. Julien has now
run B from his Mac through SSH. He reports that control worked as expected. The
terminal and saved log confirm HOLD, TELEOP, parking and all seven motors disabled,
with exit code 0. Two late target-lead warnings remain to be explained below.
G and the combined station workflow have not been physically verified.

The next bounded check is G by itself. Use the existing remote shell, after the
teammate has yielded G and its workspace is clear:

```sh
cd /tmp/yam-validation-025fab1-LWvBIc
./hold G
```

This runs candidate `025fab1` with `--arm G --start-mode hold --yes`, using the
station's existing Python environment and a copy of its current configuration.
It enables G and starts HOLD. Follow any SpaceMouse assignment prompt, then leave
the puck centred. Observe for about 20 seconds: G should hold its pose, display
temperatures and joint positions, and report no fault or BLIND reading. This
command opens only G's motor chain.

If B met an obstacle or stopped following the intended motion, report that before
continuing to commanded motion. If B felt normal and G holds normally, press `t`
and try small, slow movements within clear space. Release the puck between them,
then press `h` and check that G holds the current pose. A persistent STUCK warning,
unexpected motion, resistance or a fault ends this check for review. Do not push
through resistance or change speed, calibration or limits to make a warning vanish.
GUIDE, mirror, saved waypoints and recording are separate later checks.

**Normal exit for this test:** press `q`, wait for the menu, support the arm so it
cannot fall when torque is removed, then press `d`. Confirm that the terminal
reports motors 1–7 disabled. A second `q` requests a park; Ctrl-C can also request
parking. Fault handling may attempt the configured park before disable, so keep
that path clear and the physical power cut available. Software HOLD/quit keys are
not a hardware emergency stop. These are existing controller behaviors, not new
shutdown guarantees introduced by this test.

The helper captures a terminal log under this run's `logs/hold-G-*.log`; Codex can
read it over SSH. Julien must still report what the physical arm did: a log cannot
prove absence of unexpected motion, contact, sound or heat. If startup refuses or
a fault appears, stop the sequence and inspect that evidence before trying again.
Further commanded motion needs another explained, operator-run step.

`./hold B --plan` exercises the same helper without `--yes` and opens no devices.
That wrapper dry run and both arm plans passed during preparation. Do not copy
these /tmp paths as permanent installation instructions; verify their existence
and candidate before resuming.

To reconnect from the Mac, without starting a controller:

```sh
ssh -t -o ControlPath=none yam-pc 'cd /tmp/yam-validation-025fab1-LWvBIc && exec bash --noprofile --norc -i'
```

Codex can read this task's terminal snapshot and the saved station log. Its private
tool SSH session was closed; it is separate from Julien's interactive shell.
There is no background log monitor or automatic continuation.

## First attended run: B

Julien ran `./hold B` on September 16, 16:07:40–16:08:23 CEST. His report was:
"It seems to work as it should, I was able to control."
The app terminal shows the SSH command, operator output and a returned shell prompt.
The station log independently ends with `COMMAND_EXIT_CODE="0"`.

| Observation | Recorded result and scope |
| --- | --- |
| Candidate | `025fab1cadbb8bbed962b7727866d79f885d8743`; application source remains unchanged |
| Startup | Seven B motors enabled; existing jaw limits verified, normalized jaw position 0.999; no calibration requested |
| HOLD | About seven seconds of unchanged displayed joint positions; this was shorter than the proposed 20-second hold |
| TELEOP | Entered around t=7 s and continued until t=35 s; measured joint values changed; Julien reports successful control |
| Temperature | Highest reported motor temperature 35°C; gripper 31°C; no BLIND temperature reading |
| Timing | 3,550 passes, mean 10.1 ms (99 Hz); worst 45.6 ms at t=7.8 s; one pass over each of 15, 20 and 33 ms |
| Warning | `STUCK lead 1cm/14°` at t=34 and t=35 s; no fault stop |
| Exit | Operator-requested quit followed the park-and-disable path; park reported 0.019 rad remaining error; motors 1–7 confirmed disabled |
| Persistence | Axis map reported unchanged; all operator configuration hashes match the pre-run copy |
| Reference checkout | HEAD, porcelain status and every configuration hash still match the before-preparation snapshot |

The timing outlier occurred near TELEOP entry. This log does not attribute its
cause or establish a worst-case latency bound. The motor-disable line is the
controller's confirmation; the physical observation remains Julien's report.
This run did not exercise direct `d` disable, GUIDE, G, simultaneous arm control,
gripper open/close, cameras, recording or replay. G's behavior was not separately
reported, although the command selected B only.

### Meaning of the two warnings

The display warns when translation or rotation target lead exceeds 80% of its
configured bound. Here 14 degrees exceeds the angular threshold of 0.20 rad
(about 11.5 degrees). The displayed 1 cm is below the translation threshold of 4 cm.

The quantities come from `CartesianTeleop.lead()`: the integrated tool target
minus the solver's internal forward-kinematics pose. The solver is seeded from
measured joints on mode entry and then integrates its own state. This warning is
not a direct measurement of motor tracking error, contact or collision. The
separate status `q` values are read from the robot. A pose limitation, competing
IK objectives or requested rotation could explain the solver gap; this log cannot
select a cause. Do not claim the arm was physically blocked or that the warning
is harmless solely from this output.

An asynchronous question asks Julien whether he noticed contact, resistance or
failure to follow near the end. His general success report is already recorded;
the more specific answer is pending. Source review found no demonstrated new
cleanup regression from this run. Motion code and operating limits stay unchanged
while validating this candidate.

The 10,974-byte raw log is `logs/hold-B-20260916T140740Z.log` under the remote
validation directory and the local evidence directory below. Both copies have
SHA-256 `51f178ec2429b5bf890f2bd1b677b907806be03dcec907796e82cba09315e1ce`.
Local `after-B.json` records the read-only preservation check at 14:12:24 UTC.
Its operator application status is empty; its configuration and reference-checkout
comparisons are true. The operator's pre-existing copied park-pose difference
and vendor symlink remain expected differences from the committed tree.

## What was verified remotely

- SSH alias `yam-pc` is configured locally and works; no new credentials were
  needed. It resolves to the RoVita Linux station as user lavita.
- Existing reference checkout: `/home/lavita/yam-robotics`, branch main at
  `499d0b76faebadd413860a85e0193e2c1524ddb6`, with an uncommitted
  config/park_pose.json change. B waypoint 6 changed and 7–9 were added; the
  existing base pose was not changed in that diff.
- The team's `/home/lavita/LaRobot` is on feature/physical with unfinished changes
  to pyproject.toml, scripts/physical.py, scripts/start_sim_dual.py,
  src/robot/inputs/spacemouse.py, and untracked tests/arm_test.py. Those files were
  not edited or committed by this verification. A process scan showed editors and
  uv run viz_policy.py; a process-name scan cannot establish exclusive arm access.
- Python 3.12.3 is already available in the reference checkout's .venv. Its lockfile
  SHA-256 matches the Mac's: 3b745c875b4511c04650c0b727f8584b0184eb341562b98a04492bc716fa36a1.
  Vendor I2RT is 1276f63d640eb45c226efd3dc08430b810372e94. No install or upgrade was run.
- Candidate imported into `/tmp/yam-validation-025fab1-LWvBIc/checkout` via a
  334,177-byte incremental bundle requiring 499d0b7. Bundle SHA-256:
  b45accc6d481e8704631d6792eeaad7ecd90cfe3075eabbc39ad25e48d5d75ea.
  Import resolves to 025fab1cadbb8bbed962b7727866d79f885d8743. There was no push,
  merge or switch of either existing working checkout.
- A separate `operator` clone contains that same committed application code plus
  the station's current config directory. It has no original recordings. Both
  clones use the existing vendor directory; test commands use the existing Python
  with PYTHONPATH set explicitly to the temporary clone's src.
- After preparation, the reference checkout's HEAD, porcelain status and every
  configuration-file SHA-256 exactly matched the before snapshot. The original
  recordings were not a test destination. The simulator used only its disposable
  checkout's recordings/config.

### Software evidence

| Check | Station result | Scope |
| --- | --- | --- |
| Full candidate suite | 1,075/1,075, 70 files | Candidate application/library tests with fake devices |
| Falsifiers, first attempt | Failure: dataset fixture missing | Fresh clone contained no ignored user export |
| Falsifiers, corrected | 71/71, five scripts | Same five dataset mutations, now against temporary synthetic data |
| New fixture regression | 2/2 | Fresh-copy default succeeds; explicit bad source refuses without mutation |
| Full interactive simulator | 32/32 | Disposable copy, no physical CAN/HID/camera operation |
| Platform and USB inventory | Both exit 0 | Enumeration/configuration only; no motor health proof |
| B and G HOLD plans | Both exit 0 | Dry runs using copied station configuration |
| Logged wrapper plan | Exit 0, DRY RUN present | Actual helper/PTY invocation without device acquisition |

The corrected falsifier and its regression were copied as validation-only changes
into the disposable check checkout. The operator clone still runs the unchanged
025fab1 motor-control code. Do not imply the first falsifier attempt passed or that
the entire station suite was rerun after adding the two regression cases.

### Current inventory differs from the August notes

USB serial resolution currently maps B (2081337C594E5018) to can0 and G
(20593383594E5018) to can1. Both interfaces are up at 1,000,000 bit/s. This is the
reverse of the old interface-name table; use the serial resolver, not that table.
Two SpaceMouse Compact devices and one RealSense D405 (260323072846) are present.
The D405 colour stream is video6 in this snapshot. A C920 and a second D405 were
not present in the inventory. The runtime has no pyrealsense2; the reported D405
capability is colour, not verified depth. No camera capture was started.

The fresh empty-loop benchmark reports 99.37 Hz, mean 10.063 ms, worst 10.354 ms;
sleep overshoot medians are 0.062 ms. Mirror step reports 9.39 microseconds with
state following. This is a no-motor benchmark under the station's current load,
not real control-loop latency, arm response or a reason to raise a speed constant.
The old 97 Hz station / 84 Hz Mac measurements remain dated historical evidence.

## Fresh-checkout falsifier repair

The original falsifier searched ignored recordings/datasets and required a valid
user export. The older recursive-search fix handled split directories, but could
not make an empty clone work. That dependency caused the station's initial 66/66
partial total plus one failed script; it was not a physical-control failure.

The default now creates a temporary, nonconstant 90-row synthetic joint table and
an actual encoded video spanning three GOPs, verifies that baseline, then applies
the same five mutations to disposable copies. Its ffmpeg/ffprobe dependency stays
explicit. `--source <episode_dir>` retains the option to test an existing export;
that input is verified and copied, never modified. A failed baseline reports
CATCHES: 0/5. No user data is exported to satisfy the test, and this does not prove
that the team's training loader accepts either export format.

Two regression tests exercise a fresh copied checker directory without recordings,
and an invalid explicit source whose bytes and modification time remain unchanged.
The fix changes verification tooling only. Current motor code, calibration and
limits are unchanged. The final Mac suite passes 1,077/1,077 checks across 71 files;
local falsifiers catch 71/71, and command-flag, link and prose checks pass.

## Information only the people at the station can supply

1. Whether the teammate has yielded the hardware and no other controller owns it.
2. What each arm actually does during startup, HOLD and disable; logs alone cannot
   establish this.
3. For later camera/recording tests: the actual left/right arm and top/wrist camera
   mounting roles, plus the intended demonstration. USB serials identify devices,
   not their physical role.
4. Whether/where to promote or publish the tested candidate. Testing in a temporary
   checkout is not permission to overwrite the station's unfinished work or push.

SSH address, existing paths, interpreters, code revisions, inventory and logs were
retrieved directly. Do not ask Julien to gather them again unless something changes
or access fails. Do not send teammate messages on his behalf without instruction.

## Evidence locations and resumption

Local copied logs and the bundle are under
`agents/codex/station-validation-2026-09-16/` (ignored). Remote logs, results.json,
station-before.json, station-after-preparation.json and operator-config.json are
under `/tmp/yam-validation-025fab1-LWvBIc/`. Only the bundle and small text logs were
transferred; no recordings, dependencies or models were downloaded.

Resume with the pending observation about B's late warning and the explained G
check above. The next dependency is Julien running that check and describing its
physical behavior. After reviewing G, choose the smallest relevant combined-arm
or recording check; camera roles must be supplied before naming views in a take.
Those workflows remain unverified. Publication or replacing a working station
checkout needs a separate decision. The completed local cleanup is separate from
this still-open physical phase.
