# Station verification — September 16, 2026

## Current status

Julien chose physical-station verification after the local cleanup. He and a
teammate are at the station. Both arms can be made available. **For the first live
checks, Julien wants the exact commands and explanation first, then runs them in
the station terminal himself.** Do not start a motor controller remotely in place
of that agreed handoff. Software checks and preparation continue independently.

Software checks and simulation on Linux have passed. Julien has run B and G and
reports that both arms felt normal. Their three single-arm logs end with exit 0
and all seven selected-arm motors confirmed disabled. G's large GUIDE pose change
was deliberate hand movement. Target-lead warnings remain documented below.

The combined-arm run has also completed with exit 0, both arms parked and all
14 motors confirmed disabled. Julien confirms that selection and movement worked
as intended, without unexpected movement or resistance, and asks to use his
selection/park workflow for subsequent checks. The integrated D405 recording and
operator-run replay have now passed, as detailed below. All five live logs end
with exit 0 and disabled motors. This completes the attended validation sequence;
it does not establish every mode, camera layout or training-data use as validated.

Use [STATION_COMMANDS](STATION_COMMANDS.md) for the connection/folder map and the
completed operator sequence. Validation slot 9 is occupied; the record helper
now refuses another run to preserve that take. No new motor check is needed for
this sequence. The same short helper works from either the Mac or the station.
The previous split SSH/`cd` instructions caused a real error when the temporary
station path was pasted at the Mac prompt. Do not repeat that workflow without
clearly identifying the host.

The run-specific `check` helper accepts B, G, both or record, with optional `--plan`.
It verifies candidate `025fab1`, unchanged application files and copied config
fingerprints, then logs an interactive HOLD-start session. The record profile
adds the D405 by serial and refuses if validation slot 9 already contains data.
The B/G/both plans, recording plan and invalid-input refusals were exercised
without motors. Current helper SHA-256 is
`c80f869dc5bdf0233f463220943d65cf881cc26eb90a1c6b5b2e78a482500408`.
The source is saved under the local evidence directory and copied to the remote
validation directory as `check`. The earlier `hold` helper remains available.
These temporary helpers are validation tools; they do not replace `./teleop` as
the repository launcher. `/tmp/yam-station-check` is installed on both machines;
it chooses local execution on RoVita or SSH on the Mac. Both entry paths passed
the actual record plan. Its source SHA-256 is
`4ee491e2f37efc545aeeb29f602254732dc7c326a12b7c9e3aa44d3216eda86c`.
Arguments are whitelisted before SSH; absent/invalid arguments refuse before
connection. Source copies are in the local evidence directory. The Mac entry
is a file; the station entry is a symlink into the validation directory.

Julien's verified normal exit is `q`, wait for the menu, then `q` to park both
arms and disable them. Keep the configured park paths clear. The alternative
`d` disables directly and requires support for every enabled arm. Ctrl-C or
fault handling can also initiate parking. These software keys do not replace a
physical power cut.
Codex can read the task terminal and saved station logs. There is no background
log monitor or automatic continuation.

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

Julien subsequently confirmed: "I moved it by hand; both arms felt normal."
This resolves the question about intended GUIDE movement and records his physical
assessment. It does not identify the numerical cause of the solver lead.
Source review found no demonstrated new cleanup regression from this run. Motion
code and operating limits stay unchanged while validating this candidate.

The 10,974-byte raw log is `logs/hold-B-20260916T140740Z.log` under the remote
validation directory and the local evidence directory below. Both copies have
SHA-256 `51f178ec2429b5bf890f2bd1b677b907806be03dcec907796e82cba09315e1ce`.
Local `after-B.json` records the read-only preservation check at 14:12:24 UTC.
Its operator application status is empty; its configuration and reference-checkout
comparisons are true. The operator's pre-existing copied park-pose difference
and vendor symlink remain expected differences from the committed tree.

## Attended G runs

Julien reported "Also works" and confirmed that the GUIDE pose change was his
deliberate hand movement. The app terminal and complete raw logs were reviewed.

| Observation | TELEOP run, 16:24:56–16:27:03 CEST | GUIDE run, 16:27:29–16:27:59 CEST |
| --- | --- | --- |
| Startup | G HOLD; normalized jaw 0.922 | G HOLD; normalized jaw 0.922 |
| Motion | TELEOP after about 9 s; displayed until t=109 s | GUIDE after about 3 s; displayed until t=13 s |
| Loop | 10,917 passes; 10.1 ms mean, 99 Hz | 1,308 passes; 10.1 ms mean, 99 Hz |
| Worst pass | 53.6 ms at t=8.9 s; one pass over 15, 20, 33 and 50 ms | 10.8 ms at t=9.7 s |
| Temperatures | Peak motor 36°C, gripper 30°C | Peak motor 35°C, gripper 31°C |
| Display diagnostics | 20 STUCK rows and two SLOWED rows | GUIDE change reached 1.702 rad, 97.5° |
| Exit | Quit menu, GUIDE, then all seven motors disabled | Automatic park, 0.020 rad reported error, all seven motors disabled |
| Result | Exit 0 | Exit 0 |

Both startups reported a +6.283 rad jaw-frame adjustment. This is the existing
runtime verification against saved jaw limits. No calibration or configuration
file was written. The GUIDE change includes Julien's hand movement and must not
be described as spontaneous sag. The first log's 154-column terminal clipped
parts of the warning values before logging, so full G target-lead values cannot
be recovered from this transcript. The warning mechanism is described above.
Neither run proves a prolonged stationary HOLD test or all possible GUIDE loads.

At 14:33:40 UTC, `after-G.json` again matched the reference checkout's HEAD,
status and config to the original snapshot. Operator config matched its copied
baseline, and application status was empty. Both local raw logs match the station:

- `hold-G-20260916T142456Z.log`: 21,854 bytes; SHA-256
  `a4c81571707df965c7239a737aa659af49802e75a9c4ef778a032a83c7761f33`.
- `hold-G-20260916T142729Z.log`: 7,702 bytes; SHA-256
  `abc0e00ecd3b7feae57dc990f7bcc45a719d5827e8082c54c73edf9fadd1002c`.

## Combined-arm run and Julien's workflow

The run at 16:41:26–16:42:25 CEST used `--arms B,G --start-mode hold --yes`.
The application found one puck, announced shared input and started both arms in
HOLD. Julien then exercised B GUIDE and TELEOP, switched B/G/BOTH selection, and
drove both arms while BOTH was selected. His follow-up was "both work" and asked
that the next steps fit this workflow. This was broader than the proposed
one-arm-at-a-time sequence; the record describes the actual run.
Asked explicitly whether B/G/BOTH selection moved exactly the intended arms
without unexpected movement or resistance, Julien answered: "Yes, selection and
movement worked as intended."

The loop reported 4,994 passes, a 10.2 ms mean (98 Hz), and a 54.4 ms worst pass at
t=3.8 s. Sixteen passes exceeded 15 ms, two exceeded 20 and 33 ms, and one exceeded
50 ms. Peak temperatures were B 35°C/jaw 31°C and G 34°C/jaw 30°C. Target-lead
warnings appeared, with some values clipped by the terminal width.

The quit menu's park-and-disable path parked G with 0.020 rad reported error and
B with 0.019 rad error. Both arms confirmed motors 1–7 disabled, and the command
exited 0. Axis maps reported unchanged. The 28,582-byte raw log is
`check-both-20260916T144126Z-553331.log`, SHA-256
`2ea6c875480d58896a1820efbd063a0136fd67d91129da48728c4789fe32f0c5`.

The repeated "already in TELEOP" messages are expected: selection changes do not
reset an arm's mode. With a shared puck, unselected arms receive centred input;
an explicit HOLD still uses `h`. The application's earlier generic TELEOP banner
claimed each arm followed its own puck, and its help described selection as
affecting only mode keys and edits. The local handoff version corrects those two
messages. The complete local suite remains 1,077/1,077 after this wording-only
change. The physical station continues to run the unchanged candidate application
at `025fab1`; the corrected messages have not been deployed there.

The read-only snapshot at 15:03:25 UTC, saved locally as `after-combined.json`,
matches the reference checkout's HEAD, status and configuration to the original
snapshot. The operator configuration still matches its copied baseline and its
application status is empty. Slot 9 and its frame directory are still absent.
The teammate's checkout has independently advanced to `087a4fc` and currently
shows a change to `scripts/physical.py`; its earlier inventory below is historical.
This validation did not modify or commit that checkout.

## Camera-only check and refreshed inventory

Current enumeration shows one SpaceMouse Compact, at HID path `5-1.4:1.0`, and
one D405, serial `260323072846`. Linux USB and V4L inventory do not list the
Logitech camera. The Logitech USB item that is present identifies as a mouse.
This supersedes the preparation count of two SpaceMice; no cause for the changed
count is established here.

Julien says the Logitech overhead camera is set up behind the arms and the G
camera is plugged in beside G, unmounted. Left/right arm roles remain unspecified.
The D405 must not be described as a calibrated wrist view or used to justify the
modelled camera control frame. The current tests retain world-frame control.

A bounded five-second `capture_probe.py --indices 6 --seconds 5 --hz 100 --save`
run used the freshly identified D405 colour node. It exited 0, acquiring no motor
handle. It wrote one frame and a report only in the disposable operator checkout.
The report records 1280×720 YUYV, 26.75 fps, 134 fresh frames in 496 samples,
307 duplicate samples, 55 empty samples, 33.32 ms mean and 33.55 ms worst frame gap.
Sampling faster than the camera produces duplicates; that count alone is not
a dropped-frame count. This short test does not establish sustained two-camera
delivery or performance while controlling and recording arms.

The saved probe frame was inspected. It points upward at the arm and window; it is not
a useful workspace view yet. Position and secure cameras before meaningful
demonstrations. The image proves colour delivery in this probe; no depth stream
was tested. At this checkpoint the exact recording-camera startup path still
needed its integrated test; the following section records that completed check.

Local evidence is under `camera-probe/` in the evidence directory. The remote
frame/report names begin `recordings/cameras/2026-09-16_163638_`; the log is
`logs/camera-probe-d405.txt`. No exposure-control command was issued.

## Integrated recording and replay — completed

Julien ran the prepared record profile at 17:10:03–17:12:58 CEST and reported
"worked perfectly seems like it." The complete log, saved files and replay report
were inspected. At startup the application measured the D405 at 1280×720,
30.0 fps in YUYV. The take records B in HOLD and G in TELEOP.
Julien subsequently replayed it, drove both arms and used in-session base parking.
These later actions were beyond the suggested short take and are recorded as
observed behavior, not commands sent by the agent.

| Evidence | Result and limit |
| --- | --- |
| Joint data | 1,166 finite, correctly sized samples; 14 joints ordered B then G; strictly increasing timestamps across 11.88037 s |
| Recorded sampling | 98.06 samples/s; median interval 10.06 ms, maximum 16.29 ms |
| Recorded motion | B's largest peak-to-peak change was 0.00039 rad; G's six arm joints changed by 0.388–0.541 rad; the modes agree with the data |
| Camera files | All 357 JPEGs decode at 1280×720; each file hash is distinct; disk filenames, index entries and saved counts agree |
| Camera completion | Index reports flushed; zero reported writer drops or write errors; sequence 671–1027 has no gaps |
| Host camera timing | 29.9985 fps; mean frame interval 33.335 ms, maximum 33.916 ms |
| Timeline alignment | All camera host timestamps lie within the joint timeline; largest distance to a nearest joint sample is 5.874 ms. This does not measure exposure timing or hardware synchronization |
| Publication | Stop and saved messages both report 11.9 s and 1,166 samples; JSON and frames are published under slot 9; no pending frames or interrupted-save directory remains |
| Replay | Finished at 1.00× in 11.879 s, zero reported cursor-hold time, then HOLD. Worst target-to-measured joint lag 0.11254 rad, below the configured 0.15 rad cursor-hold threshold |
| Replay timing | 949 tracking samples; rolling loop display approximately 80–85 Hz, last stored loop_hz 81.9. This differs from the recording rate and the whole-session mean |
| Whole session | 16,135 passes, mean 10.3 ms/97 Hz; worst 55.9 ms at t=9.4 s; 206 passes over 15 ms, 10 over 20, three over 33 and one over 50 |
| Exit | Both arms parked; reported final error B 0.020 rad and G 0.028 rad; all 14 motors confirmed disabled; exit 0 |
| Temperatures | Both arms reached 37°C; jaws B 29°C and G 28°C |

The first, middle and last camera frames were visually inspected. They show
changing arm position, but retain the dark, tilted upward view of the arm/window.
This verifies image delivery with control and recording. It is not a useful
workspace demonstration view. Mounting/framing, the absent Logitech connection,
physical left/right roles, depth and the team's training loader remain unverified.
Later TELEOP output includes workspace-edge and target-lead warnings; no fault
stop occurred. The existing measurement limits above still apply.

### Corrected stationary-ending diagnosis

The original recording checker flags 1.34626 s of trailing stillness as a definite
FINDINGS §30.1 defect and tells the operator to re-record. That conclusion is not
supported by this measurement. The take froze near displayed t=30 s and was saved
near t=64 s with the same 1,166 samples and 11.9 s duration: waiting at the save
prompt did not extend this recording. The user's intended length of the ending
was not independently established, so the data is retained without trimming.

The local checker now labels the column "still tail", reports a review warning,
and explains that a pause and unwanted extra samples can look identical. The
1.0 s review threshold, speed calculation and stored recording stay unchanged.
Both ordinary and park-containing recordings retain their context without an
automatic corruption diagnosis or instruction to discard/re-record.

Verification: 76/76 recording tests and 9/9 recovery tests pass. The revised real
CLI was also run on the station take from the disposable software-check checkout:
same duration, 1.35 s ending and 357 frames at 30.0 fps, with the corrected advice.
The motor-control operator checkout remains unchanged at `025fab1`.

### Preservation and evidence

The read-only snapshot at 15:14:56 UTC (`after-recording.json`) confirms reference
HEAD/status/config still match the before snapshot; operator config matches its
copied baseline and application status is empty. LaRobot remains separately on
`feature/physical` at `087a4fc` with `scripts/physical.py` edited.

- Live log: `check-record-20260916T151003Z-556213.log`, 78,065 bytes,
  SHA-256 `2168305b48861c64cec5020ebb5a056a0be5ec70ccef848c01c2a9932320ace7`.
- Trajectory: `recordings/9.json`, 222,675 bytes,
  SHA-256 `6b7c468383781ddf0a27f8e299d77686ea792941cf10447ebe783abec7b0b6c4`.
- Replay report: `recordings/tracking/9_2026-09-16T17-12-00+02-00.json`.
- Local `recording-validation.json` records all 12 data checks, frame timing,
  dimensions and hashes for every one of the take's 359 files (36,292,877 bytes).
  `recording-review/` holds the trajectory, index, replay report and three sampled
  images; copied take-file hashes match the station. It is not the full image set.
- The full take, tracking report, five live logs, copied config and source bundle
  were copied without replacing a checkout to the persistent station directory
  `/home/lavita/yam-validation-evidence/2026-09-16-025fab1-slot9`.
  All 380 copied files, 36,862,481 bytes, match their hashes. Its MANIFEST.json
  SHA-256 is `10980a29cd567fb7d3dc2450a1b480bd5f266cb443cd873300766768fa89bf54`.
  This survives `/tmp` cleanup but remains on the same station disk.

No complete image series was downloaded to the Mac. The original temporary take
and all existing project checkouts were retained. No motor command, calibration
change or team-remote push was performed by the agent.

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

### Preparation inventory differed from the August notes

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
under `/tmp/yam-validation-025fab1-LWvBIc/`. Transfers consist of the incremental
source bundle, small logs/reports/helpers, one camera-probe image and three take
images plus the small trajectory/index/replay files. No complete
recorded take, dependency environment or model collection was downloaded.

The attended B/G, combined-arm and integrated recording/replay sequence is complete.
No new command is required to finish it. The review package is updated locally.
The Logitech connection and meaningful camera framing still need physical attention
before useful demonstration capture; a remote file check cannot establish those.
Use device serials and B/G labels until physical left/right roles are established.
Publication or replacing a working station checkout needs a separate decision.
This bounded validation close-out is not a claim that the full station layout or
the team's application has been integrated and validated.
