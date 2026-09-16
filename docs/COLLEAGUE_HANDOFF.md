# Teleop cleanup: handoff for review

## Readiness

The cleaned reference teleop is ready for colleague review and supervised use of
the verified arm-control paths. Julien tested B and G and reports normal behavior.
The local cleanup is complete. Further structural refactoring is not a prerequisite
for this handoff.

Combined-arm operation with the current shared puck, a D405-backed take and its
replay have also been verified. The current attended validation sequence is
complete. Camera mounting/framing and the absent Logitech connection need physical
setup before collecting useful demonstrations. The connection/folder map and
tested workflow are in [STATION_COMMANDS](STATION_COMMANDS.md).
Publication, installation into a working station checkout and integration with
the team's application remain separate decisions.

## What this code is

The branch is `codex/teleop-cleanup`, based on Fable's final `499d0b7`.
The entry point is `apps/teleop_session.py`, using the `src/yam` library.
The motor-control application tested on Linux is `025fab1`; subsequent changes
repair a dataset falsifier, clarify shared-puck help text, correct a misleading
recording-checker diagnosis and document verification.
They do not change the tested motion code. The latest complete local suite also
passes after the help-text clarification.

The team's `src/robot` implementation in LaRobot has its own development history.
This handoff provides a reference implementation with explicit lifecycle,
recording and UI boundaries. It does not claim that those components are already
integrated into the team application or training loader.

The cleanup added reliable ownership of acquired devices, motor-first shutdown,
nonblocking recording completion, rollback on failed take publication, and
shared camera, input, status and mode-control services. Startup order and ordered
motion coordination remain explicit in the operator. Motion limits, calibration
values and stop policy were preserved. The operator is 1,973 lines.

## Verified evidence

| Check | Result and limit |
| --- | --- |
| Mac software | Latest full suite: 1,077/1,077 in 71 files; 71/71 falsifier catches at the fixture checkpoint. After the final diagnostic wording change: 76 recording and nine recovery tests pass, plus the actual checker on the station take |
| Linux software | 1,075/1,075 candidate checks, then two focused new regressions; 71/71 falsifier catches |
| Isolated simulator | 32/32 interactions; fake devices |
| B | HOLD, TELEOP, automatic park and seven disabled motors; exit 0 |
| G | Two runs covering HOLD, TELEOP, hand-guiding, direct disable from the quit menu and automatic park; both exit 0 |
| B and G together | Shared-puck selection B/G/BOTH, GUIDE/TELEOP changes and both arms parking; 14 motors disabled, exit 0; Julien reports that both work |
| Human observation | Julien says both arms felt normal, confirms the large GUIDE pose change was deliberate, and confirms combined selection moved exactly the intended arms without unexpected movement or resistance |
| Hardware timing | Single-arm means about 99 Hz; combined mean 98 Hz, worst observed pass 54.4 ms; no hard real-time guarantee |
| Camera only | D405 colour image received at 1280×720; five-second probe reported 26.75 fps and 33.32 ms mean inter-frame gap |
| Integrated recording | 11.88037 s; 1,166 samples of 14 joints at about 98 Hz; 357 readable 1280×720 images at 30 fps; index/count/timestamps agree, no writer drops/errors or interrupted save |
| Replay | Operator-run 1.00× replay finished into HOLD; worst joint lag 0.11254 rad, below the configured 0.15 rad pause threshold. Rolling replay loop display was about 80–85 Hz |

The take has 1.35 s of stationary ending. Its duration and sample count did not
grow while awaiting save. The checker now reports stillness as a review finding;
it cannot infer the old save-prompt defect or require re-recording from that alone.
The complete test take and logs are preserved with a verified manifest at
`/home/lavita/yam-validation-evidence/2026-09-16-025fab1-slot9` on the station.

The TELEOP runs contain target-lead warnings. These describe the gap between
the requested tool pose and the solver's model pose. They do not directly measure
contact or motor error. G's narrow terminal clipped some warning details.
The misleading checker conclusion was corrected without changing stored data or
motion code. No new blocking control defect was demonstrated. These observations
do not establish every mode, pose, payload or two-arm interaction as validated.

## How to review and run

Start with [README](../README.md) for the launcher and dependency setup, then
[RESTRUCTURING](RESTRUCTURING.md) for the implemented boundaries and retention
decisions. [CLEANUP](CLEANUP.md) contains the detailed implementation evidence.
[STATION_VALIDATION](STATION_VALIDATION.md) separates measured hardware results
from operator reports and remaining checks. [STATION_COMMANDS](STATION_COMMANDS.md)
also explains the retained modes when selection changes and the normal park exit.

In a prepared checkout, the simulated operator starts with:

```sh
./teleop --sim --arms B,G --start-mode hold --yes
```

The simulator uses stationary fake pucks. The full automated simulator driver
must use a disposable checkout because it writes recordings and configuration.
The source package excludes recorded data, Python environments and the vendored
I2RT checkout. Follow README to obtain the pinned dependencies. The original
training environment and saved training branch remain preserved separately.

## Station integration boundary

The existing `/home/lavita/yam-robotics` checkout remains at `499d0b7` with the
teammate's park-pose edits. `/home/lavita/LaRobot` has unfinished work on
`feature/physical`; it advanced independently to `087a4fc` during validation and
still has an edited `scripts/physical.py`. Neither was replaced or merged by this
validation.
The candidate is isolated under `/tmp/yam-validation-025fab1-LWvBIc/operator`.
That temporary location must be rechecked before reuse.

Physical validation used a copy of the station's current configuration, including
its uncommitted park-pose edits. The source archive contains the repository's
committed configuration. Preserve and compare the station configuration during
installation; do not overwrite its newer waypoints from the archive.

The current rig exposes one SpaceMouse and one D405. The Logitech camera described
by Julien is not enumerated by Linux. The D405 is beside G and unmounted; its view
must be arranged for demonstrations. Left/right arm roles are not yet confirmed.
Use the serial identities and world control frame until physical roles and mounts
are established. Existing calibration belongs to this rig and must not be applied
to another pair of arms without checking it.

No source or message has been sent to a colleague, and no remote branch has been
pushed by this continuation. A source archive can be reviewed independently; an
incremental Git bundle requires the original `499d0b7` history to import it.
