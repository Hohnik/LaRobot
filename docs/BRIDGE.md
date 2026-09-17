# Bridge to the current LaRobot implementation

Reviewed September 16, 2026. Start with [COLLEAGUE_HANDOFF](COLLEAGUE_HANDOFF.md)
for a short reading route and runnable examples.

This reference and LaRobot serve different roles. LaRobot is the team's own
implementation. This branch provides working feature implementations, tests and
station observations that the team can study and adapt.

## 1. Revisions actually reviewed

The station snapshot at 15:47 UTC showed a clean `/home/lavita/LaRobot` checkout
on `feature/physical`, commit `fd2c64b`. A subsequent remote query confirmed the
same pushed tip. All six current remote branch heads were fetched and inspected:

| Branch | Commit | Work present at that revision |
| --- | --- | --- |
| `main` | `f08f96f` | Simulation, SpaceMouse input, Cartesian target/IK and camera interfaces |
| `feature/dual-wield` | `2041c3d` | Two-device simulation scripts and a unified per-arm state example |
| `feature/physical` | `fd2c64b` | Path-based SpaceMouse selection and a single-arm I2RT control script; includes the dual-wield commits |
| `feature/recording-pipeline` | `62671fd` | Timestamped simulation samples and synchronous MCAP recording |
| `feature/training-pipeline` | `0c94c23` | MCAP-to-LeRobot conversion for a particular topic/encoding layout |
| `feature/abc-integration` | `211129a` | ABC dependency declarations and a policy-visualization launcher |

The ABC branch adds two files' worth of changes to `main`; it does not integrate
the newer physical or recording work. Recording and training also have separate
histories. A feature missing from the physical checkout may exist on another branch.

The [August comparison](archive/bridge-2026-08-20.md) is retained as history.
Its empty-interface descriptions and `SimCamera` syntax failure are superseded.
Every Python file in the six captured source trees compiled under Python 3.12.
That establishes syntax only; it does not establish device or dependency compatibility.

## 2. Useful boundaries on both sides

LaRobot already separates input, Cartesian target integration, inverse kinematics
and simulation. Its typed `Input` ABC and per-arm state in `start_sim_unified.py`
provide a useful shape for team-owned code.

This reference separates device cleanup, recording, playback, camera acquisition,
input routing, health checks and terminal interaction. The operator coordinates
those owners and the common timeline across arms.

| Feature | Reference to read | LaRobot counterpart or integration point |
| --- | --- | --- |
| Input and axis mapping | [inputs](../src/yam/inputs/), [SessionInput](../src/yam/session_input.py) | `inputs/input.py`, `inputs/spacemouse.py`; agree units before adapting |
| Cartesian commands | [CartesianTeleop](../src/yam/teleop.py) | `kinematics/cartesian.py`; preserve the team's separate target and IK objects |
| Commands and cleanup | [SafeRobot](../src/yam/robot.py), [SessionResources](../src/yam/session_resources.py) | `scripts/physical.py`; extract acquisition, command guards and cleanup before adding modes |
| Modes, jaws and parks | [ArmSession](../src/yam/session.py), [motion](../src/yam/motion.py) | Build around a per-arm state object and explicit transitions |
| Cameras | [camera session](../src/yam/cameras/session.py), [capture](../src/yam/cameras/capture.py) | `cameras/`; distinguish real-device readers from simulation rendering |
| Recording | [RecordingSession](../src/yam/recording_session.py), [store](../src/yam/recording_store.py) | Recording branch's `Recorder` and `Sample`; transfer lifecycle ideas while retaining a chosen file format |
| Replay and scrubbing | [PlaybackSession](../src/yam/playback_session.py), [recording](../src/yam/recording.py) | One cursor for the combined state, with measured arrival and lag gates |
| Export and learning | [episode](../src/yam/episode.py), [dataset](../src/yam/dataset.py) | Training converter and ABC branch; run a real reader against a real writer fixture |
| Simulation checks | [FakeArm](../src/yam/fake/arm.py) and application tests | Keep LaRobot's MuJoCo physics; use fakes for failure and sequence tests |

The reference fake has lag and blocking behavior for application tests. It has no
gravity or coupled dynamics. It cannot replace LaRobot's physics simulation.

## 3. Confirmed issues to resolve before integration

### Physical startup and cleanup

At `fd2c64b`, `scripts/physical.py` opens hardware at module level. If robot
construction fails, its `finally` block references an unassigned `robot` and replaces
the original failure. If gravity-compensation cleanup raises, `robot.close()` is
never attempted.

Both cases were reproduced by executing the script's actual `try` statement with
injected fake dependencies. No hardware imports or motor commands ran. Put startup
inside `main()`, register each acquired handle immediately, and attempt all cleanup
steps while retaining the original exception. Gravity compensation is an active
control mode; it is not equivalent to disabling motors.

The physical script has a Cartesian position-lead bound. It has no application
counterpart for the reference's all-mode measured-joint lag limit, thermal owner
or coordinated fault cleanup. The SDK may provide other behavior; this review
has not audited its newer pinned implementation.

### SpaceMouse callers after the constructor change

The physical and dual simulation scripts pass `device_path` and `side` correctly.
Four entry points still pass `device_index`: `start_sim.py`, `start_sim_unified.py`,
`visualize_spacemouse.py` and the SpaceMouse module's own demo.
Their arguments fail binding against the current constructor signature.
This was checked without opening HID devices. Update those callers together with
the API change and cover their startup paths.

### Recorder and training converter disagree

The recording branch writes `/robot/state_action` as JSON and `/cameras/<name>`
as NPZ arrays. The training branch's `src/training/dataset.py` reader expects
`/top-camera` compressed-image bytes and eight separate arm/end-effector topics
with binary array payloads.

An isolated probe used the actual recorder to write two synthetic samples and
four MCAP messages. The actual `read_mcap_episode` function returned zero frames.
The mismatch includes topics and payload encoding, so renaming topics alone is
insufficient. The default locations also differ: `recordings/last_session/episode.mcap`
versus a glob for `recordings/episodes/*.mcap`.

Agree one schema, write a tiny fixture and make the reader assert its expected
nonzero sample count, joint order, units and image content. Then test the resulting
LeRobot dataset and the ABC loader. Installing dependencies or starting policy
visualization alone cannot establish that path.

## 4. Data and scheduling decisions

The recording branch already records measured simulation state together with the
action about to be applied. Keep that distinction. The reference's live trajectory
contains measured joint positions; its exports infer next-state actions. Those
signals answer different training questions.

The team's MCAP recorder currently encodes and writes synchronously within the
simulation loop. Measure that cost before putting it in a hardware loop. The
reference's writer uses a bounded queue, reports dropped frames, and finishes
before publishing the take. Its JSON store is one implementation of that lifecycle.
The same lifecycle can support MCAP without adopting the reference directory format.

For real cameras, a reader can cache the newest frame independently of control.
For simulation, render against the appropriate simulation state on the thread
that owns the renderer. The team's `cameras/wip.md` proposes a simulation-owned
schedule. Background encoding must receive an owned image copy and its timestamp;
moving MuJoCo rendering to an arbitrary worker is not an equivalent change.

## 5. Interfaces that need explicit adapters

| Boundary | Reference contract | Team contract / required decision |
| --- | --- | --- |
| Puck output | Six normalized axes; buttons exposed as a bitmask | `Input.read()` returns physical velocities plus a button list; avoid scaling velocities twice |
| IK state | Seed from encoders at mode entry, then evolve an internal model | Team IK seeds each solve from current measured joints; target-lead diagnostics differ |
| Joint layout | B then G; six radians plus normalized jaw position per arm | Simulation is left then right; jaw position is metres. Confirm physical B/G roles and convert jaws explicitly |
| Images | Legacy `rgb` field contains OpenCV BGR pixels | MuJoCo rendering supplies RGB; convert channel order at a named boundary |
| Time | Monotonic host nanoseconds at camera-store time | Recording branch has simulation seconds; declare clock origin, units and capture meaning |
| Frame identity | Producer sequence plus host/device timestamp fields | Physical branch has no timestamp; recording branch adds `timestamp_s`. They are different interfaces |
| Robot SDK | Vendored I2RT `1276f63` | Physical branch declares `5b72c47`; verify shutdown, gripper and command APIs before copying adapters |

The reference's `CommandSource` protocol is a normalized twist input. A policy
that emits joint targets or action chunks needs its own explicit adapter and
scheduling rules. Merely implementing a method named `read` is insufficient.

## 6. A sensible implementation order

1. Repair physical startup/cleanup and obsolete input callers. Exercise success,
   partial construction and cleanup failure with fake handles.
2. Agree joint units, arm identity, timestamps and state/action meanings. Add one
   recorder-to-loader fixture before building more recording features.
3. Add a command boundary shared by every motion mode. Keep input scaling,
   IK behavior and measured command limits independently understandable.
4. Add HOLD, TELEOP and GUIDE transitions with measured-state resynchronization.
   Test that selecting another arm leaves existing modes explicit.
5. Add parks and jaw handling, then one recording owner and one replay cursor.
   Preserve measured arrival gates before replay starts or a grasp proceeds.
6. Add scrubbing, labels, composites, mirror and policy execution as separate
   reviewed features. Reuse the same guards and ownership rules.

The team's README prioritizes understanding each building block and small reviewed
commits. Follow that approach. Copy a component only after its units, owner,
failure behavior and tests make sense in the team's architecture.

## 7. Handoff and remaining boundaries

The source is ready to review locally. The cleaned application was exercised on
both arms, including combined control, camera-backed recording and replay.
[STATION_VALIDATION](STATION_VALIDATION.md) records the exact scope and limitations.

Deployment, calibration changes and team-branch integration are separate actions.
The test operator remains isolated from the team's checkout. Useful demonstrations
still require camera framing/mounting and a working overhead connection.
End-to-end compatibility with the team's current loader remains unverified beyond
the specific failing recorder/reader probe above.

Evidence is retained locally under `agents/codex/team-review-2026-09-16/`:
station `snapshot.json`, `compatibility-review.json`, `branch-review.json`,
byte-verified source copies and the reproducible `review_branches.py` probe.
During the September 16 comparison, no team source file was modified, no message
was sent, and no branch was pushed. Julien subsequently authorized the separate
public reference branch described in [COLLEAGUE_HANDOFF](COLLEAGUE_HANDOFF.md).
