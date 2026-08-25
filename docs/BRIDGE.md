# Bridge — this repo, your repo, and what has to happen for the two to meet

> Who this is for: Julien and the team, on both sides of the two repositories.
>
> Reading all of it takes about ten minutes. It assumes nothing.
>
> Why it exists.
>
> There are two repositories. This one is a finished walkthrough of a working bimanual YAM station. `Hohnik/LaRobot` is the station you are actually building. Until now nothing said which part of one corresponds to which part of the other, so both sides could believe the other had already done something. That happened at least once, and section 5 is the case.
>
> ⚠️ Everything below was read from the repositories on 2026-08-20, the branch positions and section 4 item 5 in the evening of that day, after your camera commit landed.
>
> Branch positions move. Re-read them with the commands in section 1 rather than trusting this page.

## 1. The two repositories, and how to check their state yourself

| | this repo | your repo |
|---|---|---|
| what it is | a finished walkthrough: every feature built once, proven on the arms, written up | the station you are building |
| where | `~/Developer/Projects/yam-robotics` on Julien's Mac, `~/yam-robotics` on the station | `Hohnik/LaRobot`, cloned at `~/LaRobot` on the station |
| Python | about 20 modules under `src/yam/`. `uv run checks/run_tests.py` prints the live check count | 143 lines under `src/robot/` on `main`, plus work on two branches |
| what it is for | answering questions, so your build starts from answers | the real thing |

To check any of this yourself, on the station:

```bash
cd ~/LaRobot && git fetch && git branch -a --sort=-committerdate && wc -l src/robot/**/*.py
cd ~/yam-robotics && git log --oneline -1 && uv run checks/run_tests.py | tail -3
```

## 2. Where your repo actually stands

⭐ `main` is a skeleton, and knowing that is what makes [PLAN.md](PLAN.md) readable.

The work packages in that plan are not "fit this into a half-built system". They are "build this, and here is the answer to every question it raises".

Measured on `main` at `17ebfbe`. Only the README changed since `065a08e`, so every line count below is unchanged:

| file | lines | what that means |
|---|---|---|
| `src/robot/environment/simulation.py` | 106 | the only substantial code |
| `src/robot/inputs/keyboard.py` | 9 | a start |
| `src/robot/inputs/input.py` | 7 | one abstract method, `is_available()` |
| `src/robot/inputs/spacemouse.py` | 0 | empty file |
| `src/robot/inputs/mouse.py` | 0 | empty file |
| `src/robot/inputs/policy.py` | 0 | empty file |
| `src/robot/inputs/mcap_recording.py` | 0 | empty file |
| `src/robot/inverse_kinematics/__init__.py` | 0 | empty file |

**The two branches with work on them:**

| branch | position | what is on it |
|---|---|---|
| `feature/camera-framework` | **4 ahead of `main`, 1 behind. Your active branch** | `src/robot/cameras/` with `camera.py`, `frame.py`, `config.py`, `SimCamera.py`, plus `record.py` and `scripts/start_sim.py`. Tip `3576ce1`, and section 4 item 5 was read from it |
| `feature/spacemouse` | 1 ahead, 5 behind | one `SpaceMouseReader` class, tip `519b02c` |
| `julien/yam-teleop-wip` | 275 ahead, 9 behind | ✅ this walkthrough, current with it at `cb5c446` since the evening of 2026-08-20. See section 6 |

## 3. Which part of this repo answers which part of yours

The left column is your directory. The middle is where the answer already exists here. Work packages are [PLAN.md](PLAN.md) section 3.

| your file or directory | the answer in this repo | package | state |
|---|---|---|---|
| `src/robot/cameras/` | `src/yam/cameras/` — reading, identifying, writing, and opening a camera honestly | 6 | ⭐ **your active branch. Section 4 is written for it** |
| `src/robot/inputs/spacemouse.py` | `src/yam/inputs/spacemouse.py` — `TwistReader.read()` returns six numbers | 3 | working here, empty there |
| `src/robot/inputs/policy.py` | `src/yam/seams.py::CommandSource` — the interface a policy implements | 9 | ⭐ section 5 |
| `src/robot/inputs/mcap_recording.py` | `src/yam/recording.py` — recording, replay, labels, scrubbing | 5 | working here, empty there |
| `src/robot/inputs/keyboard.py` | `src/yam/inputs/keyboard.py` | 3 | both started |
| `src/robot/inverse_kinematics/` | `src/yam/teleop.py` — mink and MuJoCo, two arms in 0.1 ms | (C1) | working here, empty there |
| `src/robot/environment/simulation.py` | `src/yam/fake/arm.py` — an arm that lags the way the real one measurably does | 8 | both exist, different purposes |
| `src/robot/record.py` | `src/yam/cameras/writer.py` plus the recording half of `apps/teleop_session.py` | 5, 6 | on your camera branch |
| nothing yet | `src/yam/robot.py`, `src/yam/can.py` — CAN, motor faults, and `SafeRobot`'s two limits | 1, 2 | ⛔ **the safety layer has no counterpart on your side yet** |
| nothing yet | `src/yam/session.py` — modes, parks, the grab pause, the temperature guard | 4 | ⛔ the largest single piece |
| nothing yet | `src/yam/episode.py`, `src/yam/dataset.py` — the C3 log and the C4 training directory | 7 | ⛔ and section 7 is the one thing neither repo can settle |

## 4. Your camera framework, and four measurements that land on it this week

You are writing `src/robot/cameras/` now. On 2026-08-19 this repo found four camera problems on the same hardware you are building against. All four are in [FINDINGS.md](FINDINGS.md) section 76 with the measurements. Here they are against your code. Item 5 was added later the same week, after your commit `3576ce1`.

**Your `Camera` interface, as it stands:**

```python
class Camera(ABC):
    name: str
    @classmethod
    def is_available(cls) -> bool: ...
    def connect(self) -> None: ...
    def read(self) -> Frame: ...
    def close(self) -> None: ...
```

⭐ That shape is good, and until 2026-08-20 this repo had nothing like it. That is the honest state of things: you declared the interfaces and left the bodies empty, and this repo wrote the bodies and declared no interfaces at all. Two things are worth saying about your version before you finish it.

**1. Something has to sit between `read()` and your control loop.**

`read()` is a pull, so whoever calls it waits for the next picture. Called once per pass of a control loop, the loop then runs at the camera's frame rate, and with several cameras at the slowest one. [FINDINGS §21](FINDINGS.md) is the measurement from this repo getting it wrong: 6 pictures a second where 30 were expected, because a display loop waited on the camera.

⭐ The fix here is one reader thread per camera, keeping only the newest picture. The control loop then takes whatever is currently there and never waits. See `src/yam/cameras/grabber.py`, then `capture.py`. ⚠️ Your device interface is fine as it is. It is the layer above it that must not block.

**2. Your `Frame` has no timestamp, and the export needs one.**

```python
@dataclass(frozen=True, slots=True)
class Frame:
    camera_name: str
    rgb: NDArray[np.uint8]
    depth: NDArray[np.uint16] | None = None
```

Its docstring says *"a single camera frame that combined with a timestamp"*, so the intent is there. ⛔ The episode export joins pictures to control ticks by nearest timestamp ([FINDINGS §71.2](FINDINGS.md)). A picture with no timestamp cannot be joined to a robot state, and that is the whole point of an episode.

⭐ This repo's `Frame` has two more fields. `host_timestamp_ns` is set under the same lock as the picture it belongs to. `sequence` lets a consumer tell a repeated picture from a new one. Both are small, and everything downstream depends on them.

**3. Ask for MJPG before you ask for a size.**

A C920 at 1280x720 in uncompressed YUYV is limited to 10 pictures a second by the camera. In MJPG the same camera gives 29.92. YUYV is the format it advertises first, so it is chosen by default. macOS ignores the request and picks a good format by itself, and that is exactly why this went unnoticed until Linux ([FINDINGS §76.1](FINDINGS.md)).

**4. Read a real picture before you believe a size, and expect the room to change the rate.**

- A camera can accept a size and then deliver nothing at it. `cap.get` after `cap.set` returns what you asked for, never what arrives ([FINDINGS §76.0](FINDINGS.md), [§76.2](FINDINGS.md)).
- A C920 with V4L2's `exposure_dynamic_framerate` on gives 29.92 pictures a second in daylight and 14.98 in a dim room, while the driver reports 30 throughout ([FINDINGS §76.16](FINDINGS.md)). Two demonstrations of one task recorded morning and evening then contain twice as many pictures as each other, with nothing in the file to say why.

**5. Your newest commit, read on the evening of 2026-08-20.**

`feature/camera-framework` moved to `3576ce1` a few hours after the rest of this page was written. It adds `SimCamera` and a `Recorder` class. Four things about it, each one checkable in a few seconds.

- ⛔ `src/robot/cameras/SimCamera.py` does not compile. Line 15 reads `self._id = int | None = None`. Python treats that as one value assigned to two targets, and the second target is the expression `int | None`, so importing the file raises `SyntaxError: cannot assign to operator`. An annotation takes a colon: `self._id: int | None = None`.
- `SimCamera.available()` is spelled without the `is_`, so it does not implement the abstract `Camera.is_available()`. Instantiating the class raises `TypeError` even once line 15 is fixed.
- `connect()` calls `self.world.camera_id(self.name)` and nothing ever assigns `self.world`. The constructor sets `self.sim`.
- `tests/test_cameras/test_camera.py` is one comment line, so nothing imports `SimCamera` yet. That is why the three faults above are still in it.

⭐ To check the first one yourself, on any machine with Python:

```bash
python3 -m py_compile src/robot/cameras/SimCamera.py && echo "compiles"
```

⚠️ Points 1 and 2 of this section still apply word for word at `3576ce1`. Your `Frame` still has no timestamp field, and `read()` is still a pull.

## 5. The seam both sides thought the other had

⛔ [PLAN.md](PLAN.md) used to say of the command-source interface: "your `docs/ARCHITECTURE.mmd` already declares exactly this ... so this seam is agreed on both sides." That was wrong on both sides.

- Your `Input` ABC declares `is_available()` and says nothing about how a command is read.
- This repo had no declared interface at all until 2026-08-20, only a working `TwistReader.read()`.

So the agreement was real as a *design* and absent as an *interface*, and the plan told each side the other had finished it.

⭐ What the two halves are, and they fit together:

| half | who has it | what it is |
|---|---|---|
| discovery | you | `is_available() -> bool`, so a session can find what is plugged in |
| reading | this repo | `read() -> list[float]`, six numbers between -1 and 1: three for movement, three for rotation |

⭐⭐ And your directory layout already made the design decision that matters.

`inputs/policy.py` and `inputs/mcap_recording.py` sitting beside `inputs/spacemouse.py` says that a trained policy and a replayed recording are both just command sources. That is the right call, it is what makes Phase E small, and it is exactly what `src/yam/seams.py::CommandSource` describes. Everything below that interface, meaning every speed limit and every guard, then applies to a policy without being written twice.

⬜ The open decision, and it is yours to make together:

whether the read method is `read() -> list[float]` as here, or something else. Once it is agreed, a policy driving the arms is a small piece of work rather than a design question.

## 6. What has to move, and when

✅ `julien/yam-teleop-wip` is current with this repo again.

Julien gave the word on the evening of 2026-08-20 and the branch was fast-forwarded from `834c876` to `cb5c446`, 29 commits. It contains every camera measurement in section 4, the interfaces in section 5, and [PERFORMANCE.md](PERFORMANCE.md). No commit on the branch was rewritten, so an existing clone needs `git pull` and nothing else.

⬜ Pushing this repo to that branch needs Julien's word every time

([HANDOFF §4](HANDOFF.md) rule 9). It is his call, never an agent's, so the branch goes stale again between pushes. One command says how stale it is:

```bash
git fetch && git log --oneline -1 larobot/julien/yam-teleop-wip
```

How code reaches the station, for reference: a git bundle, never a push. The three commands are in [LINUX.md](LINUX.md) section 2.

## 7. Where this is going, and the one thing neither repo can settle

**What "done" means**

[PLAN.md](PLAN.md) section 1 defines it: both arms driven from one process, demonstrations collected as episodes your loader accepts unchanged, and a policy deployed through the same interface the teleop uses. Every safety property this walkthrough established still holding.

**What is missing, and where each gap is written down:**

| gap | where it is written | has an interface? |
|---|---|---|
| a policy driving the arms | [PLAN.md](PLAN.md) package 9 | ✅ `seams.py::CommandSource` |
| depth pictures | [FINDINGS §76.10](FINDINGS.md) | ✅ `seams.py::FrameSource`, and `Frame.depth` is the empty slot |
| pictures written by something other than the session | [PERFORMANCE.md](PERFORMANCE.md) section 5 | ✅ `seams.py::FrameConsumer` |
| collision distance against the real shapes | [ROADMAP §8.2](ROADMAP.md) item 35 | ⚠️ no interface. Keep `closest_approach(...) -> Closest` |
| per-joint speed ceilings | [FINDINGS §76.14](FINDINGS.md) | ⚠️ no interface needed. `TrackingLog` already measures it and mirror mode does not call it |
| how much to vary a replayed waypoint | [ROADMAP §8.2](ROADMAP.md) item 9 | ⛔ no code. It waits on a safety decision that is Julien's |
| ⭐ **an episode your loader accepts** | [PLAN.md](PLAN.md) package 7 | ⛔ **no interface can exist, and this is the real one** |

⛔ That last row is the one thing this bench cannot answer.

Both halves of the format are written here and both check their own output: the C3 log and the C4 training directory. What nobody here can test is whether your loader accepts them, because only your loader can answer that. Every episode this repo writes records `verified_against_abc_loader: false` so that nothing downstream can mistake "matches the published shape" for "loads".

⭐ So the shortest path to a trained policy runs through that one check, and it is a small piece of work for whoever owns the loader. Point it at `recordings/datasets/train/episode_slot5` and say what it says.

---

**Where to go next**

- [PLAN.md](PLAN.md) if you are building the station
- [ARCHITECTURE.md](ARCHITECTURE.md) if you want to understand this repo first
- [FINDINGS.md](FINDINGS.md) section 76 for the camera measurements behind section 4
- [PERFORMANCE.md](PERFORMANCE.md) for what is worth making faster

*Written 2026-08-20 from both repositories as they stood that day, and updated the same evening after commit `3576ce1` and the push in section 6. Branch positions move, so re-run the commands in section 1 rather than trusting the tables.*
