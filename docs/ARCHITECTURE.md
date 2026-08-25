# Architecture — how this system works

> **Who this is for: Julien, and anyone else who wants to understand the machine without reading the code.**
>
> It assumes you know nothing. Nothing about this project, nothing about robot arms, nothing about the code. Every word that needs explaining is explained here, the first time it appears.
>
> Reading all of it takes about twenty minutes. You can also stop after section 5 and still have the useful half.
>
> Two things are deliberately absent: the live state of the project, and the evidence behind every claim. An agent working in this repo reads [HANDOFF.md](HANDOFF.md) for the state and [FINDINGS.md](FINDINGS.md) for the evidence. Those two are written dense on purpose. This one is written to be read.
>
> **Contents**
>
> 1. What this system is, and what it is for
> 2. The words you need
> 3. The whole thing in one picture
> 4. What happens ninety times a second
> 5. The safety layer, and why every command passes through it last
> 6. Where the decisions live
> 7. How the software picks a device
> 8. From a movement to a training file
> 9. The five rules this project runs on
> 10. Where every piece of code lives
> 11. What is deliberately missing

## 1. What this system is, and what it is for

There are two robot arms on a desk. Each arm has seven motors in it. Six of them bend the arm. The seventh opens and closes the gripper. The gripper is a pair of fingers at the end of the arm.

The goal is to teach those arms a job by showing them. A person moves the arms through the job. The computer writes down exactly what happened, many times a second, and it saves what the cameras saw at the same time. That written-down job is called a demonstration. Collect enough demonstrations and a neural network can learn to do the job on its own.

This repo does the showing and the writing-down. It does not do the learning. Training the network happens elsewhere, in the team's own software. The last step here is to hand over a file in the shape that software expects.

So the whole system is one program that does four things.

1. It lets a person move the arms, either with a hand-held controller or by physically pushing the arm around.
2. While that happens, it writes down every joint angle and saves every camera picture.
3. It can play a saved movement back, so the arm repeats what it was taught.
4. It can convert a saved movement into the file format the training software reads.

Everything else below is detail about how those four things are made safe and made honest.

## 2. The words you need

These words appear everywhere below, so they are all defined here first.

| word | what it means |
|---|---|
| **arm B, arm G** | The two arms. Each is named after the serial number of the little box that talks to it, so the names never swap by accident. On the desk, G stands on the left and B stands on the right. |
| **joint** | One place where the arm bends. There are six. They are called base_yaw, shoulder_pitch, elbow_pitch, forearm_pitch, wrist_roll and gripper_twist. Each one is driven by one motor. |
| **jaws** | The gripper's two fingers. Motor number seven drives them. They slide instead of rotating, over a range of about 4.7 cm. |
| **joint angle** | How far one joint is currently bent, in radians. A radian is an angle unit. One radian is about 57 degrees. |
| **rad/s** | Radians per second, which measures how fast a joint is turning. One rad/s is a joint turning through about 57 degrees in a second. That is brisk but not alarming. |
| **the puck** | A SpaceMouse, which is a desk controller with a knob you can push, pull, twist and tilt. You steer the gripper with it. The program calls it the puck because that is what it looks like. |
| **teleop** | Short for teleoperation. It means driving the arm with the puck, so the gripper goes where you push. |
| **hand-guiding** | The other way to move the arm. The motors are told to hold the arm's own weight and nothing else, so the arm goes weightless and you push it around with your hands. The program calls this mode GUIDE. |
| **HOLD** | The mode where the arm is told to stay exactly where it is. |
| **park** | To drive the arm slowly and smoothly to a specific set of joint angles. Used to return to a known starting pose, and to line the arm up before a playback. |
| **waypoint** | A pose you saved earlier, so that later you can say "go there". There are ten slots for them. |
| **a recording** | One saved movement. It is a table of numbers with one row per pass of the program's loop, plus the camera pictures that go with it. |
| **CAN** | The wiring standard the motors speak. It is a two-wire bus, so all seven motors of one arm share one cable, and each motor answers to its own number. A small USB box converts between the computer and that bus. |
| **the loop** | The program's heartbeat. It aims to run about a hundred times a second. Each pass reads the inputs, decides what the arms should do, and sends the commands. |
| **an episode** | A recording that has been converted into a file format the training software reads. |

Three more names come from the team's own documents, and they are used here because the team uses them.

- ABC is the training software the demonstrations feed into. It belongs to the team. This repo only writes files for it.
- C3 and C4 are two sections of the team's setup guide. C3 describes a log-file format for one demonstration. C4 describes a training-directory format. This repo can write both.
- MCAP is the file format C3 is built on. It is a general-purpose format for logging timestamped streams of data.

## 3. The whole thing in one picture

```
  what a person does                                     what the cameras see
  ──────────────────                                     ────────────────────
  push the puck                                          1 webcam + up to 2 wrist cameras
  press a key                                            each has its own reader thread,
  (later: a trained network)                             each keeps only its newest picture
        │                                                          │
        ▼                                                          ▼
  ┌─────────────────── the loop, about 90 times a second ───────────────────┐
  │                                                                        │
  │  apps/teleop_session.py    reads the keys, prints the screen           │
  │                                                                        │
  │  ArmSession, one per arm    decides everything: which mode, where to   │
  │  (src/yam/session.py)       park, when to pause for a grab, when the   │
  │                             motors are too hot to continue             │
  └───────────┬─────────────────────────────────────────┬──────────────────┘
              │                                         │
              │ seven target angles per arm             │ pictures, only while recording
              ▼                                         ▼
     SafeRobot: the two limits                 one writer thread per camera
     (src/yam/robot.py)                        saves JPEG files
              │                                         │
              ▼                                         ▼
     the USB box, then the CAN bus            recordings/frames/<slot>/
              │
              ▼
     14 motors across 2 arms

     recordings/<slot>.json  ──┬──►  src/yam/episode.py   ──►  a C3 log file
     (the table of numbers)    │
                               └──►  src/yam/dataset.py   ──►  a C4 training directory
```

Read it left to right and top to bottom. A person pushes the puck. The loop turns that into target angles for each joint. The safety layer trims those targets if they are too aggressive. The trimmed targets go out over the CAN wire to the motors. Meanwhile the cameras are filling in their own pictures, and if a recording is running, both the numbers and the pictures get saved. Later, the saved file is converted into one of the two formats the training software reads.

One extra thing in that picture is worth pointing at, because it is easy to miss. Every device name is looked up in one single file, `src/yam/platform.py`. That is the only file in the whole repo that asks which operating system it is running on. On the Mac, the USB box is found one way and the cameras another way. On the Linux station, both are found differently again. Nothing else in the diagram knows or cares which machine it is on.

## 4. What happens ninety times a second

The loop aims for 100 passes a second and in practice manages between 83 and 90. The difference is real work. Reading cameras and compressing pictures happens inside the same program. That is fine, because every calculation below uses the measured time since the last pass instead of assuming a hundredth of a second.

Here is one pass, in order.

**Step 1. Read the puck.**

The puck reports six numbers. Three say how far it is pushed along each of three directions. Three say how far it is twisted around each of three axes. All six are between -1 and 1.

**Step 2. Turn that into a wish about the gripper.**

A full push means "move the gripper at 0.12 metres per second in that direction". A full twist means "rotate the gripper at 0.6 radians per second". Multiply by the time since the last pass and you get a small step. Add that step to where the gripper is now. You now have a wish: a position and an orientation in space where the gripper should be a moment from now.

**Step 3. Work out the joint angles that put the gripper there.**

This is the one mathematical step, and it is called inverse kinematics. The forward direction is easy: give me six joint angles and I can calculate exactly where the gripper ends up. The inverse direction is the hard one. You know where you want the gripper, and you need the six angles that get it there. No single formula does that, so a solver makes a guess and improves it until it is close enough.

The repo uses a library called mink for this, on top of a physics engine called MuJoCo which holds the arm's measurements. Both arms together cost about 0.1 milliseconds, out of roughly 10 milliseconds available. This step is nowhere near the bottleneck.

**Step 4. Hand the angles to the safety layer.**

Section 5 is entirely about this step.

**Step 5. Send them over the wire.**

The seven target angles per arm go out over CAN. Each motor runs its own small controller, which pushes toward the angle it was given.

**Step 6. If a recording is running, save this pass.**

Fourteen measured joint angles get added to the recording's table, seven for each arm. Separately, whatever picture each camera currently holds is handed to that camera's writer thread. The writer compresses and saves it without holding up the loop.

One detail of step 6 matters more than it looks. The numbers written into a recording are the angles the arm actually reached, read back from the motors. They are not the angles the program asked for. Those two are never quite the same. Writing down the wish would produce a dataset describing a robot that does not exist.

## 5. The safety layer, and why every command passes through it last

In August 2026 an arm snapped into a fast unwanted movement. Nobody had sent a bad command on purpose. The cause was a stale variable. The code still remembered where the arm had been minutes earlier. When teleop was re-entered, it aimed at that old pose in a single jump.

Julien's conclusion at the time shaped the whole design:

> "any type of safety would have to go lower than that."

The point is that no amount of care inside the driving code protects you from a bug in the driving code. The guard has to be somewhere the buggy code cannot bypass. So every command from every mode passes through one small object called `SafeRobot`, and it applies two limits.

**Limit one: how fast the command may change.**

The commanded angle may not move faster than 1.0 rad/s. At 90 passes a second that works out at about 0.011 radians per pass, or about 0.6 of a degree. So when something asks for a pose a whole radian away, it does not get a jump. It gets a ramp, one small step per pass, and there is plenty of time to press a key and stop it.

**Limit two: how far the command may run ahead of the arm.**

The commanded angle may never be more than 0.25 radians away from where the arm actually is, measured from the motors on that same pass.

This second limit deserves a worked example, because what it does is easy to guess wrong.

Say you ask the arm to move 1.0 radian. On the first pass the command may sit at most 0.25 rad ahead of the arm, so it moves 0.25 rad and stops there. The arm starts following. Once the arm has covered 0.1 rad, the command is allowed forward to 0.35 rad. The arm follows again, the command creeps forward again. The motion becomes a ratchet. The arm moves, the command advances, repeat. It arrives in the end, and no single step is ever large.

Now say the arm is blocked, because a jaw has closed on something solid or the arm is pressed against the table. The arm stops moving, so the command stops advancing, and it stays 0.25 rad past the arm. The push never grows. Bounding that push is the whole reason this limit exists.

That also explains why the limit is deliberately loose instead of tight. The motors push in proportion to the gap between command and reality. Squeeze that gap to nothing and you also squeeze away the force the arm needs to overcome its own friction, and the arm goes sluggish.

**Four limits, in series.**

The two above are the last two. There are four altogether, and the smallest one always decides. This matters when someone raises a limit and nothing changes: they raised the wrong one.

| limit | default | what it actually restricts |
|---|---|---|
| linear speed | 0.12 m/s | how fast a full push of the puck asks the gripper to travel through space |
| teleop speed | 1.5 rad/s | how far the inverse-kinematics answer may move any one joint in one pass |
| max speed | 1.0 rad/s | the same kind of cap again, applied inside `SafeRobot`, underneath all the driving logic |
| max lag | 0.25 rad | how far the command may run ahead of the measured arm |

**One refusal, on top of the four limits.**

Motors get hot. The guard warns at 55 °C and stops the session at 65 °C. It stops instead of warning and carrying on. The one place this project ever shipped a warning that carried on, a motor got cooked.

**What this layer cannot do.**

It cannot tell that a direction is wrong. Nothing at this level knows what the arm is supposed to be doing. What it does is bound how fast anything can go wrong, and that is what turns a dangerous mistake into a catchable one.

## 6. Where the decisions live

The program is split in two, and the split follows one rule. Anything that could be wrong goes in a class. The script only reads keys and prints.

The class is `ArmSession`, in `src/yam/session.py`, and there is one of them per arm. It holds every decision worth getting right:

- which mode the arm is in, and what happens when that changes
- how to build a smooth path to a park target
- the pause in the middle of a waypoint run where only the jaws move, and the check afterwards for whether anything was actually gripped
- the temperature guard
- the limit on how hard the jaws may squeeze, and the latch that gives up when they are clearly stuck

The script is `apps/teleop_session.py`. It reads the keyboard, calls methods on the class, and prints the screen.

The reason for the split is simple. A class with no hardware in it can be tested on a laptop with nothing plugged in, thousands of times, in a second. A script that talks to motors can only be tested by a person standing next to an arm. So all the risk goes in the class.

This rule was broken once, and the way it broke is a good illustration. The tested park code existed and was correct. The script also had its own older copy of park logic, and that copy was the one actually running. Nobody noticed, because the tests were green. They were testing code that nothing was using. Merging the two copies revealed two real bugs that had been live the whole time.

There is a second rule about the loop. It is narrower, and it has caught two real bugs. State only advances in the branch where something arrives or completes. A park handing over to a playback happens in the branch where the park arrives. The next leg of a multi-step run starts in the branch where the previous leg finishes. It never happens in a key-press branch, and it never happens on a timer.

On top of that, every park now records what it is for, so an arrival can only be credited to the thing it was actually for. Before that note existed, one waypoint's arrival got credited to a playback that had been set up in the same keystroke. The arm then played a recording starting 1.28 radians away from the correct pose.

## 7. How the software picks a device

USB devices get numbered in the order the computer happens to notice them. That order is not stable. On this rig it changed twice inside a single session.

So nothing here is ever selected by its position in a list. Every device is identified by something that belongs to it.

| device | how it is identified | why not by number |
|---|---|---|
| the USB boxes that talk to the arms | by serial number, one specific serial per arm | one bad reboot would swap the arms, and every command would go to the wrong one with nothing to show it |
| the wrist cameras (Intel D405) | by serial number | two D405s are physically identical, and nothing they photograph tells you which is which |
| the webcam (Logitech C920) | by model name | it reports an empty serial number, so keying it by serial cannot work |
| the pucks | by a wiggle gesture at startup | they also report empty serials, so the program asks you to move the one you want and assigns whichever moves |

Cameras have one more way to go wrong, and it produces a broken dataset with no error message anywhere. A D405 does not appear as one camera. On the Linux station it publishes six video devices, and the first one is the depth stream instead of the colour stream. Open the first one, save it as if it were a photograph, and you have a dataset of depth maps labelled as pictures. Nothing would complain. So the software reads which pixel formats each device offers, identifies the colour one from that, and refuses to guess when it cannot read the formats at all.

The same idea covers a remembered camera number. There is a small cache file mapping serials to device numbers, because resolving them from scratch takes a moment. A cached number is checked against the camera's model before it is trusted, every single time. That check caught something the first time it ran. Two stale entries were pointing both wrist cameras at the webcam.

## 8. From a movement to a training file

Here is a whole demonstration, from the command line to a training directory.

**1. Start the session.**

```bash
uv run apps/teleop_session.py --yes --arms B,G --cameras c920,d405:2603,d405:2553
```

The cameras open first, each one identified and checked as described in section 7. The pucks get assigned by wiggle. Then the motors are enabled and the loop starts. Nothing has moved yet.

**2. Start recording with `w`, and drive.**

Every pass adds fourteen measured joint angles to the recording's table. Every camera hands its newest picture to its own writer thread. You can drive with the puck (`t`), hand-guide the arm (`g`), or let a saved waypoint sequence run (`p 1 2 3` then Enter). The recording continues through all of that, and it records which modes it passed through. If a stretch goes badly you press `k`, and that span is marked inside the file as bad.

**3. Stop with `w`, then press a digit to save.**

The recording stops on the same line of code that stops the sampler, so no time accumulates while the save prompt waits for an answer. That bug existed once, and every file recorded before the fix has seconds of dead time at the end.

Pressing a digit writes `recordings/<slot>.json` and moves the pictures to `recordings/frames/<slot>/`. The file gets the git commit, the timestamp, the modes it passed through, and the per-camera picture counts stamped into it.

What that file contains is worth being concrete about. It is one table. One row per pass of the loop. Fourteen numbers per row, seven per arm, in the order the arms were named on the command line. Nobody invented that flat shape here. The training software already uses it.

**4. Check it.**

```bash
uv run checks/check_recordings.py
```

This re-counts everything from the files on disk instead of trusting what the recording says about itself. It reports the duration, any dead time at the end, the modes, and how many pictures each camera really has. Where the file's own claim and the disk disagree, it says so.

**5. Export it, in one of two shapes.**

```bash
uv run apps/export_episode.py --slot 3 --left G --right B --top c920 --left-wrist d405-260323072846 --right-wrist d405-255323071773
```

That writes the C3 log file. It holds eight streams of numbers plus one stream per camera. Every stream is stamped on a tick exactly 33,333,333 nanoseconds apart, so thirty ticks a second.

```bash
uv run apps/export_dataset.py --slot 3 --left G --right B --top c920 --left-wrist d405-260323072846 --right-wrist d405-255323071773
```

That writes the C4 training directory instead: a binary table of numbers, one video file, and a metadata file. The video is made by shrinking each camera's view to 224 by 224 pixels and stacking the views vertically into one tall frame.

Notice how many flags those two commands need. That is on purpose. Which arm physically stands on the left, and which camera looks from where, are facts about the room. The file cannot work them out from its own contents, and a wrong guess would mirror an entire dataset with nothing to show that it had happened. So the flags are required, and the export refuses to run without them.

Both exports are fed by the same function that turns the recording's rows into states and actions. That is deliberate. Two exports reading the same rows through two separate code paths could describe the same demonstration differently, and then only one of them would be right.

**6. Check that too.**

```bash
uv run checks/check_dataset.py
```

This re-opens the finished training directory and reads it back. It checks four things:

- the shape of the table
- whether every number in it is finite
- whether the actions follow the stated rule
- every property of the video the training loader depends on

The C4 video's encoding is strict, because the loader calculates where frame number k is in the file instead of decoding the video to find it. Encode it loosely and that calculation lands in the wrong place.

One verification cannot be done on this bench, and it is honest to name it. Only the team's own loader can say whether a C4 directory is truly acceptable to it.

## 9. The five rules this project runs on

Every design choice above comes from one of these five. They are worth reading even when nothing is broken, because they are the part that transfers to other projects.

**Rule 1. This stack fails by giving a confident wrong answer.**

Every defect in this project that mattered produced a plausible number and raised no error at all. Echoes of the computer's own transmissions read back as motor replies. A gravity model that was 39% short while the arm held 4.3 kg, with a calm screen throughout. A checker that stayed green while checking nothing. Three habits follow from that. Ask whether a value is plausible. Never settle for the absence of an exception. Prefer a test that could prove you wrong over one that agrees with you.

**Rule 2. Refuse, instead of warning and carrying on.**

A refusal costs somebody a retry. A warning costs whatever the hazard was, because warnings scroll past. The one warning-and-carrying-on this project ever shipped cooked a motor.

**Rule 3. A number written into a document goes stale, and nothing tells you.**

So a fact that changes is answered by a command, and documents name the command instead of copying its answer. This has been measured seven separate times in this repo. Every document that wrote a number down was wrong within days. That is why section 8 gives you commands to run instead of tables of results, and why you will not find a test count anywhere in this file.

**Rule 4. A checker should have something that deliberately breaks it.**

A checker that passes might be working, or it might have quietly stopped checking anything. You cannot tell which from a green run. So a checker gets a partner script that feeds it known-broken input and counts how many faults it catches. A green run plus a steady catch count is evidence. Three times here, a checker stopped detecting anything and kept passing, and nobody noticed. List `checks/falsify_*.py` to see which checkers have a partner. Some of them still have none.

**Rule 5. Time in front of the hardware is the scarcest thing here.**

Only Julien drives the arms, and every minute of that is a minute nobody else can spend. So anything provable without an arm gets proven without an arm first. There are unit tests, and above them a simulated arm that lags the way the measured hardware lags, which drives the entire real loop with nothing plugged in. That simulator caught a crash hundreds of unit tests had missed. The bench then gets only the questions the bench alone can answer.

## 10. Where every piece of code lives

| what it does | the code | what tests it |
|---|---|---|
| talk to the motors over CAN, build an arm | `src/yam/robot.py`, `src/yam/can.py` | `tests/test_motor_faults.py`, `checks/check_rig.py` |
| the two safety limits, the temperature guard, the jaw guard | `SafeRobot` in `src/yam/robot.py`, guards in `src/yam/session.py`, `src/yam/collision.py` | `tests/test_thermal_guard.py`, `tests/test_jaw_block.py`, `tests/test_collision.py` |
| every decision an arm makes: modes, parks, the grab pause | `src/yam/session.py`, `src/yam/motion.py` | `tests/test_arm_session.py`, `tests/test_jaw_pause.py`, `tests/test_motion.py` |
| the puck, the keyboard, the direction mappings | `src/yam/inputs/` | `tests/test_puck_assignment.py`, `tests/test_axis_map.py`, `tests/test_keyboard.py` |
| inverse kinematics, reference frames, the workspace bounds | `src/yam/teleop.py` | `tests/test_teleop_ik.py` |
| recording, playback, marking bad stretches, scrubbing | `src/yam/recording.py` | `tests/test_recording.py`, `tests/test_scrub.py`, `tests/test_save_slot.py` |
| cameras: reading, identifying, saving pictures | `src/yam/cameras/` | `tests/test_capture.py`, `tests/test_camera_identity.py`, `tests/test_frame_writer.py` |
| write a C3 log file | `src/yam/episode.py`, `apps/export_episode.py` | `tests/test_episode.py` |
| write a C4 training directory | `src/yam/dataset.py`, `apps/export_dataset.py` | `tests/test_dataset.py`, `checks/check_dataset.py` |
| work out which machine this is | `src/yam/platform.py` | `tests/test_platform.py`, `checks/check_platform.py` |
| open a camera and prove it delivers | `src/yam/cameras/open.py` | `tests/test_camera_open.py` |
| list files without picking up the operating system's litter | `src/yam/files.py` | `tests/test_files.py` |
| the loop itself, and the screen | `apps/teleop_session.py` | `checks/drive_sim_session.py`, which drives the whole loop simulated |
| the fake arm that lags like the real one | `src/yam/fake/arm.py` | `tests/test_fake_arm.py`, `checks/falsify_fake_arm.py` |
| keeping the checkers honest | `checks/check_*.py` and their `falsify_*.py` partners | `checks/run_falsifiers.py`, one command for one catch total |
| keeping the documents readable | `checks/check_prose.py` | `tests/test_prose.py`, `checks/falsify_check_prose.py` |

## 11. What is deliberately missing

Some absences here are decisions instead of gaps. They are listed so nobody has to discover them the hard way.

**There is no emergency stop button.**

Wall power is the only hard cut. That single fact shaped every motion feature in this repo: slow by default, bounded, interruptible, and sending nothing at all unless you pass `--yes`.

**Nothing checks whether the two arms are about to collide with each other.**

The operator is the guard, on purpose. At this bench spacing an automatic check raises false alarms constantly, which trains people to ignore it.

**Nothing here trains a network.**

That belongs to the team's software. This repo stops at handing over the file.

**Depth images are not recorded.**

The wrist cameras can measure depth. Getting it needs Intel's own library, and this repo reads them through the ordinary webcam protocol instead, which gives colour only. That has been measured on both machines.

The library itself turns out to be easy: one command, no administrator password, nothing compiled. It was tried on the station and it delivered depth pictures. Building it into the recording path is a real piece of work, and nothing needs depth yet. So the measurement is written down, and the work is waiting for a reason.

---

*Written 2026-08-19, then rewritten the same day for a reader with no prior context. When a shape described here changes, change this file in the same commit. A map that is out of date is worse than no map, because it still reads as authoritative.*

---

**Where to go next**

- [PLAN.md](PLAN.md) to build the real station from this one
- [BRIDGE.md](BRIDGE.md) to see how this repo maps onto the team's repo
- [PERFORMANCE.md](PERFORMANCE.md) for what is worth making faster, with the measurements
- [LAG.md](LAG.md) for why an arm trails the command it was given. That is a separate question from how fast the loop runs
- [COMMANDS.md](COMMANDS.md) when you want to actually drive the arms
- [FINDINGS.md](FINDINGS.md) when you want the evidence behind a claim here. It is written for agents and it is dense
