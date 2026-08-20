# Performance — what is actually slow, and what would change if you altered it

> Who this is for: Julien and the team.
>
> It assumes you have read [ARCHITECTURE.md](ARCHITECTURE.md), or at least its section 2, which defines every word used here.
>
> Reading all of it takes about fifteen minutes.
>
> Why it exists.
>
> Julien asked on 2026-08-20: *"does the 100Hz robot loop really have to be the same loop used for the camera reading and compression, if we only manage less than 90 Hz? How could this be done differently to drastically improve performance?"* The short answer surprised me, so the whole question is written out here with the numbers.
>
> ⛔ Every number below was measured on this hardware.
>
> Where something is an estimate it says so. Nothing here is a guess dressed as a fact, because that mistake is what [FINDINGS.md](FINDINGS.md) section 76 is about.

## 1. The short answers

If you read nothing else:

1. ⭐ The loop misses 100 passes a second because of the way it waits. No work it does is responsible. Measured on 2026-08-20: a loop with no arms, no cameras and no work at all, using this loop's own waiting line, runs at 84.3 passes a second on the Mac. The same loop runs at 97.3 on the Linux station. Section 2 has the numbers. So the shortfall everyone has been trying to explain is mostly macOS, and moving work out of the loop cannot recover it.
2. The loop is not slow because of the cameras. It was measured at about 87 passes a second before any camera code existed. The cameras cost a few passes a second on top of that, and the exact amount has never been isolated.
3. The camera work is tiny. Compressing one 1280x720 picture takes 1.4 milliseconds. Three cameras at 30 pictures a second therefore need about 0.13 seconds of compression work per second of recording. That is not a load worth restructuring for.
4. Reading and compressing already happen off the loop. Each camera has its own reader thread and its own writer thread. The loop hands over a reference and moves on.
5. 83 passes a second is already about three times faster than the arm can respond to. The arm follows a moving target with a delay of about 0.033 seconds, so it behaves like something with a 30 Hz limit. Sending it setpoints at 83 Hz is not the constraint on anything.
6. Two changes would actually move the numbers for the pictures, and both are data decisions rather than code: the compression quality, and the size you capture at. Both change the recorded data, so both are Julien's decision.

## 2. Where the loop's time actually goes

**What is measured.**

The loop asks for 100 passes a second. On the Mac it has never achieved that. [FINDINGS.md](FINDINGS.md) section 31.1 measured about 87. It was found because a playback summary did not add up: 4.0 seconds of work reported inside 4.6 seconds of wall clock. 4.0 ÷ 4.6 = 0.87.

**⭐⭐ And on 2026-08-20 the cause turned out to be the waiting, not the work.**

The bottom of the loop is one line: `time.sleep(max(0.0, dt - elapsed))`. It asks the operating system to wake it up in the time left over from a 10 ms pass. An operating system may wake you later than you asked, and by how much is a property of the operating system rather than of this program.

So it was measured on both machines, twice. How late does `time.sleep` return? And what does an empty loop achieve, using exactly this loop's own waiting line? No arms, no cameras, no inverse kinematics, no status line. Nothing but the wait.

| | Mac (macOS, where the walkthrough was built) | Linux station (RoVita, Ubuntu 24.04) |
|---|---|---|
| `time.sleep(0.008)` returns late by | 1.88 ms (median), 2.08 ms (worst of 300) | 0.366 ms (median), 0.615 ms (worst of 300) |
| an empty 100 Hz loop achieves | 11.87 ms a pass, so **84.3 Hz** | 10.28 ms a pass, so **97.3 Hz** |
| the real session, with two simulated arms | 11.8 ms a pass, so 84 to 85 Hz | not measured yet |

⛔ Read the third row against the second. On the Mac, a full session with two arms, a recording, a playback and a composite run is no slower than a loop that does nothing at all. Both come out at about 84 Hz.

⛔ Every millisecond of the Mac's shortfall is the wait. None of it is the work.

⭐ Three things follow, and the third one matters most for the station.

- Moving the cameras, or anything else, out of the loop cannot recover the missing 15 Hz on the Mac. There is no work there to move.
- The 87 Hz figure that appears throughout this repo is a macOS number. It was never a property of the system, and reading it as one is the same mistake as reading a camera's usable sizes off one platform ([FINDINGS §76.2](FINDINGS.md)).
- The station's own loop, with the real arms, has still not been measured, and it now has a plausible ceiling near 97 Hz rather than 87. It measures itself from now on: every session prints its own numbers when it stops.

**What is still not attributed.**

Nothing has broken a *station* pass down into its parts. The candidates, with what is now known:

| candidate | what is known |
|---|---|
| ⭐ the wait at the bottom of the loop | **measured, and it is the whole of the Mac's shortfall**: 1.88 ms a pass on macOS, 0.366 ms on the station |
| the CAN round trip | 14 motors commanded and read every pass over two USB adapters. `apps/bench_can.py` measured a 7-motor cycle at 3.12 ms mean and 17.8 ms worst on the Mac ([HISTORY.md](HISTORY.md)). ⚠️ That is absorbed by the wait unless a pass exceeds 10 ms, which is why removing it in simulation changed nothing |
| Python work per pass | mode logic, the status line, the tracking log. Unmeasured, and bounded above by the empty-loop result: on the Mac it costs less than the wait |
| inverse kinematics | **measured: about 0.1 ms for both arms** while running, against roughly 10 ms available. ⚠️ Its **construction** is another matter, and it is the biggest stall this loop has: see the worst-pass numbers below |
| camera hand-off | reading the newest frame reference and putting it on a queue. Microseconds |

⛔ Before anyone changes the design to make it faster, read the two rows above that say "measured". The one lever this section has found is the wait, and on the station it is already small.

⭐ The instrument exists, and as of 2026-08-20 it records the worst pass too.

Every playback writes a tracking file that records `loop_hz`, so the average rate is captured per run rather than noticed once. What it did not record was the worst single pass, and the average cannot show one. `src/yam/timing.py` now measures it: every session prints one line when it stops, every playback's tracking file contains the numbers, and a crash report contains them as well.

It keeps the five slowest passes, with the moment each one happened. Five rather than one, because one time is not a pattern. Finding the cause of the first worst pass ever measured meant putting a temporary `print` into the loop and reading the log for what had been printed just before it.

**The first measurements, and read the warning under the table before using them.**

Three runs of `checks/drive_sim_session.py` on the Mac. That driver runs a full session: two simulated arms, a recording, a playback and a composite run.

| run | passes | mean | worst pass | over 33 ms | over 50 ms |
|---|---|---|---|---|---|
| 1 | 7143 | 11.8 ms (85 Hz) | 75.4 ms | 4 | 1 |
| 2 | 7085 | 11.8 ms (85 Hz) | 67.8 ms | 3 | 1 |
| 3 | 7010 | 11.8 ms (84 Hz) | 71.2 ms | 4 | 1 |

⛔ These are simulated arms. A simulated arm answers in microseconds. A real one is 14 motors over two USB adapters, and that is the largest single item in a real pass. So these numbers are the jitter that Python and macOS contribute, and nothing else. The real ones arrive on the next bench session, automatically, because the line prints itself.

⭐ What they do establish, and it is worth having:

- The mean matches the 87 Hz measured in a completely different way in 2026-08-13, so the instrument agrees with the one number already known.
- Four passes in about 7000 were longer than the arm's own 33 ms response time, so stalls are real and rare: about one pass in 1800.
- Every one of those stalls happened in a session with no cameras attached at all. So camera work is not the only source of jitter, and moving the cameras off the loop would not have prevented any of these four.

**What the worst pass turned out to be, in the simulated session**

It is reproducible: 67 to 88 ms, always at about 19 seconds in, immediately after the line `MODE: TELEOP on B+G`. Entering teleop builds the inverse-kinematics solver for each selected arm inside the loop's pass. Measured on its own, the first `CartesianTeleop` of a process costs 33 ms because MuJoCo loads its model, and later ones cost about 4 ms each.

The other three stalls are 36 to 40 ms, one at the start of a recording and two during replay.

⚠️ A first guess was wrong, and measuring it took two minutes. That is the whole argument for measuring. Saving a recording looked like the obvious cause, because it writes a file from inside the loop. Timed directly, saving slot 8 (225 samples, 31 KB) takes 1.2 ms.

## 3. What the cameras actually cost

These numbers come from a real recorded frame from this rig, compressing the same 1280x720 picture 40 times and taking the average:

| size | quality | kilobytes per picture | milliseconds to compress | 3 cameras at 30/s |
|---|---|---|---|---|
| 1280x720 | 90 (what is used now) | 192 | 1.4 | 16.9 MB/s |
| 1280x720 | 75 | 110 | 1.2 | 9.6 MB/s |
| 848x480 | 90 | 96 | 0.6 | 8.4 MB/s |
| 848x480 | 75 | 59 | 0.5 | 5.2 MB/s |
| 640x360 | 90 | 63 | 0.3 | 5.5 MB/s |
| 640x360 | 75 | 39 | 0.3 | 3.5 MB/s |

**So three cameras at 30 pictures a second need about 0.13 seconds of compression per second.**

The station has 32 cores. Compression is not a load.

**What the loop does with a picture, per pass**

It reads a reference to the newest frame from each camera under a lock, builds one small record, and puts it on a queue if the frame is new. No picture is copied. The compression happens in a writer thread and the reading happens in a reader thread.

⚠️ So if the cameras do cost the loop anything, the mechanism is not the work.

It is that six extra Python threads exist, and Python lets only one of them run its own bytecode at a time. Every handover costs the main loop a little latency. That mechanism produces jitter rather than a lower average rate, and the worst pass has never been measured.

## 4. Does the loop need to be fast at all?

This is the question worth asking before any redesign, and the answer is no.

- The arm cannot follow faster. Its behaviour was measured as a fixed delay. The position it reaches lags the position it was told by about 0.04 to 0.10 radians, plus 0.033 seconds times the speed. That 0.033 seconds is the same on all six joints ([ROADMAP.md](ROADMAP.md) section 7.5.1), and a 0.033 second delay is a 30 Hz limit. Setpoints arriving at 83 Hz are already about three times faster than that.
- The safety limits do not care. `SafeRobot` computes its allowance from the *measured* time since the last pass, so a slower loop takes proportionally larger steps at the same speed limit. Nothing silently loosens.
- The recording does not care. It stores one row per pass, and the episode export resamples to 30 ticks a second. Even 40 passes a second would leave more rows than the export uses.
- A person does not care. In teleop the operator closes the loop by eye, at a few Hz.

⭐ So the loop has roughly three times the headroom that anything measured needs.

"83 instead of 100" is a number that looks wrong and costs nothing.

## 5. If you did want to separate the cameras, here is the seam

The design is already most of the way there, and it is worth knowing why.

**The control loop does not actually need the pictures.**

It passes them through to the writer and never looks at them. The only reason they pass through the loop at all is so that one place decides when a recording starts, stops, and which slot it saves to.

**Everything downstream already works by timestamp, not by loop position.**

The episode export joins pictures to control ticks by nearest timestamp. Both clocks are the same clock, measured 40 nanoseconds apart on the station. So a separate process writing its own pictures and its own index would join exactly as well.

**The two method shapes that are the seam:**

```
CaptureSet.sample()  -> dict of camera name to Frame or None     # the loop reads this
FrameSink.offer(dict)                                             # the loop writes this
FrameSink.stop()     -> the per-camera index, counts included
```

Both are declared as interfaces in `src/yam/seams.py` (`FrameSource` and `FrameConsumer`), so an implementation backed by a separate process can be substituted without the loop knowing. ⛔ Nothing needs it today. The seam is declared so the option stays open and so the shape does not drift.

What building it would actually involve, if the day comes:

1. A camera process that opens the cameras, writes JPEGs and an index, and takes start and stop over a pipe or a file.
2. A `FrameConsumer` in the session that forwards start and stop instead of frames.
3. One new failure to handle: the camera process dying while a recording runs. Today a camera failure shows up as dropped frames counted in the same process. In a separate process it would be silence instead. That is exactly the kind of failure this repo keeps finding.

⚠️ Point 3 is why this is not obviously an improvement.

Separating the processes gives you isolation, and gives you a new process to supervise.

## 6. The two changes that would actually move numbers

Both change the recorded pictures, so both are data decisions.

### Compression quality: 90 today, and 75 would be free

The recordings use quality 90. Quality 75 gives files 1.75 times smaller and compresses slightly faster.

**The question is whether anything can see the difference.**

The training format shrinks every camera view to 224 by 224 pixels, so that is where the comparison belongs. Measured against the uncompressed original, at 224 by 224:

| quality | difference from the original |
|---|---|
| 90 | 0.14% of full brightness range |
| 75 | 0.54% |
| 60 | 0.72% |

**Half a percent, on a picture that has been shrunk to a twentieth of its area.**

Nothing in the training pipeline can use that difference.

⭐ Recommendation: quality 75.

It is one constant, `JPEG_QUALITY` in `src/yam/cameras/writer.py`. ⚠️ It changes every recording made afterwards, so make the change once and never mid-collection, or a dataset will contain two kinds of picture.

### Capture size: the bigger lever, and the one with a real risk

Recording at 1280x720 and training at 224x224 throws away about 94% of the pixels. Capturing at 848x480 and quality 75 is 3.3 times smaller than today and still more than twice the training resolution in each direction.

⛔ But this is the one change you cannot undo.

You can always shrink a picture later. You can never recover detail you did not capture. Your own setup guide says it: mistakes in the data format are repairable only by collecting everything again.

⚠️ And the per-view size is still an open question with the team.

224 is what the guide says; their simulation renders 224 by 168. Until that is settled, capturing small would be a bet.

⭐ Recommendation: change nothing yet.

When the per-view size is settled, capture at two to three times it in each direction and no more. At 224 that means 640x480 or 848x480 rather than 1280x720.

For scale: at today's settings three cameras produce 13.1 MB a second, or 47 GB an hour. The station has 3.4 TB free, so roughly 72 hours of three-camera recording. Storage is not the constraint. The question is the cost per demonstration rather than running out of room.

## 7. Two more things that would be faster, and are not needed yet

**Hardware video encoding for the training export.**

The training directory's video is encoded by `ffmpeg` on the processor. The station has an RTX 5090, whose encoder would be much faster. ⛔ The catch: the export is deliberately strict about the encoding, because the loader calculates where each frame is rather than searching for it. A hardware encoder produces a different file, and `checks/check_dataset.py` would have to confirm every property again. Worth doing when export time is a real complaint. It is not one today.

**Exporting many episodes at once.**

Episodes are independent, so exporting them in parallel is as fast as your cores allow. `apps/build_dataset_stats.py` walks a directory of them one at a time. Worth doing when there are hundreds. There are three.

## 8. What to measure before changing anything

In this order, because each answer makes the next question smaller:

1. The worst single loop pass rather than the average rate. The average says the loop is fine. Jitter is what a redesign would fix, and nobody has looked at it.
2. Where one pass goes: CAN, Python, everything else. Without this, any speed work is guessing.
3. The loop rate with the cameras off and on, on the same machine, in the same session. The cameras' cost has been assumed and never isolated.

⚠️ And the general rule this whole file is an example of:

a measurement without its conditions becomes a claim. The 1.4 milliseconds above is one machine, one picture, one library version. Re-measure before you rely on it somewhere else.

---

**Where to go next:**

- [LAG.md](LAG.md) if your question is why one arm trails the other, or why teleop feels delayed. That file covers the arm's own response and the speed limits. This one covers the loop and the cameras
- [ARCHITECTURE.md](ARCHITECTURE.md) for how the system is built
- [FINDINGS.md](FINDINGS.md) section 76 for the camera measurements this file draws on
- [PLAN.md](PLAN.md) if you are rebuilding the station

*Written 2026-08-20. Every number in it is dated by that. If you re-measure and get something different, change this file rather than trusting it.*
