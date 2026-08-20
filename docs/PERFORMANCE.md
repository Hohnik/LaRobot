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

1. The loop is not slow because of the cameras. It was measured at about 87 passes a second before any camera code existed. The cameras cost a few passes a second on top of that, and the exact amount has never been isolated.
2. The camera work is tiny. Compressing one 1280x720 picture takes 1.4 milliseconds. Three cameras at 30 pictures a second therefore need about 0.13 seconds of compression work per second of recording. That is not a load worth restructuring for.
3. Reading and compressing already happen off the loop. Each camera has its own reader thread and its own writer thread. The loop hands over a reference and moves on.
4. 83 passes a second is already about three times faster than the arm can respond to. The arm follows a moving target with a delay of about 0.033 seconds, so it behaves like something with a 30 Hz limit. Sending it setpoints at 83 Hz is not the constraint on anything.
5. Two changes would actually move the numbers, and both are about the pictures rather than the code: the compression quality, and the size you capture at. Both change the recorded data, so both are Julien's decision.

## 2. Where the loop's time actually goes

**What is measured.**

The loop asks for 100 passes a second. It has never achieved that. [FINDINGS.md](FINDINGS.md) section 31.1 measured about 87 on the Mac. It was found because a playback summary did not add up: 4.0 seconds of work reported inside 4.6 seconds of wall clock. 4.0 ÷ 4.6 = 0.87.

**What is not measured, and this matters.**

Nothing has ever broken the pass down into its parts. The candidates, in the order they are likely to matter:

| candidate | what is known |
|---|---|
| the CAN round trip | 14 motors are commanded and read every pass over two USB adapters at 12 Mbit/s each. Unmeasured, and the most likely largest share |
| Python work per pass | mode logic, the status line, the tracking log. Unmeasured |
| inverse kinematics | **measured: about 0.1 ms for both arms**, against roughly 10 ms available. Not the problem |
| camera hand-off | reading the newest frame reference and putting it on a queue. Microseconds |

⛔ So the honest state is that the loop's own cost has never been attributed.

Before anyone changes the design to make it faster, measure where the pass goes. Changing an unmeasured thing is how you spend a week and learn nothing.

⭐ The instrument already exists.

Every playback writes a tracking file that records `loop_hz`, so the rate is captured per run rather than noticed once. What it does not record is the worst single pass. That is the number that would tell you whether the cameras cause jitter. That is a small addition and it is not built.

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

[ARCHITECTURE.md](ARCHITECTURE.md) for how the system is built · [FINDINGS.md](FINDINGS.md) section 76 for the camera measurements this file draws on · [PLAN.md](PLAN.md) if you are rebuilding the station.

*Written 2026-08-20. Every number in it is dated by that. If you re-measure and get something different, change this file rather than trusting it.*
