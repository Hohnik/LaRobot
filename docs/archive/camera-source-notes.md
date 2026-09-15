# Historical camera source notes

Source snapshot: `apps/camera_view.py` at `52421fc`, before the September 15 camera extraction.
These are historical explanations, preserved verbatim. They include dated measurements,
old device inventories and claims that may be superseded. Current contracts live in the
camera/display modules; current implementation status is in [CLEANUP](../CLEANUP.md).
A monochrome image alone does not establish that a stream contains depth data.

## Lines 88-99: comment block

```text
# Live-switchable capture sizes, bound to keys 1..6.
#
# ⚠️ A UVC webcam only supports a FIXED LIST of modes, and asking for anything else
# silently gets you the nearest one it does have. Julien noticed `--probe` reporting
# "424x240 requested -> 640x360 actual": 424x240 is not a C920 mode, so the driver
# substituted. These are all real C920 modes, so what you ask for is what you get.
#
# ⭐ For the TERMINAL view, capture resolution barely affects picture quality — the
# renderer downsamples to the character grid anyway. What it does affect is
# **latency and CPU**: fewer pixels to transfer, decode and scale. So 320x180 is the
# right choice for the lowest-latency terminal view, and the large sizes are for the
# windowed view or for recording.
```

## Lines 107-127: key_sizes docstring

```text
    """The six capture sizes bound to keys 1-6 — **this camera's own modes** if known.

    ⛔⭐ THE BUG THIS FIXES, reported by Julien on 2026-08-11: *"the numbers when I
    press them don't allow for all the quality options. They cycle between about
    three, and not in the correct order either."*

    `SIZES` above is a list of **C920** modes. Point it at a different camera and a
    UVC device silently substitutes the nearest mode it does have, so several keys
    collapse onto the same result. On the MacBook Air camera — whose seven modes are
    640x480, 1280x720, 1552x1552, 1760x1328, 1328x1760, 1920x1080, 1080x1920 — keys
    1 to 4 all land on 640x480 and only 5 and 6 differ. **Exactly three distinct
    sizes, which is exactly what he saw**, and its portrait modes are why the order
    looked wrong too.

    ⭐ That symptom is also evidence: it says he was driving the built-in camera while
    the tool told him it was the C920 — the naming bug, showing up in a second place.

    Asking the device for its own modes removes the guesswork. Six spread evenly
    across the range, always including the smallest and largest, always ascending, so
    every key does something distinct and `6` is always "as good as this camera gets".
    """
```

## Lines 130-137: comment block

```text
    # ⛔⭐ SLOW MODES ARE NOT "BETTER". The C920 advertises 2560x1472 at a **measured
    # 2 fps** — a stills mode. The first version of this function offered it on key 6
    # as "the best this camera can do", and Julien found the obvious consequence:
    # *"the frame rate drops to like two frames per second"*. It also bought nothing,
    # because what reaches the terminal is capped far below that anyway.
    #
    # A live view wants the sharpest mode that still MOVES. AVFoundation reports a
    # max frame rate per format, so this is a measurement rather than a judgement.
```

## Lines 141-144: comment block

```text
    # ⚠️ Landscape only, when there is a real choice. Apple's camera advertises
    # 1080x1920 and 1552x1552 — Center Stage crop modes — which by pixel count sort
    # ABOVE 1920x1080 and would put a portrait or square frame on key 6, where the
    # operator expects "the best this camera can do" for a video view.
```

## Lines 154-168: comment block

```text
# ============================================================================
#  CAMERA IDENTITY — which index is which camera
# ============================================================================
#
# ⭐ WHY THIS SECTION EXISTS. Four cameras are visible on this Mac — the built-in
# one, the D405 on arm B, the C920, and Julien's iPhone over Continuity — their
# indices move on replug, and OpenCV reports **no name at all** (verified on OpenCV
# 5.0: `cv2.videoio_registry` exposes backends, never devices). Driving the arm from
# the wrong camera's point of view is exactly the class of mistake FINDINGS §0 #5
# was written about, where an adapter chosen by index silently retargeted the other
# robot.
#
# The way out is that **macOS will tell us the names** even though OpenCV will not.
```

## Lines 171-177: MacCamera docstring

```text
    """One camera as macOS reports it.

    `modes` is the set of `(width, height)` this device can actually deliver, read
    from AVFoundation. ⭐ It is the load-bearing field: it is what lets an OpenCV
    index be **identified by measurement** instead of guessed at by position.
    Empty when only `system_profiler` was available.
    """
```

## Lines 183-186: comment block

```text
    # (width, height, max_fps) per mode. ⛔ Load-bearing for a LIVE view: the C920
    # advertises 2560x1472 and it is a **2 fps** mode. Offering it as "the best this
    # camera can do" is how a viewer ends up at 2 fps — measured 2026-08-12, after
    # Julien hit exactly that.
```

## Lines 191-197: usb docstring

```text
        """`vid:pid` in hex, or None for a camera that is not USB.

        ⚠️ macOS writes the IDs in **decimal** inside the model string:
        `UVC Camera VendorID_32902 ProductID_2907` is `8086:0b5b`. Every datasheet,
        `ioreg` dump and USB tool speaks hex, so convert once here rather than
        leaving two number bases loose in the codebase.
        """
```

## Lines 227-236: _av_cameras docstring

```text
    """Cameras straight from AVFoundation, **with their supported modes**.

    Needs no camera permission — enumerating is not capturing — so the agent can run
    this even though it can never open a stream (FINDINGS §21.1).

    ⚠️ Returns `[]` when the binding is missing (a non-macOS machine, or a checkout
    that skipped the optional dependency). The caller then falls back to
    `system_profiler`, which gives names but **no modes**, and without modes there is
    no way to identify an index — so the tool refuses to name rather than guess.
    """
```

## Lines 272-293: mac_cameras docstring

```text
    """Every camera macOS knows about. **The ORDER MEANS NOTHING — see below.**

    ⛔⭐ THE MISTAKE THIS DOCSTRING EXISTS TO PREVENT, because it was made here on
    2026-08-11 and shipped.

    The first version of this file assumed macOS's n-th camera was OpenCV's n-th
    index. It said so out loud, cross-checked it, and published a falsification
    procedure. **Julien ran that procedure and it failed:** covering each camera in
    turn showed the C920 answering on index 0, where macOS lists the built-in camera,
    and the built-in one answering on index 2, where macOS lists the C920.

    It is worse than a coincidence of one API. **Three separate macOS enumerations —
    `system_profiler`, `devicesWithMediaType:`, and an `AVCaptureDeviceDiscoverySession`
    asked in two different device-type orders — all return the SAME order, and it is
    not OpenCV's.** (A pattern, offered only as a lead if the measurement below ever
    fails: OpenCV's order looks like USB cameras sorted by location ID first, then
    built-in, then Continuity. `0x1120000` (C920) < `0x1210000` (D405), which is the
    order it used. Do not build on this; it is one observation.)

    ⭐ **So the order is never used.** Identity comes from `identify_indices()`, which
    asks each index for a resolution only one device supports and sees who answers.
    """
```

## Lines 302-313: ProbeResult docstring

```text
    """What one camera index answered when opened.

    ⚠️ `fps` is **not trustworthy** and is shown only for completeness: the same
    device reported 1 fps on one run and 30 on the next, so OpenCV is deriving it
    from frame timing rather than reading the format. `width`/`height` were stable
    across every run and are.

    ⭐ Since 2026-08-19 `width`/`height` come from the returned frame's own shape whenever a
    frame arrived, and from `cap.get` only when none did. A handle answers the size it was
    ASKED for, which is a different question from the size it hands over, and confusing the
    two is [FINDINGS §76.0](../docs/FINDINGS.md).
    """
```

## Lines 326-344: frame_is_mono docstring

```text
    """True if the colour channels are identical, False if not, **None if the frame
    cannot say.** That third case is the whole point.

    ⛔⭐ THE BUG THIS FIXES, found by Julien's own `--list` output on 2026-08-11.
    A **black** frame has three identical channels, so the first version declared it
    `MONO — depth/IR, not a picture` and pointed that verdict at his **iPhone**,
    which is neither a depth camera nor an infrared one. The frame was black because
    the probe read it the instant the camera opened, before the sensor had exposed.

    **A frame with no variation carries no colour information at all.** The honest
    answer is "unknown", and dressing it up as a measurement is exactly the confident,
    plausible, wrong answer of FINDINGS §0 — produced, this time, by the code written
    to prevent it.

    What it is genuinely for: the D405's UVC entry is a depth stream widened into
    three equal channels, while a colour camera disagrees between channels almost
    everywhere — white balance alone guarantees that. On a frame that actually has
    content, this separates the two.
    """
```

## Lines 355-376: probe_indices docstring

```text
    """Open each index in turn and record what answered.

    ⛔⭐ **THE FIRST FRAME IS NOT THE PICTURE**, and reading it as though it were sent
    this investigation down a blind alley on 2026-08-11. The original probe took
    exactly one frame the instant the camera opened. Apple's built-in camera takes
    roughly half a second to expose, and the iPhone over Continuity takes longer
    still — so the built-in camera reported brightness 5 while it was looking at a
    bright room, and the iPhone reported `NO FRAME`. **Both readings were about
    sensor warm-up and nothing else**, and the brightness column is precisely what
    the operator is told to use to tell cameras apart.

    So this now reads frames until one actually has variation, or `settle` seconds
    pass, and reports how long that took — a slow camera is worth knowing about
    rather than silently averaging away.

    `read_frames=False` skips capture entirely: enough to count devices, not enough
    to say what they show.

    ⚠️ A closed index does **not** end the loop. Assuming indices are contiguous is
    the kind of tidy assumption this rig punishes; an unopenable index in the middle
    would silently shift everything after it.
    """
```

## Lines 393-399: comment block

```text
        # ⛔ THE SIZE COMES FROM THE FRAME when there is one. This block already held a real
        # frame and still reported `cap.get`, which answers what was REQUESTED rather than
        # what arrived. On 2026-08-19 that exact substitution printed "delivering 1280x720"
        # for a camera that delivered nothing all session (FINDINGS §76.0). Here it was
        # harmless because `ok` records whether a frame came, and it was still a claim
        # dressed as a measurement. `cap.get` remains the fallback for the no-frame case,
        # where there is nothing better and `ok` is False anyway.
```

## Lines 416-432: discriminating_mode docstring

```text
    """A resolution **only this camera can deliver** — the question that identifies it.

    Every camera advertises a fixed list of modes, and the lists differ sharply on
    this rig: the C920 offers 34 including `160x90` and `176x144`, the MacBook Air
    camera offers 7 including the square `1552x1552`, the D405 offers `848x480`, the
    iPhone offers `1920x1440`. Ask a camera for a mode it does not have and **it
    substitutes the nearest one it does** — measured here in session 7, where
    `424x240` came back as `640x360`. So an *exact* match to a mode only one device
    owns is that device answering, whatever index it happens to be sitting on.

    Returns None when the camera shares every one of its modes with another — which
    ⚠️ **is exactly what will happen when the second D405 is plugged in.** Two
    identical cameras cannot be told apart this way, and the tool must say so rather
    than pick one.

    The smallest qualifying mode is chosen: least bandwidth, quickest to negotiate.
    """
```

## Lines 441-446: model_discriminating_mode docstring

```text
    """A resolution only this camera's MODEL can deliver — same-model twins may share it.

    ⛔⭐ WHY THIS EXISTS BESIDE `discriminating_mode` (FINDINGS §71.5): a hint-resolved D405 cannot be verified as the RIGHT D405 (identical twins), but it CAN be verified as *a* D405 — and that check is not optional. Two stale hint rows were found pointing BOTH D405 uniqueIDs at index 0, which is the C920's index today; a session trusting them would have recorded the webcam under a wrist camera's name with full confidence. Asking the opened index for a mode only the claimed model offers (`848x480` for the D405 on this rig) catches exactly that, and a cable swap between two same-model cameras remains the one case no software question can see.

    `cams` is the full list including `cam` itself; cameras sharing `cam`'s USB vid:pid (or, for non-USB devices, its name) count as the same model and do not disqualify a mode.
    """
```

## Lines 459-480: identify_indices docstring

```text
    """Work out which OpenCV index is which camera **by measurement**.

    ⛔⭐ THIS REPLACES A POSITIONAL GUESS THAT WAS WRONG. See `mac_cameras()` for the
    full account: macOS's enumeration order is not OpenCV's, three separate macOS
    APIs agree with each other and disagree with reality, and Julien's covering test
    caught it. Position is now never used for anything.

    The method: for each index, ask for every camera's discriminating mode in turn
    and see which one comes back **exactly**. A camera cannot deliver a mode it does
    not have, so an exact match is an identification rather than an inference.

    ⚠️ This rests on one measured property of the backend: `cap.get(FRAME_WIDTH)`
    after a `set` returns what the camera ACTUALLY did, not what was asked for. That
    is how the 424x240 → 640x360 substitution was discovered in the first place. If a
    future OpenCV echoes the request instead, every camera would match every mode —
    which shows up here as *every* index being ambiguous, not as a wrong name.

    Costs one open per index plus a few format changes: a few seconds, paid once at
    startup. It is deliberately **not cached**: a replug can reorder indices without
    changing anything a cache could key on, and a stale camera map is the failure
    this whole section exists to prevent.
    """
```

## Lines 583-613: find_camera_index docstring

```text
    """Which index is **this one** camera? Fast path for `--camera`.

    ⛔⭐ WHY THIS EXISTS, AND IT IS JULIEN'S COMPLAINT VERBATIM: *"when I enter that
    command, then it takes ages to choose the camera … it takes like twenty seconds
    for the camera to start running, which shouldn't be the case because I
    deliberately said which camera I want."*

    He is right, and the first version deserved it. `--camera c920` called the full
    `identify_indices()`, which opens **every** camera and asks **every** camera's
    question at each index — 4 opens and 16 reconfigurations to answer a question
    about one device. Worse, it woke his iPhone over Continuity every single time,
    which is the slowest device on the bus and was never wanted.

    Two changes, and together they turn ~20 s into about one open:

    1. **One question, not N.** Only this camera's discriminating mode is asked at
       each index. Stopping at the first exact match is safe *because* the mode is
       unique to this camera — and if it were not unique, `discriminating_mode()`
       returns None and we refuse before probing anything (two D405s, when the
       second arrives).
    2. ⭐ **A remembered index, always re-verified.** `config/camera_index_hint.json`
       records where this camera was last found and that index is tried first.

    ⚠️ **The hint is a place to look first, NOT a cached answer** — and that
    distinction is the whole reason it is safe. FINDINGS §22 argues against caching
    the identity, because a replug can reorder indices without changing anything a
    cache could key on. Nothing here trusts the hint: the camera at that index is
    still asked the same question, and a wrong hint costs one extra open and then
    falls through to the scan. A cache you verify on every use is not a cache of the
    answer; it is an ordering of the search.
    """
```

## Lines 641-644: comment block

```text
            # ⭐ HANDED BACK STILL OPEN. Opening a camera on macOS costs seconds, and
            # this one is already open and already the right device — releasing it so
            # the caller can open the same index again was pure waste, and it was a
            # measurable part of the startup delay Julien reported.
```

## Lines 666-675: resolve_camera docstring

```text
    """Turn `--camera d405` (or `--camera 2`) into an index, or refuse loudly.

    ⛔ Never falls back to index 0, and — since 2026-08-11 — never falls back to
    position either. The index comes from `identify_indices()`, which measures it.

    `cams` and `identified` exist to be injected by the tests. ⚠️ They must stay
    injectable: without them every test run would open all four cameras — waking
    Julien's iPhone over Continuity — and a test suite with side effects on hardware
    is one people stop running.
    """
```

## Lines 676-679: comment block

```text
    # ⭐ On Linux the whole question is answered by /dev/v4l/by-id, so route there and never
    # touch the AVFoundation machinery below (which would find nothing and then probe every
    # index for no reason). One dispatch, and every caller of this function — the viewer,
    # capture_probe — works on both platforms (ROADMAP §8.2 item 49).
```

## Lines 777-782: comment block

```text
            # ⭐ The uniqueID is printed for ROADMAP §8.2 item 5's open hypothesis: if a
            # D405's AVFoundation uniqueID embeds its USB locationID (0x01220000 and
            # 0x01210000 for the two D405s measured in FINDINGS §70.6), then
            # serial→locationID→uniqueID→index closes identical-camera identification
            # with no root and no lens-covering. One glance at this line with both
            # D405s attached answers it.
```

## Lines 823-830: probe_modes docstring

```text
    """Measure the REAL frame rate at each resolution, with and without MJPG.

    ⚠️ This exists because the agent cannot run it. macOS grants camera access
    **per application**, and the permission Julien granted covers his terminal, not
    the process the agent's shell runs under — so every agent-side attempt returns
    `not authorized to capture video` no matter what the code does. The measurement
    therefore has to be a command he runs, which is what this is.
    """
```

## Lines 848-853: comment block

```text
            # ⛔⭐ THE "actual" COLUMN USED TO COME FROM `cap.get`, WHICH ANSWERS THE REQUEST.
            # In this table of all places: its whole purpose is telling the operator which
            # sizes a camera really delivers, and the line below is the one they read to pick
            # `--width`/`--height`. A camera that accepts a size and streams nothing at it
            # showed up here as working (FINDINGS §76.0, §76.2). The shape of a real frame is
            # the only honest answer, and this loop was already reading frames.
```

## Lines 872-878: configure_camera docstring

```text
    """Apply the capture settings to an already-open handle.

    ⭐ Split out of `open_camera` so an identification probe can hand its handle
    straight to the viewer. **Opening a camera on macOS is the expensive step** —
    seconds, not milliseconds — and doing it once to ask "who are you?" and again to
    actually watch was most of the startup delay Julien measured.
    """
```

## Lines 879-885: comment block

```text
    # ⛔⭐ THE SETTINGS THEMSELVES MOVED TO `yam/cameras/open.py`, and the reason is a
    # defect this exact comment predicted. It used to say the MJPG line was "load-bearing
    # on Linux, where this rig is ultimately headed" — and then the session's Linux camera
    # path was written with three bare set() calls and no codec request, because it does
    # not import this file. The C920 ran a whole recording at 10 fps instead of 30 as a
    # result (FINDINGS §76). Knowledge in a comment in a file you do not import is not
    # shared. One copy, both platforms.
```

## Lines 902-925: render_ansi docstring

```text
    """Turn a camera frame into coloured text that fills a terminal.

    ⭐ WHY THIS EXISTS. Julien, 2026-08-11: *"the order of my open programs on my mac
    constantly gets moved around whenever the cam opens to the desktop as a small
    window, and I have to re-sort the windows manually."* A native window makes the
    Python process a GUI application, so macOS brings it to the front and reshuffles
    his layout every single time. Nothing in OpenCV prevents that.

    Drawing into a terminal sidesteps it completely: he already keeps terminals open
    for the teleop session, so the camera becomes one more pane in a layout he
    already has, and no window is ever created.

    **The half-block trick.** A terminal cell is roughly twice as tall as it is wide,
    which would squash the picture. Printing `▀` with a foreground colour and a
    background colour puts **two** vertically-stacked pixels in one cell — the
    foreground paints the top half, the background the bottom. That doubles the
    vertical resolution and makes each rendered pixel close to square.

    ⚠️ **Colour codes are only emitted when the colour changes.** A full-colour cell
    costs ~40 bytes; a 100x50 render is 5000 cells, so a naive version writes 200 KB
    per frame and 6 MB/s at 30 fps, which the terminal cannot keep up with. Emitting
    a code only on change collapses that dramatically on real images, where
    neighbouring pixels are usually similar.
    """
```

## Lines 967-986: cell_size docstring

```text
    """Measure the character cell in pixels, or fall back and **say so**.

    ⭐ WHY THIS EXISTS. Two things were being guessed at once, and both were visible
    on screen. The grid geometry assumed a cell is exactly twice as tall as it is
    wide, which stretches the picture whenever the font disagrees; and the image sent
    in kitty/iTerm2 mode was a fixed 480 px regardless of how large the pane was, so
    on a big terminal it was upscaled into a soft mess — **which is exactly what
    Julien reported on 2026-08-11: "the resolution is not great … pressing the
    numbers doesn't do anything."**

    Both are answered by one number the terminal already knows. `TIOCGWINSZ` returns
    `ws_xpixel`/`ws_ypixel` alongside the row and column count, so the cell is simply
    pixels ÷ cells. kitty and Ghostty fill those fields in; Apple Terminal reports
    zeros, and a piped or captured run has no terminal at all.

    ⚠️ `measured` is returned, never hidden. An assumed cell size is a fine default
    and a terrible silent one — the status line prints which it used, because a
    fallback you cannot see is indistinguishable from a bug (the same lesson as `b`
    silently doing nothing).
    """
```

## Lines 998-1018: comment block

```text
# ⭐ How many pixels wide the image sent to the terminal may get, per protocol.
# MEASURED 2026-08-11 on this Mac, encoding a detailed 16:9 frame (gradients, hard
# edges, text and grain — a realistic camera picture), best of five:
#
#     width   kitty: PNG level 1      iTerm2: JPEG q60
#      480     3.8 ms   277 KB/frame    0.1 ms    16 KB/frame
#      640     6.7 ms   521 KB          0.3 ms    26 KB
#      720    ~8   ms  ~650 KB          ~0.4 ms   ~32 KB
#      960    16.1 ms  1266 KB          0.6 ms    46 KB
#     1280    28.8 ms  2259 KB          1.1 ms    70 KB
#
# ⛔ **The kitty protocol has exactly one compressed format, and it is PNG** — `f`
# takes 24 (raw RGB), 32 (raw RGBA) or 100 (PNG). There is no JPEG. That single
# protocol fact is why the terminal view is soft: PNG of a photo is ~20x the bytes
# of a JPEG and ~25x the encode time, so detail costs frame rate in Ghostty/kitty in
# a way it simply does not in iTerm2. 720 px keeps encoding to ~25% of a 33 ms frame
# budget; 1280 px would spend 87% of it before a single byte is written.
#
# The numbers above are the *floor*: the terminal must also read and draw them, and
# writing ~650 KB into a pty every frame is not free. The live `draw ms` readout is
# the real measurement — these values only pick a sane starting point.
```

## Lines 1023-1038: auto_image_width docstring

```text
    """Pixels wide to send, when the operator has not fixed it by hand.

    Three ceilings, and the smallest wins:

    1. **The box on screen** — `cols x cell.width` pixels. Sending more than the
       terminal can display is pure cost; the extra pixels are scaled straight back
       out again.
    2. **What the camera captured** — upscaling before transmission invents nothing
       and costs bytes.
    3. **The protocol's budget** — see `IMAGE_WIDTH_CAP`.

    ⭐ This is what makes keys 1-6 do something visible. They change the *capture*
    size, and previously the sent image was pinned at 480 px, so a sharper capture
    changed nothing on screen and the keys looked dead. With ceiling 2 in place, a
    bigger capture genuinely produces a bigger image — up to what the pane can show.
    """
```

## Lines 1044-1058: useful_image_width docstring

```text
    """The largest image that could ever be worth sending — the pane, bounded by
    what the camera actually captured. **No protocol budget here**: this is the
    ceiling the adaptive controller in `run_terminal` is allowed to climb toward.

    ⭐ Why a controller and not a constant. `IMAGE_WIDTH_CAP` was measured from PNG
    *encode* time, and on Julien's machine encode is not the wall: at 720 px he
    measured a ~40 ms draw where encoding accounts for perhaps 8 ms. The rest is
    writing ~650 KB into a pty every frame. That ratio depends on the terminal, the
    font size, the window, and what else the machine is doing — none of which a
    constant chosen on one afternoon can know.

    So the viewer measures its own draw cost and climbs to whatever this terminal
    actually sustains, which is the same discipline the rest of this repo applies to
    the hardware: **measure the consequence rather than predicting it.**
    """
```

## Lines 1069-1101: tune_image_width docstring

```text
    """Pick the next image width from the measured draw cost. Returns `(width, over)`.

    ⛔⭐ THE BUG THIS FIXES WAS A ONE-WAY RATCHET, and Julien's two screenshots contain the
    proof. He reported the terminal view *"gets worse over time, which is a bit weird…
    it has the highest quality when I use it for FaceTime."*

    The old rule was: shrink by 0.85 above the target, grow by 1.15 below 0.6 of the target,
    do nothing in between. His screenshots read `sent 520x292` at `draw 13.3 ms`, and later
    `sent 442x249` at `draw 10.5 ms`, with a target of 16.7 ms.

        520 x 0.85 = 442.0     exactly one shrink step

    And at 442 the cost is 10.5 ms, which sits **inside the dead band and above the 10 ms
    grow threshold**. So it could never climb back. Every width whose cost landed between
    10 and 16.7 ms was a fixed point, so one transient hiccup knocked the picture down a
    step and it stayed there. The dead band was added to stop oscillation, and it removed
    every path back up.

    ⭐ Three changes, and the first is the one that matters:

    1. **Shrink only after `SHRINK_AFTER` consecutive over-budget frames**, so a single
       hiccup cannot move it at all. This is what removes the ratchet.
    2. **Grow below 0.85 of the target** rather than 0.6, so the dead band is narrow and
       recovery actually happens.
    3. **Smaller steps** (0.93 down, 1.06 up) so the residual hunting either side of the
       target is too small to see.

    ⚠️ The camera was never the problem, and nothing here improves it. FaceTime looks better
    because it puts pixels on the screen, while this path encodes every frame as a PNG and
    writes it into a terminal. Ghostty implements only the kitty protocol, which has no JPEG
    at all (§21.4), and PNG of a photograph costs roughly 25x the encode time. **For a good
    look at the picture, drop `--term` and use the window.**
    """
```

## Lines 1114-1138: terminal_grid docstring

```text
    """Character columns and rows to use, **preserving the picture's aspect ratio**.

    ⛔ THE BUG THIS FIXES. The first version returned the whole terminal and let
    `render_ansi` squash the frame into it, so a 16:9 camera came out stretched to
    whatever shape the pane happened to be. Julien's screenshot showed it clearly.

    The geometry: a cell is `k` times taller than it is wide, so a grid of `C`
    columns by `R` rows occupies a box whose aspect is `C / (R · k)`. To display a
    source of aspect `A` undistorted we therefore need `C = A · R · k`, and we take
    the largest such grid that fits inside the terminal.

    ⭐ `k` used to be hard-coded to 2 — right for a typical font, wrong for any
    other, and a silent stretch when wrong. It is now **measured** from the
    terminal's own pixel geometry (`cell_size()`) and passed in, falling back to 2
    only when the terminal will not say. This matters in both drawing modes: blocks
    mode paints two stacked pixels per cell, and image mode hands the terminal a
    `C x R` cell box that it scales the picture into.

    `scale` shrinks it below the maximum, for a small corner view rather than a
    full-pane one.

    ⚠️ `margin_rows` reserves space for the status lines underneath. Reserve too
    little and the picture pushes them off the bottom, or worse, scrolls the whole
    view every frame.
    """
```

## Lines 1149-1157: comment block

```text
# ⭐ Terminals that can draw a real image, and how. Keys are what they set in the
# environment; values are the protocol they speak.
#
# ⚠️ This list is the reason `b` appeared broken. It toggled between "blocks" and
# `detect_term_mode()`, so in a terminal this list did not recognise, BOTH sides of
# the toggle were "blocks" and pressing it changed nothing visibly. A toggle whose
# two states can be identical is not a toggle — and worse, kitty was *detected* and
# then silently discarded because its protocol was unimplemented. Both are fixed:
# kitty is implemented, and the mode is always reported so a downgrade is visible.
```

## Lines 1174-1182: detect_term_mode docstring

```text
    """Best available drawing method, and a human-readable reason.

    Returns e.g. `("iterm", "iTerm.app supports inline images")` or
    `("blocks", "Apple_Terminal has no image protocol — coloured text only")`.

    ⛔ The reason is returned, not just the mode. A silent fallback to blocks is
    indistinguishable from a broken feature, which is exactly how `b` wasted
    Julien's time.
    """
```

## Lines 1215-1225: _downscale docstring

```text
    """Shrink to at most `max_width`, preserving shape. Payload is latency.

    The terminal scales whatever it receives into the cell box, so sending more
    pixels than the box can display is pure cost. Measured PNG payloads for a
    photo-like frame, and why the default is not 720p:

        1280x720   998 KB   31 ms encode   ->  40 MB/s at 30 fps. Impossible.
         640x360   283 KB    6.6 ms
         480x270  ~180 KB   ~3 ms          ->  the default
         320x180    78 KB    1.6 ms        ->  still 22x the detail of blocks
    """
```

## Lines 1235-1281: render_kitty docstring

```text
    """A real image, drawn by the kitty graphics protocol (kitty, Ghostty, Konsole).

    ⛔⭐ THE FLICKER, reported by Julien 2026-08-11: *"the image in the terminal is
    flickering because some frames seem to not be drawn."*

    Every frame used to begin `a=d,d=A` — **delete all images** — and only then
    transmit the new one. Between the delete and the new image being decoded there is
    nothing on screen, so at 30 fps the picture is blanked 30 times a second. Whether
    that reads as flicker depends on how fast the terminal decodes, which is why it
    got worse as the image got bigger.

    The fix is double buffering, the same idea as any graphics pipeline: pass
    `image_id` and `previous_id`, and the new image is **placed first, over the old
    one**, then the old id is deleted underneath it. There is no moment with nothing
    on screen. The caller alternates two ids each frame.

    ⚠️ Deleting still has to happen: `d=I` frees the image data as well as the
    placement, and without it a 30 fps stream leaks an image per frame into the
    terminal's memory. Omitting `previous_id` (as `--term-test` does) keeps the old
    delete-then-draw path, which is correct for a single still image.

    ⛔⭐ THE BUG THIS FIXES, and it is a good lesson in reading a spec properly.

    The first version encoded **JPEG** and labelled it `f=100`. But in the kitty
    protocol `f` takes only three values — `f=24` (raw RGB), `f=32` (raw RGBA) and
    `f=100` (**PNG**). **There is no JPEG.** So the terminal was handed JPEG bytes,
    told they were PNG, failed to decode them, and said nothing — because `q=2` had
    suppressed exactly the error message that would have explained it. Julien saw a
    blank screen in kitty mode while blocks mode worked.

    ⚠️ Two lessons worth keeping. **A format code is not a MIME type**: `f=100`
    named the container, and assuming it meant "some compressed image" is the same
    class of error as assuming an SDK flag means what its name suggests. And
    **suppressing errors cost more than the noise it saved** — `q=2` is right for a
    30 fps redraw, but it turned a one-line diagnosis into a session of guessing,
    which is why `--term-test` now exists to send one image with errors ENABLED.

    Three further protocol facts, all load-bearing:

    - **PNG is the only compressed format**, and PNG of a photo is large — hence the
      downscale. `IMWRITE_PNG_COMPRESSION=1` is deliberate: level 1 costs ~1.6 ms at
      320x180 where the default costs several times that, for a few percent of size.
    - **Images persist until deleted.** One per frame at 30 fps accumulates
      placements without bound, so `a=d,d=A` clears the previous frame first.
    - **The terminal replies on stdin**, which this viewer reads for keypresses, so
      `q=2` is needed in the live loop or every frame injects junk input.
    """
```

## Lines 1306-1322: term_test docstring

```text
    """Send a test image in **each** protocol, errors ENABLED, and report the replies.

    ⭐ This exists because `q=2` — correct for a 30 fps loop — silently swallowed the
    error that would have identified the JPEG-labelled-as-PNG bug immediately. One
    command now produces ground truth instead of a guess.

    kitty replies `ESC _G i=<id>;OK ESC \\` on success, or `ESC _G i=<id>;<ERROR>
    ESC \\` on failure. Anything else — including silence — means the terminal does
    not implement the protocol at all.

    ⭐⭐ It now tests **iTerm2's protocol too**, and the reason is worth stating: that
    one carries JPEG, and JPEG is ~25x cheaper than the PNG the kitty protocol
    forces. If a terminal happens to speak both — Ghostty is the open question on
    this rig — then the sharpness ceiling on the terminal view moves by a factor of
    two. That is worth ten seconds of looking at the screen. iTerm2's protocol
    defines no reply, so this half is a question to a human, not a measurement.
    """
```

## Lines 1349-1358: comment block

```text
        # ⛔⭐ `image_id` IS WHAT MAKES A REPLY POSSIBLE, and leaving it out is why
        # this test reported a false negative on 2026-08-12. The kitty protocol keys
        # its response to an image **id** (`i=`) or number (`I=`); with neither, the
        # terminal has nothing to answer *about* and correctly says nothing. Ghostty
        # 1.3.1 was accordingly declared "does not implement this protocol" while it
        # was, at that very moment, drawing the camera view in kitty mode.
        #
        # A test that says "unsupported" when it means "I asked a question that has
        # no addressee" is the same confident-wrong-answer this file keeps being
        # rewritten to avoid — this time in the diagnostic itself.
```

## Lines 1361-1366: comment block

```text
        # ⛔⭐ READ THE DESCRIPTOR, NOT `sys.stdin`. This had the same defect as
        # KeyReader: `select()` asks the file descriptor whether bytes are waiting
        # while `sys.stdin.read(1)` pulls them into Python's own buffer, where the
        # descriptor cannot see them. The visible result on 2026-08-12 was a reply of
        # exactly `'\x1b'` — one byte of a longer answer — which this then reported as
        # **"⛔ ERROR"** about a terminal that had answered perfectly well.
```

## Lines 1377-1381: comment block

```text
        # ⭐ And now the OTHER protocol. It is worth knowing whether this terminal
        # takes iTerm2's escape as well, because that one carries **JPEG**: measured
        # 0.3 ms and 26 KB per 640px frame against PNG's 6.7 ms and 391 KB. A
        # terminal that speaks both should be driven with iterm mode, not kitty.
        # ⚠️ iTerm2's protocol defines no reply, so only a human can answer this.
```

## Lines 1411-1416: render_iterm docstring

```text
    """A real image, drawn inline by iTerm2/WezTerm at full resolution.

    The frame is JPEG-encoded and base64'd into iTerm2's inline-image escape. JPEG
    rather than PNG deliberately: a 320x180 PNG is several times larger and the whole
    payload is written to the terminal every frame, so size is latency.
    """
```

## Lines 1427-1437: run_terminal docstring

```text
    """Draw the camera into this terminal. Creates no window at all.

    `label` is the camera's name, shown in the status line so a two-terminal setup
    can never be confused about which arm's view is which. `cam` supplies that
    camera's real capture modes for keys 1-6 (see `key_sizes`).

    ⛔ Quitting is guaranteed: `q`/ESC, Ctrl-C and the `finally` block all restore the
    cursor and colours and stop the grabber thread. Julien asked specifically that
    every test be quittable, and a tool that leaves a terminal without a cursor is one
    people stop reaching for.
    """
```

## Lines 1483-1487: comment block

```text
                    # ⛔ Keys must still be read here. Before the first frame arrives
                    # this branch skipped the key handler entirely, so a camera that
                    # never delivered one left `q` dead and Ctrl-C as the only way
                    # out — against Julien's standing requirement that every test be
                    # quittable.
```

## Lines 1497-1506: comment block

```text
                # ⛔⭐ REDRAWING A FRAME THE TERMINAL ALREADY HAS IS PURE HARM, and it
                # was happening constantly. The display loop runs faster than the
                # camera delivers — at 30 fps capture and a ~18 ms draw it goes round
                # about 55 times a second — so nearly half of every second was spent
                # re-encoding and re-transmitting an identical picture. In kitty mode
                # each of those redraws also deleted and replaced the image, which is
                # **the flicker Julien reported**: "some frames seem to not be drawn".
                # Skipping them halves the terminal's load and removes half the
                # flashes. `dirty` covers keypresses, which must redraw at once
                # rather than waiting for the next frame.
```

## Lines 1543-1546: comment block

```text
                    # ⭐ The status line answers "did that keypress do anything?" —
                    # which is the question that made keys 1-6 look broken. Every
                    # number a key can change is on screen: the capture size, the size
                    # actually sent to the terminal, the cell grid, the draw cost.
```

## Lines 1573-1576: comment block

```text
                    # ⭐ The draw cost is displayed because it is the latency the
                    # SOFTWARE controls. If it approaches the frame interval the
                    # terminal cannot keep up, output backs up in the pipe, and lag
                    # grows without any single component looking wrong.
```

## Lines 1579-1589: comment block

```text
                    # ⭐⭐ CLIMB TO WHAT THIS TERMINAL ACTUALLY SUSTAINS. Julien:
                    # *"the max resolution that can be sent is 720x405 … it doesn't
                    # really make any sort of difference"* — because 720 was a
                    # constant picked from PNG encode cost on one machine, and on his
                    # the cost is dominated by writing the bytes, not encoding them.
                    #
                    # Spend at most half the frame interval drawing, and use the
                    # measured draw to decide. Backs off fast (×0.85) and climbs
                    # slowly (×1.15) with a dead band between, because overshooting
                    # costs frame rate the operator notices and undershooting only
                    # costs detail they can ask for with `]`.
```

## Lines 1605-1608: comment block

```text
                        # ⛔ CYCLES through all three, rather than toggling against a
                        # detection that may return the mode you are already in. The
                        # old two-way toggle was a no-op in any terminal the detector
                        # did not recognise, which is indistinguishable from broken.
```

## Lines 1670-1676: comment block

```text
    # ⭐ 1280x720 by default. An earlier version defaulted to 640x480 on the theory
    # that USB 2.0 bandwidth capped an uncompressed 1080p stream at ~5.8 fps, which
    # matched the 5 fps Julien saw. **That theory was REFUTED by --probe on
    # 2026-08-11**: the camera delivers ~30 fps at every size up to 1920x1080, so it
    # is compressing and bandwidth was never the constraint. The 5 fps came from the
    # viewer's own frame-draining loop (see FrameGrabber). Resolution is now a free
    # choice, and 720p is a good default; keys 1..5 change it live.
```

## Lines 1728-1731: comment block

```text
    # ⭐ Name → index BEFORE anything opens a device, so --probe and the viewer both
    # act on the camera that was asked for. Resolution refuses rather than guessing,
    # and it prints what it chose: FINDINGS §0 #5 is about an adapter picked by index
    # that silently drove the other robot.
```

## Lines 1762-1766: comment block

```text
    # ⛔⭐ ONE REAL FRAME BEFORE THE SIZE IS PRINTED. This line used to read the size back
    # from the handle, which returns what was ASKED for. The fps already said "requested"
    # and the size did not, so the same line carried a request and a claim side by side
    # (FINDINGS §76.0). `await_first_frame` is the same helper the session's camera open uses, so
    # there is one copy of "wait for a frame, with a deadline".
```

