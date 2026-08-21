3. Simulation-Owned Capture Scheduler

SimCamera.connect() validates and registers the camera. Simulation remains the sole owner of MuJoCo state and rendering. As simulated time advances, it publishes frames that are due.
Simulation.step()
    |
    +-- advance physics
    |
    +-- determine which cameras are due
    |
    +-- render each required camera
            |
            +-- replace SimCamera's latest frame
            |
            +-- append to recorder queue
Consumers then get two separate behaviors:
- read() immediately returns the latest frame, possibly the same frame as the previous call.
- A recorder drains the queue if it must preserve every published frame.
Disk writing or video encoding can happen in a background thread because that worker receives completed arrays and never touches live MuJoCo state.
Advantages
- Deterministic simulated-time FPS.
- Safe ownership of MuJoCo and the renderer.
- Independent camera rates.
- Same consumer-facing behavior as a real camera.
- Supports both latest-frame reads and lossless recording.
- Can run faster or slower than wall-clock time.
Disadvantages
- Simulation must know about registered cameras.
- Capture scheduling must handle reset and skipped time carefully.
- A 60 FPS camera in a 30 Hz control loop requires explicit semantics.
For your chosen semantics, a 60 FPS camera can publish two frame records per control tick. Because you do not require intermediate physics states, both records can reference the same rendered image while having distinct sequence numbers and simulated timestamps.