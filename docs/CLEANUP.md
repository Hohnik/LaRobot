# Teleop cleanup: current work and handoff

Updated September 15, 2026. This is the current implementation handoff for the cleanup branch. The August evidence remains in FINDINGS and the earlier sections of HANDOFF.

The [architecture review](RESTRUCTURING.md) records the responsibility map and remaining sequence. Recording completion, shared camera acquisition, camera rendering, replay state and composite sequencing are now extracted, as described below. The controlled stop interaction also has its own module. Operator prompts, entry-point assembly and acquired-device cleanup still need work.

## Status

The cleanup continues on `codex/teleop-cleanup` from Fable's final `499d0b7`. Completed work covers startup/shutdown ownership, shared recording state, readability, slot rollback, recording completion, shared camera/display extraction, replay state, composite sequencing, and explicit shutdown causes. Current validation passes 959/959 checks across 54 files, 71/71 falsifier catches, and 32/32 isolated simulation checks. Structural and flag checks pass. No physical hardware has been operated and nothing has been pushed. All 3,455 original recording files still match the pre-switch hashes.

Correction to the earlier handoff: `c8069cc` was missing the operator's `effective_limits` import after the display extraction. The old structural log already reported that failure. A previous simulation log said 32/32, but it did not establish that the final saved source was valid. The earlier claim that the checkpoint was fully verified was wrong. This continuation restores the import and adds a direct application test that enters TELEOP on both arms. The current isolated simulation passes with that fix.

Julien supplied Fable's final message, which explicitly named `499d0b7`. The original restore was correct. He then authorized proceeding sensibly with the assessment's cleanup plan and requested detailed durable notes. No user decision is needed for the present software work.

## What must stay preserved

- `main` and `julien/yam-teleop-wip` preserve the original teleop source at `499d0b7`.
- `codex/saved-training-2026-09-15` preserves the three training edits in `3887161`; `feature/training-pipeline` stays at `0c94c23`.
- `.local-backups/2026-09-15_102317/working-files.tar` contains the pre-switch working files, including ignored local data and the nested I2RT checkout. All 4,167 archived regular files were checked by SHA-256. Top-level Git metadata and the original `.venv` were excluded and left in place.
- The original Python 3.14 training environment remains in `.venv`. The initial teleop validation environment is `.local-backups/2026-09-15_102317/teleop-validation-py312`, built offline with Python 3.12.12 and this branch's lockfile.
- All 3,455 existing recording files were verified unchanged after the restore and again after the cleanup. Do not run the simulator driver against these local files without isolating its output.
- Calibration files and operating limits are not cleanup material. Preserve their values, arm ordering, and the shared recording/playback timeline.

The backup is on the same disk. It protects against checkout mistakes, not disk failure. Local-only analysis and logs are under `agents/codex`; they do not travel with Git. This file records the implementation decisions that must travel.

## Repository identities

The reference teleop program is `apps/teleop_session.py` with the `yam` library. The team's newer implementation is under `src/robot` on the other branches. They are distinct efforts with different dependencies and different feature coverage.

The live GitHub refs were read during the assessment and fetched for inspection: `main` at `f08f96f` (September 9), `feature/dual-wield` at `2041c3d` (September 14), `feature/recording-pipeline` at `62671fd` (September 6), and the training branch at `0c94c23`. The remote no longer advertised the old teleop branch. Its local/cached refs remain intact. The newer simulation launcher is not a complete replacement for the physical-arm reference program. No newer Fable close-out was found; 31 unreachable local commits were older intermediate snapshots.

## Scope and decisions

1. Fix demonstrated correctness gaps before structural moves.
2. Keep existing `ArmSession`, `SafeRobot`, kinematics, motion, camera, and trajectory behavior. Do not rewrite these because the operator program is large.
3. Device cleanup starts with ownership at acquisition, not successful session initialization. All devices must be covered by a cleanup boundary, including failed startup.
4. Motor shutdown must not wait behind camera flushing, file deletion, or map persistence. Preserve the existing normal quit/park interaction. Partial startup has no validated park context and uses the existing disable/close operation; this work does not introduce automatic motion there.
5. Report an unsuccessful run with a nonzero process status. Preserve a specific failure reason for incident reporting, and treat missing motor-disable confirmations as a failure.
6. Tests should execute the application's coordination paths using fake handles. A copied policy expression or a source-string check cannot establish that the application invokes it correctly.
7. A normal environment and launch command should replace the temporary validation path. Preserve the training environment while doing that.
8. Extract recording/replay coordination and terminal interaction in small steps, after their behavior is characterized. A moved large function is not an architectural improvement by itself.
9. Keep history and safety rationale, but make current documentation describe current behavior without appended contradictory updates.

## Findings and current changes

### Acquired robots could miss shutdown

The original builder returned an enabled handle before `ArmSession` was constructed and before its first position read. Only afterward was the session appended to `arms`. A failure in that gap left no cleanup entry. An injected constructor failure against the real `main()` reproduced it, with no shutdown call and return status zero.

The application now registers the raw returned handle immediately in a separate `robots` dictionary. Shutdown iterates that dictionary, so an incomplete session does not hide a live handle. The `arms` list continues to contain fully initialized sessions for motion, state, and reporting.

### Peripheral cleanup and early returns

Cameras and puck assignment originally happened before the main `try/finally`. A missing puck returned without explicitly closing capture. A puck's `set_nonblocking()` and reader construction also preceded registration of its handle.

Those startup phases now run inside the cleanup boundary. Puck handles are recorded before their configuration. Each puck close and each peripheral cleanup is attempted independently. Robot shutdown precedes potentially slow camera/file operations. Signal and thread exception handlers are restored when the invocation ends.

### Failed runs appeared successful

The outer exception handler printed an error and then returned zero. It now records a nonzero status and a stop reason. Interrupted startup returns 130. Controlled fault stops also return failure; planned quit keeps success. Missing disable confirmations are named. Incident records can include acquired handles even when no `ArmSession` completed construction.

### The builder could ignore a failed gripper verification

Reading the code below the application boundary found another ownership gap. A normal exception from the first normalized gripper read was swallowed, while a RuntimeError could escape without cleanup. Both occur after the vendor returns a robot.

The builder now refuses when verification fails, closes its acquired handle, and re-raises the failure. NaN also fails the range check. Interrupted verification and wrapper construction are covered. The six-joint/no-gripper path applies the same ownership rule when wrapping fails. Failures internal to the vendor constructor before it returns a handle remain the vendor's responsibility; this change cannot clean up an object it never receives.

## Evidence and limits

The first lifecycle runs passed 883/883 checks across 45 files, 71/71 falsifier catches, and 1,860/1,860 documentation links. A later full run caught two README prose regressions introduced while writing launch instructions; these were fixed without raising a ceiling. The recording-lifecycle checkpoint passed 891/891. The later readability/save checkpoint passed 902/902, as reported in Status. Logs for the first increment are in `agents/codex/validation/lifecycle-*.txt`.

Baseline earlier today: 843/843 checks across 41 test files; 71/71 catches across five deliberately broken-input scripts. These were run with the isolated Python 3.12 environment. All existing documentation links resolved. No physical hardware claim follows from these checks.

New focused tests cover 16 application lifecycle/help cases, ten builder cases, ten camera startup cases, and four camera teardown cases. They inject failures before the control loop and use fake devices, including mocked physical-device factories for puck startup. No physical constructor or real device API is executed.

The pre-fix audit under `agents/codex/audit_lifecycle.py` intentionally asserts the old defects. It is retained as historical evidence, not a current passing test. Current regression tests live in `tests/test_session_lifecycle.py`, `tests/test_robot_startup.py`, `tests/test_camera_startup.py`, and `tests/test_camera_cleanup.py`.

The isolated simulator passed 32/32 interaction checks, including recording, replay, composite runs, and both arms parking. It ran in a disposable copy with its own recordings and config. The driver now invokes its own Python interpreter instead of resolving another environment through `uv run`.

The dedicated root launcher `./teleop` uses Python 3.12 and `.venv-teleop`, leaving `.venv` intact. Help and a two-arm dry run passed. Help initially crashed because argparse interpreted a literal percent sign in a help string; the escape is fixed and a regression now renders actual help.

Both camera factories now own captures immediately and close partially started readers after failure. FrameSink also stops writers already started if a later writer cannot start. Teardown attempts all readers/writers even when one fails, reporting the collected failures. Device-reader release runs even when joining its thread fails.

The shared trajectory lifecycle and slot persistence are now extracted, as detailed below. The later camera and replay extractions are described below; terminal coordination remains. Per-take camera ownership and completion validation are implemented in the continuation below. The architectural cleanup is still incomplete.

## Decisions still reserved for the operator

No changes to motion limits, calibration, camera exposure policy, or the physical stopping policy are bundled into cleanup. No physical motor setpoints or real camera sessions are run by this agent. Hardware validation belongs to Julien. Pushing to the team remote requires his explicit instruction. The Linux station has not been contacted during this work.

## Recording lifecycle checkpoint

`yam.recording_session.RecordingSession` now owns active/frozen trajectory state, the shared start time, labels, and ordered mode history. The application actually calls its start/sample/freeze/discard operations. It still reads measured joint positions in arm layout order, controls the slot prompt, and owns camera I/O. Eight direct tests cover the common timeline, frozen save prompt, mode provenance, simulation marker, labels, overwrite refusal, reset, and layout errors.

Manual stop and sample-limit stop now both call `freeze()`. Previously only manual stop finalized the mode history; a take stopped by the 100,000-sample limit could describe only the modes it started in. The new path records every mode seen while preserving the separate `simulated` marker. Existing recording schemas, file locations, key bindings, and overwrite confirmation remain unchanged.

The application now imports `git_commit` and `dt_now` from `yam.provenance`. The obsolete ArmSession header has been replaced with its current responsibilities. Historical rationale remains in FINDINGS and Git. Current README/HANDOFF entry points distinguish the August hardware evidence from this software-only continuation.

At the recording-lifecycle checkpoint, the operator `main()` still spanned roughly 3,900 lines. The later readability pass below reduces that further; architectural work remains. Camera state, slot persistence, replay/composite coordination, terminal prompts, and the control loop still meet in that function. Reducing a line count is not the acceptance criterion; removing duplicated decisions and testing the actual application boundary is.

## Next work, in order

1. Extract acquired-device cleanup and terminal prompt ownership in bounded steps. Explicit shutdown causes and the controlled stop interaction are complete; partial-startup ownership and motor-first cleanup must remain intact.
2. Separate argument/plan assembly and thin the entry point after the remaining owners are explicit. Preserve confirmations, selection, physical stopping behavior and the existing motion limits.
3. Add recovery inspection for retained frame directories and interrupted slot publication. Inspect actual files before proposing repairs. Joint samples held only in memory are not automatically preserved on process exit.
4. Re-read the team's refs before proposing specific transfers. The September fetch is a dated snapshot; the newer dual-wield simulator has narrower behavior than this reference operator.

The [architecture review](RESTRUCTURING.md#implementation-sequence-and-acceptance) gives the remaining boundaries and acceptance criteria.
The current simulator does not validate USB/CAN timing, gravity compensation, camera delivery, gripper calibration, collision avoidance, or real stopping behavior. Fake-device failure tests establish call ordering and ownership; they cannot establish that a motor physically disabled. Missing disable confirmations remain an operator-visible failure.

## Reproducing the software evidence without changing saved data

From the repository root, create the environment with `./teleop --help`. Then run `.venv-teleop/bin/python checks/run_tests.py`, `checks/run_falsifiers.py`, `checks/check_links.py`, `checks/check_flags.py`, and `checks/check_restructure.py` with the same interpreter. Do not change the existing prose ceilings to make a failing documentation check green.

For the full interaction driver, copy `apps`, `checks`, `src`, `config`, and `pyproject.toml` into a new temporary directory. Symlink that directory's `third_party` to this checkout's vendored tree. Run the copied `checks/drive_sim_session.py` with the absolute path of this checkout's `.venv-teleop/bin/python`, set `PYTHONPATH` to the copy's `src`, and use the copy as the working directory. This puts its synthetic recordings, slots 0/8, tracking logs and temporary settings under the copy. The driver requires a pseudo-terminal and uses timed input, so inspect its child log if an interaction check fails.

Local evidence paths:

- `agents/codex/validation/lifecycle-*.txt`: first lifecycle runs.
- `agents/codex/validation/recording-*.txt`: final coordinator suite, falsifiers, links, flags and simulation. `recording-simulator-copy.txt` names the disposable copy and its child log under `recordings/sim/last_drive.log`.
- `agents/codex/validation/launcher-help.txt` and `launcher-dry-run.txt`: actual launcher output.
- `agents/codex/validation/final-recording-preservation.json`: all 3,455 original recording files still match the pre-switch SHA-256 manifest after this work.
- `agents/codex/CLEANUP_ASSESSMENT.md`, `REPORT.md`, and `unreachable-commits.txt`: earlier investigation and recovery details. They are dated snapshots; this document takes precedence for current implementation status.


## Readability, slot-save and GUIDE continuation

Julien asked whether the files themselves had been cleaned up and then authorized the next steps. This pass addresses that concern directly:

- `apps/teleop_session.py` shrank from 5,055 to about 3,850 lines. Its `main()` shrank from 3,898 to about 3,100 lines. It remains large; camera startup, replay/composite state, prompts and dispatch still need separate ownership.
- 87 comment/docstring blocks moved verbatim to [archived source notes](archive/teleop-source-notes.md). Concise explanations of current constraints remain at the call sites. The removed notes are explicitly historical and can contain superseded claims.
- The prose-only transformation was checked by comparing parsed executable statements after removing docstrings. They were identical. The three moved display functions also had identical parsed function bodies before and after extraction.
- `yam.ui.session_status` now formats per-arm status, joint labels and tracking tables. The application imports those functions, preserving existing app-level callers. It makes no robot commands.
- `yam.recording_store.save_take` now owns slot publication and rollback. The application owns slot selection and occupied-slot confirmation.

### Failed recording saves

The previous handler replaced the frame directory before calling Trajectory.save. The new store serializes a candidate recording first, then temporarily removes the previous slot JSON before swapping frame directories. It publishes the new JSON last. In-process exceptions and interrupts restore the old frames before the old JSON. A failed save keeps the same take pending at the application's slot prompt so the operator can retry or choose a different slot.

A `.save-<slot>-<random>/` recovery directory holds candidate JSON, previous JSON/frames when moved, and `recovery.json` instructions. Failed rollback retains the directory and prevents another overwrite of that slot. Failure to delete old data after successful publication reports a cleanup warning instead of telling the operator that a successful save failed.

Limits: this is a single-writer file workflow, not a database transaction. It does not synchronize concurrent readers that cached old JSON, and does not claim power-loss durability (there is no filesystem flush protocol). Process termination can leave a slot temporarily absent and require recovery. Other tools still using the older standalone frame helpers do not automatically gain this behavior. At that checkpoint, saving and flushing still blocked the application loop. The recording-completion increment below moves those operations into workers.

Nine store tests inject serialization, publication, frame-move, interrupt and rollback failures in temporary directories. An additional test drives actual application keys through a failed save followed by retry into a different slot, checking that the same take survives and the run exits normally.

### What Julien's terminal run showed

The app terminal can be read through the terminal snapshot tool. It showed Julien selecting arms, trying GUIDE, entering TELEOP on both, and quitting via park. Both simulated arms reported seven disabled motors; the loop averaged about 83 Hz with a 105.8 ms worst pass around TELEOP entry. That timing is one observed simulator run, not a hardware benchmark.

The run exposed a real contradiction: ArmSession.enter_guide set mode to GUIDE before checking for the gravity-compensation API. The fake arm lacks that API, so the warning said HOLD while state and banners said GUIDE. The transition now falls back to actual HOLD when the API is missing, and only sets GUIDE after a successful API call. All three application banners name only arms actually in GUIDE. The existing ArmSession test now checks the mode; another actual-application test verifies that a refused GUIDE request prints no success banner.

The simulator is a terminal workflow with stationary fake SpaceMice. It does not open a 3D view or simulate hand-guiding/gravity. `?` lists keys; `q`, wait for the quit menu, then `q` parks and exits. The agent can run a private PTY instance, but the computer-control tool refused interaction with both Codex and macOS Terminal, so Julien starts his interactive run by pasting the launcher command. The agent's private instance was closed normally. Its isolated snapshot remains at `.local-backups/simulation-playground-2026-09-15`; it preserves the earlier source and is not a live view of subsequent edits.

Latest local logs use `agents/codex/validation/readability-*.txt`. `readability-preservation.json` again verifies all 3,455 original recording files unchanged, including after Julien's terminal exercise. The broad simulator passed 32/32 in another disposable copy and never wrote the user's recording slots. This continuation ends at a saved checkpoint, with no agent-owned simulator or background job left running.

## Recording completion and interface continuation

`RecordingSession` now owns the frame directory, per-take sink, final report and one pending save/discard operation. It retains the shared joint timeline and freezes it at the stop request. Camera readers remain session resources. The operator handles keys and presentation; background work never commands robots.

The lifecycle is:

1. Start a take and acquire an empty sink under its owner before starting individual writers. Partial startup retains every writer acquired so far.
2. Sample measured joints and offer fresh camera frames. Sampling or writer-startup failures freeze this take without changing arm modes. Failed takes remain available for explicit discard.
3. On stop, request draining without joining a thread. A writer creates its final index only after its image queue drains. `poll_stop()` returns a report only after its worker terminates, including the index write.
4. Permit publication only when every camera report has `flushed: true`. Missing or false completion is rejected before any old-slot mutation. Completion failure retains the files and prevents saving them as a complete take.
5. Run slot saving or directory deletion in one background job. Polling observes completion without waiting or doing filesystem work. Save failure keeps the same pending trajectory and frame directory for retry. Failed deletion retains ownership for another attempt.

While files finish, premature save/discard keys receive a waiting message. They do not select a future slot or queue deletion implicitly. `q` still reaches the ordinary quit flow. A requested discard, including a too-short take, waits for writer termination before deleting files. The UI confirms completed saves and deletions only after the job finishes.

After motor shutdown, recording cleanup allows a bounded one-second completion window. Unfinished or unsaved camera files are retained and the run returns failure with their path. It never deletes a directory behind a live writer. A save/discard already requested can finish during this window. If its daemon worker is still busy when the process exits, publication may be interrupted; existing store recovery rules still apply.

Limits: threads remove direct waits from the control loop but do not guarantee a hard timing deadline. CPU contention, Python scheduling and slow startup filesystem calls remain. There is no power-loss durability protocol or automatic recovery recording for unsaved joint samples. Retained images alone are not a recoverable full take. Physical camera delivery and robot response remain unverified.

The interface cleanup makes `FrameSource.names` a property, matching `CaptureSet`. Frame consumer contracts now describe nonblocking stop/poll, completion and failure behavior. Input documentation now matches `TwistReader`: axis state persists between HID reports and device errors reach the application. No input decoding or motion policy changed. A policy adapter still needs freshness and device integration rules.

Six new ownership tests use real writer threads with blocked encoders, failed index writes, blocked saves and failed deletions. The actual-application recording test file now has ten cases. It covers save retry, loop progress during blocked I/O, early-key refusal, partial startup, sampling/completion failure, quit retention, GUIDE refusal and TELEOP entry. The store test rejects unfinished or missing completion before touching the old slot. Interface tests check callable methods, the camera property and input behavior.

Current logs are `agents/codex/validation/completion-*.txt`; the simulator-copy file names its disposable checkout. `completion-preservation.json` verifies all 3,455 original recording files unchanged. These local logs are ignored; this section preserves their results and limits for future checkouts.

## Shared camera and display extraction

The operator and capture diagnostic now import acquisition services from `yam.cameras`; neither imports the viewer application. The viewer itself consumes the same library functions. Its public discovery/rendering function imports remain available for existing callers, while tests target the owning modules directly.

- `yam.cameras.discovery` owns device descriptions, name resolution, mode probes and index hints. The hint file still resolves to this repository's `config/camera_index_hint.json`.
- `yam.cameras.open.open_camera` uses the existing configuration function and releases a newly opened handle if configuration fails.
- `yam.cameras.session` owns the Linux/macOS recording-camera factories and their partial-startup cleanup.
- `yam.ui.camera_render` owns terminal geometry, image encoding and adaptive image sizing. It owns no capture devices.
- `capture_probe` now closes earlier cameras if a later open or reader construction fails. Duplicate requested camera names/indices refuse instead of overwriting an owned handle.

The operator shrank from 3,853 to 3,624 lines; the viewer from 1,891 to 895. The new discovery module is 411 lines, the camera-session module 228, and the rendering module 222. These boundaries separate shared device services from the application and display code. They do not merely divide the viewer into numbered fragments.

Seventeen discovery definitions, twelve rendering definitions and both recording-camera factories retained their executable statements. The comparison excluded docstrings, return annotations and import statements; imports and names were checked separately across the libraries and all three applications. Two existing resolver annotations were corrected to describe their actual three-value returns. Long explanations and measurements are preserved in [historical camera source notes](archive/camera-source-notes.md), with current contracts beside the code.

Nine additional tests cover configuration failure, diagnostic cleanup, duplicate indices, full-serial recording names, stale hints, ambiguous prefixes and dependency direction. The existing 62 camera rendering/discovery checks and ten factory failure checks pass against the extracted modules. Current suite: 928/928 across 50 files; falsifiers: 71/71; isolated simulation: 32/32. Both diagnostic help commands and the structural check pass. Local logs use `agents/codex/validation/camera-*`.

Identity limits remain explicit. A configuration-mode response is not proof of frame delivery. Same-model twins require physical identification. The existing macOS behavior warns and proceeds when no model-distinguishing mode exists; that policy was preserved and tested, not strengthened silently. Linux keeps its measured-delivery checks. A monochrome image alone does not establish depth data. No actual camera enumeration, stream, motor operation or terminal-image display was tested here.

Remaining blocking work includes per-take writer construction and some diagnostic/startup I/O. At this camera checkpoint playback and operator prompt coordination still lived in the main loop; the next section records the playback extraction. The successful extraction does not close those architectural items.


## Replay state and composite sequencing

`yam.playback_session.PlaybackSession` owns pending/active takes, their ordered layout, replay-start arrival credits, one shared cursor, speed/scrub mode and tracking history. `yam.composite.CompositeRun` owns validation of all take legs before starting, the remaining queue, selected pose arms and pose-arrival gates. The operator calls these owners rather than keeping parallel local state.

Arm commands remain in the application. Each cycle still measures in recording order, computes one shared target, commands each arm through the existing constraints, then records tracking. Both grippers are excluded by layout from the arm-lag gate. A fresh measured-pose comparison for every participating arm still occurs immediately before playback starts. The completed path's purpose is captured before advancing a composite: an arrival that starts the next take cannot also count as arrival at that take's start.

Two reset details are explicit: replacing a composite clears its old pose-arrival credits, and preparing a new take clears its old scrub choice. Cancellation discards pending replay arrival credits and queued composite work. Leaving replay on one arm puts every other replay arm into HOLD.

The operator is now 3,460 lines, down from 3,624 at the camera checkpoint and 5,055 before the readability work. The two new owners total about 200 lines. This is still an incomplete architectural cleanup: prompt decoding, physical command dispatch, replay plan/loading, measured start verification and report persistence remain in the operator. The owners do not introduce background robot control or establish hardware stopping performance.

Six composite tests and six playback tests cover validation before motion, arm ordering, all-arm arrival gates, cancellation, replacement, shared-clock lag behavior and tracking order. An additional actual-application test starts a two-arm replay and changes only the selected arm to HOLD, verifying both arms leave replay. The old source-name assertion was updated as supplementary evidence. The full suite passes 941/941 across 52 files, falsifiers 71/71, and the isolated interaction driver 32/32. Logs use `agents/codex/validation/playback-*`; the disposable simulator copy contains its own synthetic recordings.


## Explicit stop causes and controlled shutdown interaction

`yam.lifecycle.StopRequest` records a `StopCause` (quit, interrupt or fault) separately from its display message. Every application stop site supplies that category. Changing wording cannot turn a fault into an intentional quit. Incidents retain their string `stop_reason` and now also include `stop_cause`; a failed device cleanup still produces a failed run even after a planned quit.

`controlled_stop` owns the existing live-arm park/quit interaction. Planned quit keeps the menu. Interrupts and controlled faults attempt the existing guarded park when live arms have a base pose. A failed park returns to HOLD and the menu; it does not authorize release. Dead arms are omitted from motion, successful automatic parking does not print a dead-chain warning, and a second interrupt propagates into the outer cleanup. GUIDE banners name only arms that actually entered GUIDE.

The operator retains acquired handles, the actual `park_arms` command implementation, signal handling, motor-disable attempts, recording/camera completion and incident persistence. No hardware motion policy, calibration or limit has changed. The new module calls the supplied park action and never disables devices itself. This separation makes the interaction executable in isolation without hiding resource ownership inside it.

Six old tests copied a stopping expression or inspected source strings. Their six behavioral equivalents now execute the production flow, alongside seven additional cases for failed parks, menu return, GUIDE refusal, liveness loss, missing base poses and interruption. Three actual-application regressions verify that a fault containing the words `quit requested` still auto-parks and returns failure, a second interrupt still shuts down both fake arms before reporting an incident, and a renamed planned quit retains its menu and returns success. The full suite passes 959/959 across 54 files; isolated simulation passes 32/32. Source and documentation checks pass, and falsifiers retain 71/71 catches. Logs use `agents/codex/validation/stop-*`.

The structural checker now uses Python's symbol tables for global-name resolution. Its previous flat scan rejected a valid lambda parameter and could also let an unrelated function's local import or assignment hide a missing global. Eight tests check callbacks, closures, real imports, missing imports, sibling scopes, f-string reads and the actual application. This check does not prove that every local has been assigned before each runtime read.

The operator is now 3,372 physical lines, and the controlled-stop module is 111. Its remaining size is mostly prompt dispatch, configuration/plan assembly and cycle coordination, with further historical prose to shorten selectively. Moving all of that into one new runtime class would not complete the architecture. Next extractions should own transitions or resources, preserve command order and confirmations, and execute their application call sites in tests. No user decision is pending for that software work.
