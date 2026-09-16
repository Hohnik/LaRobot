# Teleop cleanup: current work and handoff

Updated September 16, 2026. This is the current implementation handoff for the cleanup branch. The August evidence remains in FINDINGS and the earlier sections of HANDOFF.

The [architecture review](RESTRUCTURING.md) records the responsibility map and remaining sequence. Recording completion, shared camera acquisition, camera rendering, replay state and composite sequencing are now extracted, as described below. The controlled stop interaction also has its own module. Settings, park, recording-save and playback prompts now own their key transitions. Startup plan formatting, acquired-device cleanup, incident-field assembly and command-line definitions are separate. Controls mapping edits, mirror confirmation, input polling and health checks also have owners. The application retains cross-mode key dispatch, startup assembly and the ordered motion-command cycle. Read-only recovery inspection is available; current source contracts replace another set of historical ArmSession notes.

## Status

The reviewed local cleanup is saved on `codex/teleop-cleanup` from Fable's final `499d0b7`. Completed work covers startup/shutdown ownership, shared recording state, readability, slot rollback, recording completion, shared camera/display extraction, replay state, composite sequencing, explicit shutdown causes, settings/park/recording/playback prompt ownership, startup plan formatting, acquired-device cleanup, incident assembly, CLI definitions, controls mapping edits, mirror coordination, input polling, health checks and recovery inspection. Current validation passes 1064/1064 checks across 68 files, 71/71 falsifier catches, and 32/32 isolated simulation checks. Structural and flag checks pass. No physical hardware has been operated and nothing has been pushed. All 3,455 original recording files still match the pre-switch hashes.

Correction to the earlier handoff: `c8069cc` was missing the operator's `effective_limits` import after the display extraction. The old structural log already reported that failure. A previous simulation log said 32/32, but it did not establish that the final saved source was valid. The earlier claim that the checkpoint was fully verified was wrong. This continuation restores the import and adds a direct application test that enters TELEOP on both arms. The current isolated simulation passes with that fix.

Julien supplied Fable's final message, which explicitly named `499d0b7`. The original restore was correct. He then authorized proceeding sensibly with the assessment's cleanup plan and requested detailed durable notes. The continuation rule and explicit completion assessment live in [AGENTS.md](../AGENTS.md) and [WORK_QUEUE.md](WORK_QUEUE.md). A test run or commit alone is not a reason to stop authorized work.

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

## Remaining boundaries and the next decisions

The operator is now 2,261 lines; ArmSession is 755. Neither number is a target to preserve. Independent state machines and acquired resources have owners; moving the remaining code merely to shorten the entry point would hide coordination without simplifying it.

- **Startup assembly stays visible:** parse/validate first, acquire inside the cleanup boundary, register returned handles before configuration, then initialize sessions. Resource cleanup already has a tested owner. A further builder abstraction should earn its place by reducing repeated configuration, not by passing every factory and local through a new context object.
- **Cross-mode dispatch stays ordered:** prompt consumption precedes ordinary keys; selection affects edits/mode changes; input polling precedes arm commands; composite arrival and playback start gates share one cycle. Moving these branches independently can silently change which key or cycle performs motion. Direct application tests now cover the important recent boundaries; this is still not a claim of exhaustive behavior coverage.
- **Motion calculation stays with its existing owners:** ArmSession, MirrorLink, CartesianTeleop and SafeRobot. Their physical behavior, limits and calibration were not redesigned. A future single command interface is a separate design change because modes currently command differently.
- **Recovery is inspection only:** use the commands below before deciding how to repair an actual interrupted save. No recovery evidence exists in the two current local recording roots. An automatic repair policy cannot be justified by file presence alone.
- **Team transfer remains a separate step:** re-read the team's refs before proposing particular transfers. The September fetch is a dated snapshot. Nothing has been pushed and the station has not been contacted.

The [architecture review](RESTRUCTURING.md#implementation-sequence-and-acceptance) preserves the earlier proposal. This continuation completes its selected local ownership/recovery work; the remaining entries above describe deliberate boundaries and future design choices, not a technical blocker or a request for Julien to repeat permission. Save verified checkpoints without treating each as a reason to interrupt authorized work.

The simulator does not validate USB/CAN timing, gravity compensation, camera delivery, gripper calibration, collision avoidance or real stopping behavior. Fake-device tests establish software call ordering and ownership, not that a motor physically disabled. Missing disable confirmations remain an operator-visible failure. Hardware validation and team publication require their own operator action/instruction.

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


## Settings and park prompts; startup plan formatting

Three application consumers now use small modules under `yam.ui`:

- [SettingsPanel](../src/yam/ui/settings_panel.py) owns the selected setting, a copied snapshot for revert, key interpretation and panel messages. It returns stay/close/quit. The operator still applies values to both live robots and the current mirror link, renders measured arm status, persists explicit saves and enters the existing quit flow. The first mode key closes settings; a second press is required to change mode. Revert uses the effective session-start values after defaults and flags, not built-ins. Saving is still synchronous and save exceptions still reach the outer cleanup; this extraction does not add background settings persistence or a new failure policy.
- [ParkPrompt](../src/yam/ui/park_prompt.py) owns entered pose/take legs, the pending recording prefix and the second-confirmation state. It returns immutable choices. A single pose (or empty/base choice) runs on the first acceptance; multiple legs still require a second acceptance. Speed, corners and easing keys remain intercepted by the operator before prompt decoding, so they can be changed without starting the run. The operator resolves each selected arm's own slots and routes take legs through the existing composite validation. No file lookup or robot command occurs in the prompt owner.
- [session_plan_lines](../src/yam/ui/session_plan.py) formats resolved settings, maps and saved poses. The application still loads configuration, prints the plan and saved-default provenance, performs any explicitly requested defaults save, then handles dry-run exit before acquiring devices. Five dry runs matched the previous stdout, stderr and exit code exactly: ordinary defaults, two simulated arms, no gripper/no rotation, custom limits/feedforward and a tool-frame configuration. This verifies presentation compatibility, not physical operation.

### Demonstrated stale-prefix defect

Before extraction, `park_take_next` was separate from the entered sequence and was not cleared on cancel or reopen. The actual-application test reproduced `p`, `w`, cancel (`x`), then `p 1 Enter` interpreting the digit as **recording 1** instead of **pose 1**. The temporary test setup had no recording 1, so the application refused it; an existing compatible take could instead have entered its park-to-start flow. `ParkPrompt.open()` and every completed/cancelled choice now clear the marker and confirmation together. The same test now invokes arm B's pose-1 path and starts no composite. The existing behavior of accepting an unfinished `w` with no digit as the empty/base selection is preserved; this change does not redefine that key sequence.

### Verification and remaining scope

Eight settings-panel tests cover selection without mutation, bounded adjustment through the existing ladder, apply-before-status ordering, copied revert values, consumed mode keys, explicit save/failure reporting, quit and named unknown keys. Eight park-prompt tests cover first/second confirmation, one-use take prefixes, cancellation/reopening, immutable choices and existing base behavior. Five additional application tests verify changes/revert reach both live fake robots, the two-press settings-to-mode transition, temporary settings persistence, stale-prefix isolation and second confirmation before starting a multi-pose path.

The save-retry application test also lost a timing assumption: retry and quit now follow the RecordingSession's observed error/completion state. An independently launched test run had exposed that fixed cycle numbers could send retry before the worker completed. The full suite passed before that correction, but the separate failure was real test instability; the final suite passes after the state-based correction.

Final software evidence: 980/980 checks across 56 files, 71/71 falsifier catches, structural/flag/prose/link checks, five matching dry-run outputs, and 32/32 isolated simulator interactions. All original recording hashes still match. Logs use `agents/codex/validation/prompts-*`; the simulator-copy file identifies its disposable data directory. No real hardware was opened and no changes were pushed.

The operator is 3,149 lines, down from 3,372 at `5e0fa0b`; the settings, park-prompt and plan modules are 86, 67 and 88 lines. This is a reduction in shared state and duplicated prompt branches, not completion of the entire restructure. Historical explanations removed from these blocks remain in `5e0fa0b` and their cited FINDINGS sections. The remaining single key-dispatch loop still coordinates other modal prompts, per-arm actions and shared modes.

`checks/check_flags.py::read_parser` currently reads `add_argument` calls from each application's source; it does not follow an imported parser builder. Moving argument definitions without adapting that check would break its evidence. Preserve the existing deliberately-invalid-command checks when making that later extraction. No additional user decision is needed for these software steps.


## Recording-save prompt ownership

[RecordingPrompt](../src/yam/ui/recording_prompt.py) now owns slot choice, overwrite confirmation, the saved take's display summary and progress messages for asynchronous save/discard. It uses explicit choosing/replacing/saving/discarding/closed states. `RecordingSession` continues to own joint samples, camera writers, frame directories and disk jobs. The prompt observes completion once per control cycle; it never commands an arm or performs publication itself. Slot description still reads the selected JSON synchronously, as before; this is not a claim that every filesystem operation has left the control loop.

The application opens the prompt after manual recording stop, the sample limit, writer startup failure or sampling failure. It asks the same controller to discard too-short takes. Opening a recording prompt still cancels gripper-button learning, and messages still pass through the application's status-line renderer. These cross-cutting call sites matter: extracting key handling alone would have missed both interactions. The original `save_slot_action` remains imported by the application for compatibility; its existing tests now import the owning module directly.

Preserved interaction rules:

- A free slot saves immediately. An occupied slot requires the same digit again. A different digit selects a different slot; an occupied new target needs its own confirmation.
- A non-digit at the slot-choice prompt, including `q`, discards the **new** take and retains the old slot. This is the established interaction, not a newly unified meaning for `q`.
- While files are busy, digit and discard keys are consumed without queuing an action. `q` closes the prompt and reaches the existing quit flow. Closing that prompt does not cancel the disk job or release its resources; recording shutdown still owns completion/retention.
- Failure to start saving or an asynchronous save/discard failure returns to slot choice. The same take remains owned, and any subsequent overwrite requires fresh confirmation. No success is printed before completion; the saved duration/sample count is captured before the recording owner releases the take.

Ten new controller tests cover those transitions using controlled completion states. Two additional actual-application tests verify that changing from occupied slot 4 to occupied slot 5 leaves both byte-for-byte unchanged until a second 5, and that opening the recording prompt cancels button learning. The overwrite test also checks that feedback uses the status-line renderer. The existing application tests continue to cover loop progress during blocked disk work, early-key refusal, same-take retry, camera failure and quit retention. Both the earlier save-retry test and the new overwrite test follow observed completion rather than assuming the worker finishes within a fixed cycle count.

The operator is now 3,046 lines, down from 3,149 at `b516dc7`; the new controller is 148 lines. Total code need not shrink when explicit interfaces replace shared local state. This increment gives recording-save interaction one owner and executable boundaries. It does not complete application assembly, physical command coordination, playback/mirror confirmation, controls editing or acquired-device cleanup.

Validation: 992/992 checks across 57 files, 71/71 falsifier catches and 32/32 isolated simulator interactions. All 3,455 original recording hashes match. Structural, flag, link and prose checks pass. The results are recorded in the local `agents/codex/validation/recording-prompt-*` logs. The simulator-copy file identifies its disposable data directory; these logs are ignored, so this section carries the durable conclusions. No real hardware was opened and no changes were pushed.

### Assessment of what still deserves work

The remaining work is substantive, but file length alone is no longer a useful finish line. Acquired-device cleanup still combines raw handles, fully initialized arm sessions and final reporting in the application. Its next extraction must retain motor-first ordering and cover partially initialized devices. Playback/mirror prompts and controls editing still own several transitions inside the key loop. Argument assembly can move once the source-based flag checker follows its new owner; merely hiding `main()` inside a large class would leave the same coupling.

Recovery inspection is a separate functional gap: retained camera directories are not complete recovered takes when joint samples died with the process. Inspection should report what actually exists and the slot store's recovery metadata before offering any repair. Physical station validation and transfer to the team's code remain separate, operator-led work. No user decision is needed for the remaining local software cleanup; there is no reason to rewrite the established motion or kinematics algorithms just to make their files shorter.


## Acquired resources, incident assembly, CLI and playback continuation

Julien asked why work was stopping at checkpoints despite having no pending question, and why the operator was still around 3,000 lines. A green checkpoint was not a blocker. This continuation covered several connected boundaries before the final integrated validation. There is no architectural requirement for a 3,000-line operator: the remaining size reflects unfinished separation plus historical explanation. The first count in this pass found 741 comment-only lines and 145 blank lines. Current size is 2,556 lines, including 500 comment-only and 140 blank lines; the other 1,916 include executable statements, help strings and docstrings, so that is not a pure code-line count.

### Acquired handles

[SessionResources](../src/yam/session_resources.py) owns every returned robot, physical puck and capture set. Registration happens before robot configuration/session construction and before puck configuration/reader construction. Initialized `ArmSession` objects remain separate because a failed constructor must not hide an enabled handle. Each robot registration retains its expected motor IDs; shutdown confirmation is checked against those IDs.

Cleanup attempts all robots first, then inputs, recording completion and camera readers. Ordinary failures are reported and do not skip later resources. The result makes even an intentional quit return failure when motor-disable confirmation is incomplete. Completed repeated closes reuse the result. Register resources during acquisition, before closing this owner. `KeyboardInterrupt` and `SystemExit` during cleanup keep their existing process-level behavior; this change does not add a new interruption policy. Actual parking, process signal hooks, map persistence and final report invocation remain in the application.

Five owner tests verify order, independent failures, per-handle motor IDs, repeated close and camera-only startup. The existing 16 application lifecycle tests still exercise constructor/read/configuration failures and process-hook restoration. An additional actual-application case performs a planned quit, withholds one motor's confirmation after fake shutdown, and verifies failure status while both fake robots still receive shutdown.

### Incident data and test isolation

`yam.incident.session_facts` assembles the existing incident schema. Liveness still comes from before cleanup; loop locals are passed as deferred reads so failed startup cannot raise while binding a missing timer. Cached arm facts remain individually guarded. Incident persistence stays after resource cleanup. The old source-string ordering test now executes the real application: its fake robots become non-live during shutdown, while the emitted incident retains `chain_alive_at_teardown: true` for both.

Inspection also found that `tests/test_session_recording_failures.py` isolated trajectories and configuration but had left the incident module's default output directory and USB enumeration unpatched. Its failure cases could therefore create new incident artifacts in the checkout and inspect the host USB inventory. The harness now directs incident files into its disposable directory and supplies an empty fake USB snapshot. This correction does not change production incident capture. Pre-existing ignored artifacts have not been silently deleted; the 3,455 original recording files still match the backup hashes.

### CLI definitions and their checker

[yam.teleop_cli](../src/yam/teleop_cli.py) owns the argument definitions. The application supplies its three coordination defaults (linear scale, gripper step and planned speed), installs saved defaults, validates cross-option constraints and chooses arms before acquiring devices. No default value, flag, help text or precedence changed. Twelve subprocess comparisons match stdout, stderr and exit status exactly: help; ordinary and two-arm dry runs; no-gripper/no-rotation; adjusted limits/feedforward; tool frame; unknown arm; invalid plural arms; retired box flag; simulated cameras; contradictory map flags; and two-arm GUIDE refusal.

The flag checker reads directly called imported `yam.*.build_parser` definitions, including aliases, without importing or executing code. Local argument definitions still win. Missing, syntactically invalid and cyclic builder sources remain known parsers with no accepted flags, so documented options fail rather than escaping validation. This is an explicit static convention, not arbitrary Python factory evaluation. Seven new checker tests exercise that boundary and the existing deliberately broken command cases still run in the falsifier suite.

### Playback confirmation and historical explanation

[PlaybackPrompt](../src/yam/ui/playback_prompt.py) owns slot selection, speed preview, plan formatting, cancellation and the explicit choice between ordinary replay and puck scrubbing. Its start result requests the application's existing park-to-start action. It never commands robots; arrival bookkeeping and fresh measured-pose checks still gate actual replay. Speed keys retain the 0.05 floor and existing ceiling; `q` at the preview cancels and consumes that key. Seven prompt tests and an actual-application test verify that selection/speed adjustment do not start a path. Existing two-arm replay/cancellation tests still execute the application.

Twenty-six long historical comment blocks were moved to [the source-note archive](archive/teleop-source-notes.md#remaining-long-operator-comments--september-16-2026), leaving concise current contracts. Python ASTs matched exactly across that comment-only pass. The old claim that CONTROLS does not move motors was historical and contradicted current behavior; the replacement states that puck deflection drives the isolated motion at reduced speed while keys edit the map. Incident history removed from its assembly block is preserved there too.

Validation: 1,013/1,013 checks across 60 files, 71/71 falsifier catches, 32/32 isolated simulator interactions, structural/flag/link/prose checks, twelve identical CLI results and 3,455 unchanged original recording hashes. Local evidence uses `agents/codex/validation/resources-*` and `cli-*`; the simulator-copy file identifies the disposable checkout. No motor or camera device was operated and no change was pushed. The remaining work is listed above rather than implied complete by the line-count reduction.


## Controls editing continuation

After `53dece4` passed integrated validation, work continued into the controls branch rather than ending at that checkpoint. [edit_controls](../src/yam/ui/controls_editor.py) now handles explicit mapping keys (`f`, `1`–`6`, `u`, `0`) for the wizard's arm. `ArmSession` retains the map, initial snapshot and observed puck axis. The helper performs no robot command or mode transition; mode changes, shared speed keys, button-specific reversal, input sampling and reduced-speed motion remain in their existing application order.

The first production-handler test exposed an existing mismatch between behavior and instructions: after moving puck axis X from motion 1 to motion 2, pressing 2 again swaps motion 2 with itself. It does not undo the earlier exchange, although the help and feedback claimed it did. The underlying map transformation is preserved. Feedback now names the original motion key (1 in that example) to swap the pair back, and selecting the already-driven motion reports unchanged rather than claiming an exchange. Current keyboard help and COMMANDS agree. Earlier historical statements in the source-note archive describe the old mistaken claim.

Six handler tests exercise permuted-axis reversal, paired exchange and actual reversal, unbind/rebind direction, copied revert snapshots, missing observed axes and non-edit key pass-through. An additional application test forks the maps in a temporary configuration, enters controls on B, edits and reverts, swaps 1→2, verifies a repeated 2 leaves that swap intact, then uses 1 to restore it; G's map remains unchanged throughout. Existing shared-map behavior is not changed by this test's deliberate fork.

Current evidence: 1,020/1,020 checks across 61 files, 71/71 falsifier catches, 32/32 isolated simulator checks, structural/flag/link/prose checks and unchanged original recording hashes. The operator is 2,512 lines and the editing helper is 65 lines. Logs use `agents/codex/validation/controls-*`. Neither this extraction nor the reduced line count means all keyboard coordination is finished.


## Mirror, input and health coordination

This continuation follows `783c8f1` (operator 2,512 lines). The operator is now 2,251 lines and main spans 1,750. It has not been moved wholesale into a new class.

| Owner | Responsibility | Stays in the operator |
| --- | --- | --- |
| [MirrorSession](../src/yam/mirror_session.py) | Pair preview, copy/reflection choice, confirmed engagement, cancellation and stop diagnostics | Pose reads, MirrorLink.step, jaw constraint and actual follower commands |
| [SessionInput](../src/yam/session_input.py) | One shared-reader drain per cycle, selection routing, per-arm reads, button edges/learning and jaw target edits | Input service placement before mode commands; handle acquisition/cleanup |
| [SessionHealth](../src/yam/session_health.py) | Liveness/thermal stop requests and stall reporting/latching | Controlled stop and final cleanup ordering |
| [ArmSession](../src/yam/session.py) | Single thermal read, current measurement cache and gripper-stall decision | Session-wide coordination |

Mirror confirmation still enables follower position control before setting its mode to mirror. It takes the follower's actual speed cap and current clipping-counter baseline. Preview/toggle alone never commands a robot. Leaving the follower's mode clears the link. Six owner tests and an actual application test exercise preview, toggle, confirmation and mode exit.

The old operator duplicated ArmSession's thermal/stall logic while its tested helper methods were unused. SessionHealth now calls those methods. A read failure remains blind, preserves the last temperature list for incident reporting and resets the stall timer; current states/hottest/jaw are unavailable. The public states and legacy _states reference the same successful read. A latched jaw does not restart stall pushing. Thresholds are supplied unchanged by the operator. Six tests use real ArmSession objects with fake chains, including missing readings, six-joint arms, latches and one overheated arm in a two-arm session.

A temporary extraction error reset a pending puck-failure stop at the next health pass. The new actual-application failure test caught it before commit: the session ran past the intended stop. SessionHealth now accepts and preserves the pending StopRequest; the app test verifies fault status, parking before both shutdown calls and absence of an uncontrolled loop exit. This was an introduced, uncommitted extraction regression, not a newly discovered defect in `783c8f1`. The old source-string test was replaced by behavior evidence. The shared-puck source test now also drives real application selection keys and verifies exactly one read per pass through B, G and BOTH. Six direct input tests additionally cover failures, edge learning and jaw bounds.

The structural checker still forbids moved state as bare operator locals. For fields consumed by the new services, it now requires an attribute access in the declared owner source and the operator's service call. A regression removes each in turn. This is a limited static check, not proof of runtime control flow; the application tests supply the stronger integration evidence.

## Read-only recording recovery inspection

Run from the repository root:

```sh
.venv-teleop/bin/python checks/check_recording_recovery.py --dir recordings
.venv-teleop/bin/python checks/check_recording_recovery.py --dir recordings/sim
```

Add `--json` for structured evidence. Exit 0 means no interrupted-save/unclaimed-frame evidence in that one directory, 1 means inspection is needed, and 2 means the directory could not be inspected. This is not a camera-quality or trajectory-validity certificate. Stop recording before making recovery decisions; the report is a snapshot, not a lock.

[recording_recovery](../src/yam/recording_recovery.py) reports published, candidate and previous JSON separately, plus current/previous frame directories. It counts images/indexes without decoding images or following symlinks. Recovery manifest paths are displayed but never used to locate arbitrary files. Invalid manifests do not suppress the remaining candidate/previous evidence. It cannot infer matching JSON/image provenance, durable writes or unsaved in-memory joints. It never repairs, renames or deletes anything.

The older recording checker said unclaimed frames were safe to delete and returned early when no JSON existed. It now reports retained evidence even without published slots, identifies staging directories and directs the reader to inspection. The unconditional deletion advice is removed. Nine tests include actual store publication/rollback failures, post-publication cleanup failure, malformed metadata, symlinks, frame-only evidence, directory errors and CLI behavior. Byte content and modification-time snapshots verify read-only inspection in the failed-rollback and CLI cases.

On September 16, local inspection found nine published JSON files under recordings and two under recordings/sim, with no interrupted-save or unclaimed/pending-frame evidence. This is a dated local finding; re-run inspection after a failure rather than treating these counts as permanent.

## Source contracts and evidence for this continuation

Sixteen ArmSession passages moved verbatim into [the source-note archive](archive/session-source-notes.md). Current docstrings explain the actual owners, selection including a shared puck, the base-pose distinction, blind thermal state, jaw latch, path splitting, measured arrival and jaw settlement. The previous class docstring's claim that map state was absent was obsolete. Historical incident narratives remain available without contradicting the current contracts. This prose pass reduced ArmSession from 991 to 755 lines; parsed executable statements were identical after removing docstrings. Motion algorithms were not altered by it.

Validation: 1,049/1,049 checks across 65 files, 71/71 falsifier catches and 32/32 isolated simulator interactions. Structural, flag, link and prose checks pass with the existing ceilings. All 3,455 original recording files still match the pre-switch SHA-256 manifest. Logs use `agents/codex/validation/coordination-*`, `recovery-*` and `session-prose-equivalence.txt`; the simulator-copy file names its disposable checkout. An early suite run exposed the now-replaced shared-puck source assertion and a temporary malformed test fixture; the final suite, not that intermediate run, is the evidence above.

No physical robot/camera session, station connection, change to calibration/limits or remote push occurred. The saved data, original training branch/environment and Fable close-out refs remain preserved.


## Continuation rule, export dependency and live configuration failures

Julien again identified an unnecessary stop after `190070e`. There was no technical blocker. A passing checkpoint had been substituted for completion of the broader task. The repository now has [persistent agent instructions](../AGENTS.md) and a [work queue](WORK_QUEUE.md): after each increment, proceed to the next authorized useful item; before ending, inspect all unfinished items; stop for actual completion, explicit user pause, or a concrete dependency. Do not relabel work as optional to finish a turn. These are instructions, not an automatic scheduler, and cannot guarantee resumption after an application interruption or usage limit. No recurring job or separate task was created.

### Export selection now belongs to the library

The earlier review explicitly identified an application-to-application dependency that was still outstanding. `export_dataset.py` imported `find_slot` from `export_episode.py` and modified sys.path to do it. Both applications now call [find_export_slot](../src/yam/recording_slots.py) with their own recording root. The batch inventory uses that same policy silently before actual export.

Real recordings still win over simulated ones, including a frameless real recording shadowing a simulated recording with frames. Simulation-only selection warns and preserves its metadata; missing slots retain the existing CLI refusal. The operator's simulation playback deliberately prefers its simulated slot and is unchanged. Seven tests cover the resolver and both real main functions; export encoders are stubbed in these selection tests, while existing episode/dataset tests still exercise their real outputs. No user recording was exported or changed by these tests.

### Configuration errors remain local to saving

The review also found that live settings-save errors escaped the control loop into direct cleanup. Pose saving updated the in-memory base pose and announced success before writing the file. These contradicted the review's requirement to keep persistence failures separate from robot-control failures.

SettingsPanel now catches only filesystem errors from its save callback, reports NOT SAVED, retains live values and remains open for retry/close/quit. Errors from device application and non-filesystem programmer errors are not swallowed. Pose saving collects selected arms' measured poses, publishes the file, and only then updates slots/base destinations and announces success. A filesystem failure retains every previous base/slot, reports failure and keeps the digit prompt open for retry or cancellation. It does not automatically park or disable healthy arms because a configuration write failed.

[write_json_atomic](../src/yam/files.py) serializes first, writes a candidate in the destination directory, closes it, then replaces the target. Settings defaults and saved poses use it. Partial writes and failed replacement leave the previous file intact; failed candidates are cleaned up, existing permissions are preserved, and existing symlink targets remain the write destination. This is a single-writer filesystem operation with no fsync/power-loss durability guarantee. It does not make synchronous configuration writes nonblocking, and does not change recording-slot publication or map-save behavior after shutdown.

Two actual-application tests inject settings/pose save failures and verify subsequent live cycles, retry, no false success, and unchanged prior base poses/files. The initial settings test fixture incorrectly assumed a defaults file existed; it was corrected to create a temporary one, and the old SettingsPanel from `190070e` then reproduced the real failure with the injected error confirmed. The pose test reproduced the existing failure directly. Six file-publication tests cover serialization, partial writes, replacement failure, interruption, permissions and symlink semantics. The old settings-panel unit test expected propagation; it now checks containment and a successful retry.

Current validation: 1,064/1,064 checks across 68 files. The isolated simulator passes 32/32, falsifiers catch 71/71, structural/flag/link/prose checks pass, 12 CLI outputs and exit codes match the earlier baseline, and all 3,455 original recording hashes remain unchanged. The completed queue records the scope and design disposition. Local evidence uses `agents/codex/validation/continuation-*`, `config-failure-before.txt` and `config-settings-before-corrected.txt`. The first settings fixture failure is not evidence of a production defect; the corrected reproduction is.
