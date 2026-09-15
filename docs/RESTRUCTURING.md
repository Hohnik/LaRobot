# Teleop architecture review

Reviewed September 15, 2026 against `c8069cc`. This document proposes the next changes; it does not claim they are implemented. [CLEANUP](CLEANUP.md) records completed work and validation.

## Decision

Continue with a staged restructuring across the operator, recording and camera code. The operator still holds too many independent responsibilities. Moving its remaining comments alone would leave a large, tightly coupled program.

Keep the working control stack and extract one owner at a time. `ArmSession`, `SafeRobot`, motion planning, IK and mirror calculations already provide useful boundaries. Replacing them wholesale would discard tested behavior without resolving the application's ownership problems.

The earlier cleanup was substantive. It fixed startup ownership, extracted trajectory state and slot persistence, and removed long historical notes from the operator. It did not finish the architecture. In particular, the new save boundary still accepts an unfinished camera report. That gap belongs in the next increment.

## What the lengths mean

These are physical lines at the reviewed commit. The code estimate excludes blank lines, standalone comments and docstrings; it is not a complexity score.

| File | Total lines | Approximate code lines | Assessment |
| --- | ---: | ---: | --- |
| `apps/teleop_session.py` | 3,854 | 2,626 | Several state machines and resource lifetimes need separate owners. |
| `apps/camera_view.py` | 1,891 | 973 | Shared camera setup and terminal rendering need separate homes. |
| `src/yam/robot.py` | 1,078 | 399 | Much of the length is explanation; trim that before splitting the wrapper. |
| `src/yam/session.py` | 980 | 421 | Per-arm ownership is useful; shorten historical prose and retain the boundary. |
| `src/yam/recording.py` | 803 | 322 | Mostly a documentation problem at this scale; split only around real consumers. |
| `src/yam/mirror.py` | 580 | 208 | Keep the cohesive algorithm and its essential constraints together. |

The operator's `main()` spans 3,098 lines. It contains 20 nested helpers and a roughly 989-line key dispatch block. Recording, playback, composite sequences, prompts and shutdown share local state through closures. That is the stronger reason to restructure.

A future entry point could be tens of lines: parse options, assemble resources, run the operator and return its status. Treat that as an illustration rather than a quota. Moving the entire function into `Runtime.run()` would leave the same problem. A cohesive several-hundred-line module can be easier to maintain than many tiny files that exchange mutable state.

## Findings that determine the order

### 1. A take needs ownership through camera completion

[RecordingSession](../src/yam/recording_session.py) owns the trajectory but not the whole take. The application separately owns its writer, frame directory, start timestamp and final report.

[FrameWriter.stop](../src/yam/cameras/writer.py) waits up to ten seconds per writer. It can return `flushed: false` while its thread remains alive. The application's `stop_take_frames()` then drops its writer reference and permits the pending take to proceed toward save or discard.

A temporary-directory probe passed an explicitly unfinished report to [save_take](../src/yam/recording_store.py). Publication succeeded, and the published camera metadata omitted the `flushed` field. The probe used fake file contents and no camera thread. It establishes missing completion validation; it does not reproduce an actual encoder race.

Two changes belong together:

- Retain ownership of the writer and its directory until completion is known. A take still being written must not be published or have its directory deleted.
- Move waiting and disk work out of the control-loop path. The loop should continue handling commands and guards while recording finishes.

Use explicit states such as recording, finishing, ready to save, saving and failed. Freeze the shared joint timeline immediately when recording stops. Poll for camera completion afterward. Preserve the take and its failure details when completion or persistence fails.

Long-lived camera readers belong to the session; per-take writers belong to the take owner. Background work must never command robots. Retain ownership of a stuck writer and report its failure. Never delete its directory while it can still write, or wait indefinitely before motor shutdown.

The existing store's rollback behavior remains useful. It protects replacement of an old slot against in-process failures. It does not establish writer completion, concurrent-reader consistency or power-loss durability.

### 2. Interface claims need behavioral checks

[FrameSource](../src/yam/seams.py) declares `names()` as a method; [CaptureSet](../src/yam/cameras/capture.py) implements a `names` property. The existing seam test checks attribute presence and misses this mismatch.

The command interface also promises zero values on an empty or dead source. A fake disconnected HID handle makes `TwistReader.read()` raise `OSError`. A fake report followed by no new reports retains its previous axis value. The application handles read errors itself.

Retaining an axis value between HID reports can be correct for a stateful device. Do not change that to zero on every empty read merely to match the prose. Define which layer owns failure handling, command age and neutral input. Then test that contract through the actual application adapter.

The claim that any six-number policy can enter the current loop with no other changes is too broad. Discovery, buttons, device ownership and input freshness also need integration. Correct the documentation and tests before using these protocols as a design foundation.

### 3. Camera setup crosses application boundaries

The operator's macOS camera factory imports helpers from `apps/camera_view.py`. `apps/capture_probe.py` also depends on that app. Discovery and opening are shared services, even though they currently live beside terminal-viewer code.

Extend the existing `yam.cameras.identity`, `open`, `specs` and `startup` modules where their responsibilities fit. Move the remaining shared discovery, hint resolution and model verification out of the viewer. Both applications should consume that library. Preserve serial-prefix resolution, full serial recording names, stale-hint checks and measured opening behavior.

Only then separate viewer rendering into `yam.ui`. Do not make a new library factory that imports the old application behind the scenes.

### 4. Playback and interaction are implicit state machines

Replay uses a shared cursor plus pending, ready, layout, speed and scrub state. Composite sequences add queue, arrival and purpose state. Nested helpers modify these alongside recording and prompt variables.

A playback coordinator should own these transitions. Keep one timeline for all arms, the park-to-start gate and the arrival-purpose ordering described in FINDINGS §72. The coordinator should provide the requested action for a cycle; existing arm and robot guards still apply.

Prompt state should also become explicit. Slot selection, overwrite confirmation, settings selection and quit confirmation currently share strings and flags. Separate key decoding from the action it requests. Keep the existing two-step motion confirmations and occupied-slot behavior.

Shutdown needs a typed cause as well as readable text. Current policy includes checking whether the reason string contains `quit requested`. Display wording should not determine how resources or motion are handled.

## Proposed responsibility map

Names below are suggestions. Introduce a file when its extraction has a concrete caller and tested behavior.

| Owner | Responsibility | Boundary to preserve |
| --- | --- | --- |
| `apps/teleop_session.py` | Parse, assemble, run, return status | No application state machine hidden in startup code. |
| `yam.operator.runtime` | Cycle ordering and coordination | Delegate transitions; do not accumulate every mutable field here. |
| `yam.operator.lifecycle` | Acquired resources, stop causes, ordered shutdown | Handle partial startup and attempt motor shutdown before slow peripheral cleanup. |
| `yam.operator.interaction` | Prompt states and keys translated into intents | Preserve selection and confirmation semantics. |
| `yam.operator.playback` | Shared replay cursor and composite handovers | Keep arm ordering and arrival gates explicit. |
| Existing `yam.recording_session` | Whole take lifecycle, including its writers | A frozen trajectory may still have unfinished camera work. |
| Existing `yam.recording_store` | Slot publication, rollback and recovery metadata | Accept only a take whose files are ready for publication. |
| Existing `yam.cameras` | Discovery, verified opening, capture and writing | Shared helpers must not import applications. |
| Existing `yam.ui` | Format operator and camera views | Ultimately render a supplied snapshot, without extra hardware reads. |
| Existing `ArmSession` and `SafeRobot` | Per-arm transitions and final command constraints | Keep shared recording and replay outside individual arms. |

Use existing settings support for validated configuration; extract application-specific parsing only as needed. Preserve cycle ordering during the initial moves. A later cycle snapshot should distinguish measured positions from requested positions. Current status formatting still performs position reads, so that separation is not already complete.

Avoid a generic event framework, a universal manager class or separate control threads for each arm. None is required to fix the observed coupling.

## Where information belongs

| Information | Home |
| --- | --- |
| Units, coordinate conventions, immediate hazards and non-obvious invariants | Beside the code that relies on them. |
| Inputs, outputs, exceptions, ownership and blocking behavior | Short function or class contract. |
| Rationale spanning several modules | A focused design section linked from the relevant boundary. |
| Hardware measurements, machine conditions and experiment outcomes | Dated findings or performance documentation. |
| Superseded explanations and session narrative | Historical archive or Git history. |
| Current launch instructions and implementation status | Existing entry documents, updated in place. |

For example, retain a short warning about shared replay timing beside the cursor update. Move the story of discovering that requirement to the findings. A reader should understand the rule without reading the full experiment history.

Apply this to touched modules rather than another indiscriminate archival pass. Some older docstrings claim behavior that implementations do not provide. Moving those claims unchanged into a design document would preserve the error. Mark historical claims as historical and verify current contracts.

`robot.py` may later benefit from moving pure pose helpers into existing motion code and separating calibration or thermal policy. Its roughly 400 code lines give little reason for an urgent multi-file split by themselves. The same caution applies to `recording.py` and `session.py`.

One smaller dependency cleanup is shared recording-slot resolution: `export_dataset.py` imports it from `export_episode.py`. Move shared resolution into the library when touching those tools. Preserve caller-specific precedence between real and simulated recordings.

Keep narrower demos and diagnostics when they serve a distinct purpose. Label their purpose in the entry documentation. A newer demo is not a replacement for the full operator until its required behavior is covered.

## Implementation sequence and acceptance

1. Finish take ownership and correct interface contracts. Add real application-path tests for unfinished writers, sampling/stop failures and retry. Verify that save and discard cannot race an owned writer. Verify control cycles continue while completion is pending.
2. Move shared camera setup into the existing camera library. Test discovery and failure cleanup with fake backends. Update all application callers together. Hardware identity and timing claims still require a separate physical check.
3. Extract replay and composite coordination. Characterize start, arrival, cancellation, scrubbing and next-leg transitions first. Preserve the common cursor and arm layout.
4. Extract interaction and lifecycle coordination, then thin the entry point. Use typed states and stop causes. Retain partial-startup coverage and the existing shutdown policy. Keep configuration persistence failures distinct from robot-control failures.
5. Shorten remaining documentation and clean up smaller dependencies as touched. Build recovery inspection tooling around retained save directories before offering automatic recovery actions.

Each increment should have one understandable owner and pass its relevant existing checks. Source-string checks must follow the new public boundary; do not disable them merely because code moved. Add behavior checks where attribute-presence or copied expressions currently give false confidence.

Success means a maintainer can locate a transition, see who owns its resources and test its failure without constructing unrelated subsystems. A smaller operator is a consequence. Line counts and a larger green test count are insufficient on their own.

## Evidence and limits of this review

This review inspected the current application, domain modules, camera code, contracts and tests. Local artifacts are `agents/codex/architecture-inventory.json` and `architecture-contract-probes.json`. They are ignored analysis files; the findings needed to resume are recorded above.

The probes used fake HID objects, an empty capture set and a disposable directory. They confirmed the contract mismatches and acceptance of an unfinished writer report. They did not operate hardware, start an encoder or measure control-loop timing.

The preceding implementation checkpoint passed 902/902 checks, 71/71 falsifier catches and 32/32 isolated simulation checks. Those validation results belong to the preceding checkpoint. The full suite was not rerun for this review. This review changes documentation only. No physical operation, remote push or team-branch replacement is part of it.
