# Teleop cleanup work queue

Updated September 16, 2026. This queue implements Julien's instruction to continue
through useful local work without stopping at each commit. The initial state below
is based on verified checkout `190070e`; subsequent evidence is recorded per item.

## Active phase: make the colleague handoff easy to use

After the attended checks passed, Julien explicitly asked for a fresh readability
review against the teammate's current implementation. The reference should be
understandable and useful as inspiration for their own arm/features implementation.
This extends the completed cleanup and validation; do not stop at the earlier
package checkpoint while these newly requested items remain actionable.

1. Capture source and revision provenance. Complete: clean station physical source
   at fd2c64b, matching the pushed tip. Six current remote branch tips fetched and
   their source captured with hashes. The teammate's checkout was not modified.
2. Compare current implementations. Complete: BRIDGE now distinguishes physical,
   recording, training and ABC work. Actual physical cleanup failures reproduced
   with fake dependencies; obsolete input calls fail signature binding. Two samples
   from the actual team recorder produce zero frames in its current training reader.
   The August comparison is preserved as explicitly historical material.
3. Improve concrete readability problems. Complete: IK 529 to 308 lines, ArmSession
   755 to 692. Historical explanations archived; model/measurement, GUIDE displacement
   and Frame contracts corrected. Existing executable statements are unchanged.
   The operator keeps its tested ordering; no additional large-loop move is justified.
4. Prepare a feature reading map and examples. Complete: COLLEAGUE_HANDOFF pairs
   features with owners/tests and explains reuse versus rig policy. Two hardware-free
   examples execute actual command limiting, cleanup, recording writers and publication.
   Both initial runs pass; final validation follows below.
5. Verify, commit locally and regenerate the review package. In progress. Sending/pushing to a
   colleague or changing their live checkout still requires a concrete destination
   and instruction; a readiness question alone does not authorize publication.

No answer from Julien is needed for these read-only comparisons and local changes.

## Attended physical-station validation — completed

Julien selected physical-station verification on September 16 and is there with a
teammate who is currently working on it. This is new work after the completed local
cleanup. The station is reachable as yam-pc; no login details are needed from him.

1. Inspect station checkout/runtime without modifying existing work. Complete:
   reference checkout remains at 499d0b7 with changed park poses; team checkout is
   on feature/physical with uncommitted work. Python/lockfile/vendor revision match
   the requirements. Current CAN interfaces are up; that alone proves no motor health.
2. Import candidate 025fab1 into a disposable station checkout and run its software
   checks, simulator and read-only inventory. Complete: 1,075/1,075 candidate tests,
   32/32 simulator interactions and read-only inventory pass. A missing dataset
   fixture was repaired; 71/71 falsifier catches and two new regressions pass.
   Neither existing checkout is switched or merged.
3. Prepare a separate operator checkout with the station's current configuration,
   review its dry-run plan and write exact single-arm HOLD/exit instructions.
   Complete: separate operator copy, B/G plans and logged helper dry run verified.
4. Perform attended real-device checks when the teammate yields the selected arm.
   Both arms are available, but Julien explicitly wants to run the first commands
   himself after explanation. B, two G runs and the combined-arm run are complete.
   All four logs end with exit 0 and disabled motors. Julien reports that both
   arms felt normal and the GUIDE movement was deliberate. Combined B/G/BOTH
   shared-puck selection and parking were exercised. Julien explicitly confirms
   selection moved the intended arms without unexpected movement or resistance.
   Target-lead warnings are explained with their measurement limits. The agent has
   sent no motor command.
5. Record results, limits and preserved state. Complete: all five live logs saved
   locally. The post-recording snapshot confirms reference checkout and operator
   config remain unchanged. The full take and supporting evidence are also copied
   outside /tmp on the station; all 380 copied files match their checksums.
6. Verify one camera-backed take. Complete: 11.88 s, 1,166 finite/ordered samples of
   14 joints, 357 readable 1280×720 images at 30 fps. Index/counts/host timestamps
   agree, no writer losses/errors or interrupted save. Julien also replayed the
   take successfully into HOLD. The 1.35 s stationary ending exposed a misleading
   checker diagnosis; wording corrected and verified by 76 recording tests, nine
   recovery tests and the actual checker on the take. Useful camera framing and
   the Logitech connection still need physical setup before demonstration capture;
   the completed test establishes the pipeline with the available D405 view.
7. Improve command handoff and prepare colleague review. Complete: the same short command
   now works on the Mac and station. Two misleading shared-puck help messages are
   corrected locally; the full suite passes 1,077/1,077. README now includes the
   missing pinned I2RT checkout step. The concise colleague handoff is written.
   The source archive matches every tracked file's path, content and executable
   mode, and its extracted help and simulation plan succeed. The incremental Git
   bundle verifies with prerequisite 499d0b7. Their exact revision, sizes and hashes are
   recorded under agents/codex/station-validation-2026-09-16/handoff/MANIFEST.json.
   Publication and promotion into an existing working checkout remain separate
   decisions; no colleague message or push is authorized by a readiness question.

Evidence: [STATION_VALIDATION](STATION_VALIDATION.md). Connection and folder map:
[STATION_COMMANDS](STATION_COMMANDS.md). No further motion is needed for this
sequence. The new colleague-readability phase above is actionable independently.

## Broader review — completed locally

Julien explicitly confirmed continuation after the `216fda0` queue proved too narrow.
All four agreed review items have now been resolved through changes or specific
code-based retention reasons; the detailed evidence is in
[CLEANUP](CLEANUP.md#broader-review-close-out--september-16-2026).

| Item | Disposition and evidence |
| --- | --- |
| robot.py / recording.py explanations | Complete: 40 passages archived verbatim, stale export/platform/validation claims corrected, executable AST unchanged. Files now 673 and 615 lines. |
| Status acquisition/rendering | Complete: immutable value snapshots used by both app callers; one pose read per arm, mirror reuses it. Five new tests plus existing content tests; read failures retain fault cleanup. |
| Operator startup / cross-mode dispatch | Complete review: duplicated speed/rotation policy extracted into DriveControls and tested through real callers and nonzero simulated input. Startup keeps acquisition order; loop keeps multi-arm arrival/key/command order. Specific rationale in CLEANUP. Operator now 1,974 lines. |
| Current status / detailed handoff / preservation / save | Complete: current summary rewritten, nuanced evidence retained; 1,075/1,075 checks, 71/71 catches, 32/32 isolated interactions, CLI/checkers pass, all 3,455 original recording hashes unchanged. This local close-out commit contains code/tests/docs together. |

No item in this broader pass remains waiting for another "continue". No answer is
needed from Julien. Hardware/station validation and publication are reserved below,
not unexplained software blockers. No background work or automatic resumption is
running. Do not invent more cleanup to avoid a legitimate completion; reopen this
queue for a concrete defect, requested change or new evidence.

## Previous checkpoint objective

The `216fda0` checkpoint completed its concrete implementation queue. Its narrower
completion assessment did not cover the broader review now listed above.

## Previous checkpoint work items

1. **Persistent continuation rule:** add repository instructions and this queue;
   use the next item after a commit instead of ending merely because checks pass.
   Status: complete. AGENTS.md exists at the repository root, links resolve, and this run follows the queue. Automatic loading in a fresh app session has not been independently tested.
2. **Shared recording-slot resolution:** remove the exporter's application-to-application
   dependency identified in RESTRUCTURING. Preserve real/simulation precedence and
   refusal behavior; exercise both actual callers. Status: complete. Both export main functions use yam.recording_slots; seven tests cover selection, warnings and refusal.
3. **Contain live configuration-save failures:** review found that settings-write
   errors escape the live loop and pose-save success/base state precede publication.
   Reproduce, preserve previous files on failure, and let the operator retry without
   a file error becoming a robot-control failure. Status: complete. Two application tests and six file-publication tests pass; previous files and live base state survive failure.
4. **Close the architecture review against current code:** account for every planned
   boundary, distinguish implemented work from justified retained coordination,
   and identify any remaining useful authorized work before ending. Status: complete; disposition below.
5. **Final verification and durable handoff:** relevant checks, local save, clean
   status and an explicit completion/dependency assessment. Status: complete; results below.

## Already completed

`190070e` and its predecessors implement resource cleanup, recording completion and
publication rollback, camera services/rendering, replay/composite ownership, prompt
owners, typed stopping, input/health/mirror services and recovery inspection.
Detailed evidence and limitations are in [CLEANUP](CLEANUP.md). Do not repeat these
changes or their entire test matrix without a new reason.

## Reserved decisions

Physical validation, changes to limits/calibration and remote publication remain
outside this cleanup. They do not block independent local software work. There is
no requirement to reach a particular line count, and moving an entire remaining
loop into a new class does not itself resolve coupling.


## Previous checkpoint architecture disposition

The table records the assessment at `216fda0`; the broader review above supersedes
its source/status/dispatch conclusions.

| Planned work | Current result | Evidence / retained boundary |
| --- | --- | --- |
| Recording ownership and contracts | Implemented | RecordingSession owns writers and disk jobs; actual-loop tests cover incomplete flush, sampling failure, retry and shutdown retention. |
| Shared camera setup and rendering | Implemented | Both apps call yam.cameras.session; rendering has a separate UI module; startup/partial-failure tests use fake backends. |
| Replay and composite coordination | Implemented | PlaybackSession and CompositeRun own timeline and sequence state; app retains measured arrival/start gates and actual commands. |
| Interaction and lifecycle | Implemented for independent owners | Prompt owners, controls editor, MirrorSession, SessionResources and typed controlled_stop are called by the operator. Cross-mode dispatch remains ordered in one place. |
| Input and health cycle services | Implemented | Shared input drains once; pending faults survive next-pass health checks; existing ArmSession guards replace duplicate inline decisions. |
| Live persistence failures | Fixed in this continuation | Filesystem failure remains a save failure; published files and base poses change only after success. |
| Recovery inspection | Implemented | Read-only report distinguishes retained images, candidate and previous trajectories; it never infers permission to delete or auto-repair. |
| Shared export-slot policy | Implemented in this continuation | Both real main functions use the library; simulation warning and real-first precedence are preserved. |
| Source documentation | Applied to touched boundaries | Current contracts in source; incident narratives in explicitly historical archives. No indiscriminate whole-repository rewrite. |

The proposal's `yam.operator.*` names were suggestions, not required directories.
Moving the remaining startup and ordered dispatch into another large class would
not satisfy its acceptance criterion. Those blocks assemble/use the owners above;
their remaining coupling is explicit and covered at the app boundary. This does
not assert that 2,261 operator lines are ideal or immutable.

A uniform command interface, cycle-wide measurement snapshot, independent control
threads and automatic interrupted-save repair would change runtime/data semantics.
They are not unfinished mechanical extractions: each needs a separate design and
acceptance criteria. No demonstrated defect is being deferred to one of those
projects in this queue. A newly discovered defect or useful independent owner
should reopen a concrete item, not an indefinite "clean up more" loop.

The previous checkpoint finished the items listed in that checkpoint. The broader
review above subsequently resolved its additional source/status/dispatch questions.
Neither checkpoint authorizes physical operation or team publication.

## Previous checkpoint verification (`216fda0`)

- 1,064/1,064 checks across 68 files, including actual application failure/retry paths.
- 71/71 deliberately broken cases caught by five falsifiers.
- 32/32 interactions in a disposable simulator copy; no user slots used.
- Structural, command-flag and documentation checks pass; prose ceilings unchanged.
- 12 CLI output/exit-status comparisons match the prior baseline.
- All 3,455 original recording files match the pre-switch SHA-256 manifest.
- The local commit containing this queue saves the instructions, code, tests and
  handoff together; post-commit Git status is checked before the final response.

Evidence is under agents/codex/validation/continuation-* (ignored local logs).
Physical behavior is not verified by these checks. No remote push, hardware session,
calibration/limit change, original branch rewrite or recurring automation occurred.

Repository AGENTS.md is the documented Codex mechanism for project instructions:
[official instruction-discovery documentation](https://learn.chatgpt.com/docs/agent-configuration/agents-md).
The rule has been read and applied in this run. Fresh-session automatic loading
has not been independently tested; this file does not claim a scheduler or an
unbreakable execution guard.
