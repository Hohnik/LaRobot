# Teleop cleanup work queue

Updated September 16, 2026. This queue implements Julien's instruction to continue
through useful local work without stopping at each commit. The initial state below
is based on verified checkout `190070e`; subsequent evidence is recorded per item.

## Current objective

Finish the remaining concrete, behavior-preserving cleanup items in the architecture
review, preserve the original work/data, and leave a usable current handoff.
No physical operation or remote publication is included.

## Work items

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


## Architecture review disposition

This is an assessment against the current callers and failure paths, not just file lengths.

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

All concrete local items in this queue are complete. No item is waiting for a
user answer, no software blocker is claimed, and no background work is running.
The next physical validation or team publication is separately reserved; no
continuation prompt is required to reauthorize this already-completed local work.

## Final verification

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
