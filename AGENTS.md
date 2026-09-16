# Working in this repository

Read [the current cleanup handoff](docs/CLEANUP.md) and
[the work queue](docs/WORK_QUEUE.md) before continuing the teleop cleanup.
Recheck the branch and working tree; dated notes are not live verification.

## Continue through checkpoints

Julien has repeatedly authorized sensible continuation of the local cleanup.
Do not require him to send another "continue" after each passing test or commit.

- Keep a concrete queue with completion evidence and any genuine dependencies.
- After a verified increment, save it, update the queue, then execute the next
  authorized, useful item in the same turn. Report checkpoints in commentary.
- Before ending, inspect every unfinished item. If an item can be advanced within
  the authorized scope, continue. A clean working tree, smaller file, completed
  subtask or green test suite is not by itself a stopping condition.
- End when the requested scope is completed, the user pauses/cancels, or progress
  genuinely requires missing information, authorization or an unavailable resource.
  Name the concrete condition and what would resolve it. Continue independent
  items even when one item is blocked.
- Do not relabel unfinished agreed work "future work" merely to finish a turn.
  Explain a real scope/design reason when deciding against a proposed change.
- Do not manufacture extra work or rewrite working algorithms to remain busy.
  Completion is allowed; state what is complete and what was intentionally excluded.
- Keep status honest: a final response ends the turn. Do not imply background work
  continues unless an actual running task or automation exists.

These are execution instructions, not a scheduler or a guarantee against app
interruptions, tool failures or usage limits. Do not claim they provide automatic
resumption. Do not create recurring jobs, other tasks or subagents without the
applicable user authorization.

## Preserve and verify

- Keep the saved training work, original Fable refs, recordings and environments
  identified in CLEANUP. Never run the full simulator driver against user slots.
- Use `.venv-teleop/bin/python` for this branch's checks. Exercise the simulator
  in a disposable copy, as described in CLEANUP.
- Keep calibration, limits and physical stop policy unchanged during cleanup.
  This work authorizes local edits/checks/commits, not operating physical devices
  or pushing to the team remote.
- Test actual application behavior at changed boundaries. Source-string checks
  alone cannot establish execution, ownership or failure handling.
- Preserve concise current contracts in source; put dated incident narratives in
  an explicitly historical archive. Do not discard rationale while shortening code.
- Record verified outcomes and limitations in the handoff; distinguish a completed
  check from a started check or an older checkpoint's result.
