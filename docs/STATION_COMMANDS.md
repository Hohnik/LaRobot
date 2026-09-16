# Station checks: connection, folders and workflow

## Current status

The attended B/G, combined-arm and camera-backed recording/replay checks are
complete. Slot 9 contains the verified take. `/tmp/yam-station-check record`
now refuses another run because that slot is occupied; preserve it.
No further motor command is required to finish this validation sequence.

## How the computers and folders relate

The Mac connects by SSH using alias `yam-pc`, which resolves to
`lavita@10.64.9.60` on the Linux station **RoVita**. Commands launched through SSH
run on that station. They use its USB/CAN devices and write to its filesystem.

| Location | Purpose | Git state at the verification snapshot |
| --- | --- | --- |
| Mac: `/Users/julien/Developer/Projects/yam-robotics` | Cleanup development and review package | `codex/teleop-cleanup`; latest revision is in Git and the package manifest |
| Station: `/tmp/yam-validation-025fab1-LWvBIc/operator` | Tested application and disposable recordings | Fixed commit `025fab1`, detached HEAD; a pinned revision without an active branch |
| Station: `/home/lavita/LaRobot` | Teammate's working project | Clean `feature/physical` at `fd2c64b` in the September 16, 15:47 UTC snapshot; the pushed tip matched afterward |
| Station: `/home/lavita/yam-robotics` | Original Fable/reference checkout | `main` at `499d0b7`, with the station's park-pose edits |

These are separate source folders and Git working trees on the same station.
The earlier recording snapshot showed `087a4fc` with an edited physical script;
the teammate committed that work independently before the later source review.
Branch positions and working changes can advance again. [BRIDGE](BRIDGE.md)
records the revisions compared across the team's branches.
The test has its own copied configuration and recording directory. It reuses the
reference checkout's Python environment and I2RT vendor directory without changing
them. Both projects access the same physical arms, so controller ownership still
has to be coordinated with the teammate.

At the recording-check snapshot, Julien's `bash-5.2$` shell was in
`/tmp/yam-validation-025fab1-LWvBIc`, the parent of `operator/`. The helper changes
directory in its child process; on exit the original shell stays in that parent.
That explains the `ls` output containing `check`, `logs`, `operator` and `checkout`.
`cd .` leaves the current directory unchanged. To identify the current shell:

```sh
hostname
pwd
```

The Codex terminal panel's local workspace label does not change when its shell
enters SSH. The prompt and these commands identify where terminal commands run.

## Helper and completed recording sequence

`/tmp/yam-station-check` is installed on the Mac and RoVita. It connects from the
Mac or runs locally on the station, from any directory. It returns to the shell
where it started; no separate SSH or `cd` steps are needed. Do not close the SSH
connection while a controller is running. These are temporary validation helpers;
the normal repository launcher remains `./teleop`.

The completed check used the record profile, which opened D405 serial
`260323072846` in colour and enabled both arms in HOLD. Julien selected G with
`a`, entered TELEOP with `t`, used `w` to start/stop and saved into slot 9.
The result contains 11.88 seconds, 1,166 samples of 14 joints and 357 images at
about 30 fps. He also replayed it with the application's `l` workflow.

The helper verifies candidate `025fab1`, application status and copied config
hashes before starting; its occupied-slot guard now prevents accidental reuse.
A new capture test needs a deliberately prepared empty destination. Do not delete
the verified take or bypass the guard to obtain another run.

## Selection and exit workflow

With the currently connected shared puck, `a` cycles B, G and BOTH. Each arm
retains its mode. Pressing `t` on an arm already in TELEOP reports that state;
it is not an error. `h` selects explicit HOLD. BOTH routes the shared puck to both
arms. With separate pucks, each puck drives its own arm regardless of mode-key
selection. The local handoff corrects the older generic shared-puck help text;
the physical operator remains the tested application at `025fab1`.

The verified normal exit is `q`, wait for the menu, then `q` again. The second
`q` intentionally parks the arms before disabling them. Keep their park paths
clear. Ctrl-C and fault handling can also initiate parking. The alternative `d`
disables directly and requires support for every enabled arm. Software keys do
not replace the physical power cut. All five attended logs confirm motors disabled.

## Saved evidence and remaining physical setup

The full take, replay report and logs are preserved outside `/tmp` at
`/home/lavita/yam-validation-evidence/2026-09-16-025fab1-slot9` on the station.
It has a checksum manifest and README; it is evidence, not an installed controller.
The temporary originals remain in place. Mac evidence contains metadata, logs and
three sampled take images rather than the full image series.

The D405 view is still dark and tilted upward toward the arm and window. Aim and
secure it toward the task before collecting useful demonstrations. The overhead
Logitech was not enumerated; its connection still needs physical inspection.
Left/right roles and camera mounts are not established, so the tests retained
world-frame control. These setup limits do not undo the verified capture/save path.

Full results and limits: [STATION_VALIDATION](STATION_VALIDATION.md).
Review readiness: [COLLEAGUE_HANDOFF](COLLEAGUE_HANDOFF.md).
