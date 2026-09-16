# Station checks: commands for Julien

## One command on either computer

`/tmp/yam-station-check` is installed on Julien's Mac and the RoVita station.
Use the same command from any directory on either machine. It connects when
needed and returns to the shell where it was started. There are no separate SSH
or `cd` steps. Do not close an SSH connection while a controller is running.

The helper checks candidate `025fab1`, its application files and copied config
before starting. It writes a terminal log. These are temporary validation tools;
the normal repository launcher remains `./teleop`.

## Next check: one short recording with the D405

B and G have passed the attended motion checks, including the shared-puck
selection workflow. Now verify joint sampling, camera frames and take publication
together. Arrange the D405 securely with G's working area in view. A stable table
position is sufficient for this check.

After other controllers are stopped and both workspaces are clear, paste:

```sh
/tmp/yam-station-check record
```

This opens D405 serial `260323072846` in colour and enables both arms in HOLD.
It records B/G joint streams on one timeline. The planned take uses empty slot 9
inside the disposable operator checkout; original recordings are untouched.

1. Press `a` once to select G, then `t` for TELEOP.
2. Press `w` to start recording. Move G gently for about 10–15 seconds.
3. Release the puck and press `w` again.
4. Wait for the save-slot prompt, press `9`, then wait for the saved confirmation.
5. Use the verified park exit: `q`, wait for the menu, then `q` again.
6. Confirm both arms report motors 1–7 disabled. Report how the run felt.

The second `q` intentionally parks both arms before disabling them. Keep both
park paths clear. Ctrl-C and fault handling can also initiate parking. The
alternative `d` disables directly and requires support for every enabled arm.
Software keys do not replace the physical power cut.

For a preview that opens no devices:

```sh
/tmp/yam-station-check record --plan
```

## Your selection workflow

With the currently connected shared puck, `a` cycles B, G and BOTH. It changes the
selection while each arm retains its mode. Pressing `t` again on an arm already
in TELEOP reports that state; it is not an error. Use `h` for an explicit HOLD.
BOTH sends the shared puck input to both arms. With separate pucks, each puck
continues to drive its own arm regardless of mode-key selection.

The station still runs the tested application. The local handoff version corrects
two generic help messages that previously described separate pucks even when the
session was using one shared puck. This changes wording only.

## After recording

Codex can read the terminal and saved log, then validate the saved joint samples,
image files, frame metadata and publication result. That check is still pending.

The D405 probe produced colour frames. Its view pointed up at the arm
and window; aim and secure it toward the intended workspace before collecting
useful demonstrations. It is not a verified wrist-camera mount. Keep control in
the world frame. The overhead Logitech is not currently enumerated on the station;
its USB connection needs checking. Left/right arm roles remain unspecified.

Full evidence and limits: [STATION_VALIDATION](STATION_VALIDATION.md).
