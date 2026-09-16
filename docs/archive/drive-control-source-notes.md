# Historical drive-key branches

Verbatim branches and angular bounds from the operator before DriveControls.
They preserve prior comments and message policy; current code lives in
`src/yam/ui/drive_controls.py`.

```python
MAX_LINEAR_SCALE = 15.0
MIN_LINEAR_SCALE = 0.005

#: ⭐ The same backstop for rotation. Default is 0.6 rad/s, so 12 rad/s is 20x it and about
#: 690°/s — well past useful and nowhere near reachable, which is what a backstop should be.
#: ⚠️ His 2026-08-17 readout showed `rot 954°/s`, so this direction was being pushed too.
MAX_ANGULAR_SCALE = 12.0
MIN_ANGULAR_SCALE = 0.02
```

```python
                        # Allow both linear and angular speed adjustments in CONTROLS; display both scales.
                        elif k in "+=":
                            args.linear_scale = adjust_setting(
                                "linear_scale", args.linear_scale, True)
                            hint(f"linear speed {args.linear_scale:.3f} m/s"
                                 + (" (ceiling)"
                                    if args.linear_scale >= MAX_LINEAR_SCALE else ""))
                        elif k == "-":
                            args.linear_scale = adjust_setting(
                                "linear_scale", args.linear_scale, False)
                            hint(f"linear speed {args.linear_scale:.3f} m/s")
                        elif k == ".":
                            angular_scale = min(MAX_ANGULAR_SCALE,
                                                angular_scale * 1.25)
                            print(f"\n  rotation speed → {angular_scale:.2f} rad/s "
                                  f"({np.degrees(angular_scale):.0f}°/s)\n")
                        elif k == ",":
                            angular_scale = max(MIN_ANGULAR_SCALE,
                                                angular_scale / 1.25)
                            print(f"\n  rotation speed → {angular_scale:.2f} rad/s "
                                  f"({np.degrees(angular_scale):.0f}°/s)\n")
                        elif k == "r":
                            rotation = not rotation
                            print(f"\n  wrist rotation {'ON' if rotation else 'OFF'}"
                                  f"{'' if rotation else ' — ROLL/PITCH/YAW will not move'}\n")
```

```python
                    elif k == "r":
                        rotation = not rotation
                        hint(f"wrist rotation {'ON' if rotation else 'OFF'}")
                    elif k == ".":
                        angular_scale = min(MAX_ANGULAR_SCALE,
                                            angular_scale * 1.25)
                        hint(f"rotation speed {angular_scale:.2f} rad/s")
                    elif k == ",":
                        angular_scale = max(MIN_ANGULAR_SCALE,
                                            angular_scale / 1.25)
                        hint(f"rotation speed {angular_scale:.2f} rad/s")
```

```python
                    elif k == "+" or k == "=":
                        # Speed keys adjust park speed in PARK and cursor pace during scrub.
                        # Scrub remains subject to replay lag gating and robot command limits.
                        if playback.active is not None and playback.scrub:
                            args.scrub_max = adjust_setting(
                                "scrub_max", args.scrub_max, True)
                            hint(f"scrub pace: full push = {args.scrub_max:g}x the "
                                 f"recording's own speed")
                        elif any(one.mode == "park" for one in aimed):
                            for one in aimed:
                                one.park_speed = min(args.teleop_speed,
                                                     one.park_speed * 1.25)
                            hint(f"park speed {edit_arm.park_speed:.2f} rad/s")
                        else:
                            args.linear_scale = adjust_setting(
                                "linear_scale", args.linear_scale, True)
                            hint(f"linear speed {args.linear_scale:.3f} m/s"
                                 + (" (ceiling)"
                                    if args.linear_scale >= MAX_LINEAR_SCALE else ""))
                    elif k == "-":
                        if playback.active is not None and playback.scrub:
                            args.scrub_max = adjust_setting(
                                "scrub_max", args.scrub_max, False)
                            hint(f"scrub pace: full push = {args.scrub_max:g}x the "
                                 f"recording's own speed")
                        elif any(one.mode == "park" for one in aimed):
                            for one in aimed:
                                one.park_speed = max(0.05, one.park_speed / 1.25)
                            hint(f"park speed {edit_arm.park_speed:.2f} rad/s")
                        else:
                            args.linear_scale = adjust_setting(
                                "linear_scale", args.linear_scale, False)
                            hint(f"linear speed {args.linear_scale:.3f} m/s")
```
