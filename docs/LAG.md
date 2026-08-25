# Why the follower arm is behind your hand

> Who this is for: Julien and the team. It assumes you have never seen this code.
>
> Reading all of it takes about twelve minutes. Section 1 is the answer in six lines, and section 7 is the code.
>
> Why it exists.
>
> Julien asked on 2026-08-20: *"why is the mirroring arm in mirror mode so far behind the first arm? I don't quite understand why I can't just read out the position data of the one arm and then basically paste it on the other arm. That should be really easy and really quick because nothing has to be calculated. It only has to be sent. Why does that take so long?"*
>
> ⭐ The reading and the pasting are indeed free. Measured below: about 8 microseconds of a 11 800 microsecond pass. Everything that makes the follower late happens after the paste. Most of it is a limit somebody chose on purpose.
>
> ⛔ Every number in this file was measured. Where something is a guess it says so.

## 1. The short answer

1. Copying the leader's angles onto the follower costs about 8 microseconds per pass, out of about 11 800. Your instinct is right: that part is free.
2. The follower is then only allowed to move at 1.0 radian per second, and that is a software limit this repo chose. Your hand, hand-guiding the leader, has been measured at 5.66 and 6.83 radians per second. So the follower falls behind at up to 5.8 radians per second while you move fast, and it has nothing to do with computing time.
3. On top of that, the command is never allowed to be more than 0.25 radians ahead of where the follower actually is. That caps how hard the motor is asked to catch up, because the motor's push is proportional to that distance.
4. Then physics adds its own share. The follower's motors only push while they are away from their target, so the arm always trails a moving command and always settles a little short of a still one. Measured: 0.04 to 0.10 radians short at rest, plus a delay of about 0.033 seconds while moving.
5. When you move slowly the whole chain works well. Measured on your own hardware on 2026-08-17: 0.012 radians behind, through 83.8 degrees of hand-guided movement.
6. So the lag you see is the price of the speed limits. The copy is free. Section 5 says which dial moves which part, and which part no dial can move.

## 2. The words used here

Read this once and the rest of the file needs nothing else.

| word | what it means here |
|---|---|
| joint | one motor in the arm. Each arm has six that move it plus one for the jaws |
| radian | an angle. One radian is about 57 degrees. Joint angles and joint errors are always in radians in this repo |
| rad/s | radians per second, so a speed of rotation. 1.0 rad/s is one joint turning 57 degrees in a second |
| the loop | the one piece of code that runs the whole session, over and over, about 85 times a second on the Mac and about 97 on the Linux station |
| a pass | one trip round that loop. At 85 times a second, one pass is 11.8 milliseconds long |
| leader | the arm your hand moves |
| follower | the other arm, the one copying |
| measured position | where a joint actually is, read from its motor |
| command | where we tell a joint to go. The two are never the same while anything is moving |
| position control | how these motors work. You give a motor an angle to go to, and it pushes towards that angle with a force proportional to how far away it is. It cannot be told "use this much force" |
| rate limit | a rule that says a command may not change by more than so much per pass. It turns a jump into a ramp |
| droop | how far short of its command a joint settles when everything has stopped moving. Caused by gravity and friction against a push that fades as the error shrinks |
| the gap | how far the follower is from the leader, in radians, at this moment |

## 3. What happens in one pass, in order

Every pass of the loop does these six things for mirror mode. The cost column is what each one takes out of the 11.8 milliseconds a pass has.

| # | what happens | cost | where |
|---|---|---|---|
| 1 | Read the leader's seven joint angles from its motors, over its own USB CAN adapter | about 3 ms for seven motors, measured | [teleop_session.py:4182](../apps/teleop_session.py) |
| 2 | Read the follower's seven angles the same way | about 3 ms | [teleop_session.py:4183](../apps/teleop_session.py) |
| 3 | Copy the leader's angles into a target for the follower, negating three of them if you asked for `mirror` instead of `copy` | **0.3 µs** | [mirror.py:206](../src/yam/mirror.py) `follower_target` |
| 4 | Decide the follower's command: move the previous command towards the target, by at most `follow_speed × dt` | **about 8 µs for all of step 4** | [mirror.py:532](../src/yam/mirror.py) |
| 5 | Clamp that command twice more: no faster than `max_speed × dt`, and no further than `max_lag` from where the follower actually is | microseconds | [robot.py:904](../src/yam/robot.py) and [robot.py:906](../src/yam/robot.py) |
| 6 | Send it to the follower's motors | part of the CAN traffic above | inside the vendor library |

⭐ Steps 3, 4 and 5 are the whole of the thinking, and together they take about 8 microseconds. That is 0.07% of one pass. Nothing in the software is slow.

⚠️ Two honest notes on that 8. It is a median of 3000 calls with the link actually following, because a stopped link returns early and times faster. And it doubles to 15 or 19 microseconds if the processor has just been idling, because the clock speed has to come back up. Neither changes the conclusion, and both are why the number comes with the command that produced it: `uv run apps/bench_loop.py`.

⚠️ Steps 1, 2 and 6 are the CAN traffic, and they are the real time cost inside a pass. About 3 milliseconds per arm, measured with `apps/bench_can.py` on the real hardware. Even so, they fit inside 11.8 milliseconds with room to spare, and none of that is what puts the follower behind.

## 4. So where does the lag come from

**The rate limit, and this is most of it.**

Step 4 above moves the command towards the target by a bounded amount each pass. The bound is `follow_speed × dt`. With the default 1.0 rad/s and a pass of 11.8 ms, that is 0.0118 radians per pass. In degrees, 0.68.

So the follower's command can advance by at most 1.0 radian per second, whatever your hand does. Two measurements from your own logs of 2026-08-17 show what that produces:

| your run | what the follower was allowed | how fast you moved the leader | what happened |
|---|---|---|---|
| 1 | 4.0 rad/s (`--max-speed 4`) | 5.66 rad/s | the gap reached 0.636 rad after 0.38 s, and mirror stopped itself |
| 2 | 10 rad/s (`--max-speed 10`) | 6.83 rad/s | the follower **managed only 2.64 rad/s**, the gap reached 0.640 rad after 0.15 s, and mirror stopped |

⭐ The arithmetic is simply the difference. In run 1 the gap grows at 5.66 − 4.0 = 1.66 rad/s, and 0.636 ÷ 1.66 = 0.38 seconds. In run 2 the follower was allowed more than it could physically do. So the gap grows at 6.83 − 2.64 = 4.19 rad/s, and 0.640 ÷ 4.19 = 0.15 seconds. Both match the logs.

⛔ Run 2 is the important one. The follower was allowed 10 rad/s and delivered 2.64. That joint is the gripper twist, and it physically tops out near 2.6 rad/s. Past that point no software limit is involved at all, so raising one changes nothing.

**The following-error clamp, which caps the catching up.**

Step 5 also refuses to let the command sit more than `max_lag` from the follower's measured position. The default is 0.25 radians.

This matters more than it looks. The motor's push is proportional to the distance between command and measured position. Capping that distance at 0.25 radians therefore caps the force available to close a gap. So a follower that has fallen behind cannot be pulled back by raising `--max-speed` alone: the speed limit stops binding, and this one starts.

⚠️ Raising `max_lag` is a real decision rather than a tuning knob. A bigger allowance means the arm pushes harder when it meets something, including a hand.

**Physics, which no dial removes.**

The follower's motors are position controlled. They push towards the commanded angle with a force proportional to the error, so there must be an error for there to be a push. Two consequences, both measured on your arms in 2026-08-13:

> following error ≈ 0.04 to 0.10 radians, plus 0.033 seconds × speed

The constant part is friction: below a small error the joint does not move at all. That is your two-centimetre sphere. Measured: 0.024 radians of joint error is 11 millimetres at the tip in the extended pose your log shows.

The speed-dependent part is a delay of about 0.033 seconds. Section 6 is about what that number really is, because it is easy to read it as a computing cost and it is not one.

## 5. The dials, and what each one can and cannot do

| dial | what it changes | what it cannot do |
|---|---|---|
| `--max-speed` | The ceiling on every commanded joint speed, and the mirror's follow speed is taken from it. Default 1.0 rad/s | It cannot beat the arm's own top speed. The gripper twist tops out near 2.6 rad/s, measured, so anything above that is fiction |
| `--mirror-gap` | How far behind the follower may fall before mirror stops itself. Default 0.35 rad, scaled per joint | ⛔ It does not make the follower faster. It only raises the tolerance before the stop, so a bigger number means a bigger gap is accepted rather than a smaller gap achieved |
| `--max-lag` | How far ahead of the arm the command may sit. Default 0.25 rad | It is also a force limit, so raising it makes every collision harder. It is a safety decision |
| `--mirror-catchup` | Accumulates the standing droop into a small bias and aims past the leader, up to 0.06 rad. Default off | It targets the offset when you are moving slowly. It deliberately does nothing during a fast sweep, and on hardware in 2026-08-18 it was weak |
| `--vel-ff` | Sends the command's own speed to the motor alongside the angle, so torque can flow before an error builds. Default off, capped at 1.0 | Above 1.0 is a measured dead end: the velocity setpoint then contradicts the position command and the arm steps and buzzes |

⭐ If you want the follower closer during slow, careful work, `--mirror-catchup` and `--vel-ff` are the two aimed at that. If you want it to survive a fast flick of your wrist, nothing here does that, because the arm itself cannot move that fast.

## 6. The 33 milliseconds, and what that number actually measures

⛔ In chat on 2026-08-20 this was described as each arm taking 33 milliseconds to process. That was wrong, and it is worth correcting properly. Nothing in this system takes 33 milliseconds to compute anything. The measured software cost of a mirror decision is about 8 microseconds. That is roughly four thousand times smaller.

What the 0.033 seconds is: the speed-dependent half of the arm's measured following error. Drive a joint at a steady speed and it trails its command by whatever it would have covered in 0.033 seconds. At 1 rad/s that is 0.033 radians of lag, and at 2 rad/s it is 0.066.

⚠️ And there is an open question about it, recorded here because it changes what you would do about it. A later reading of the same data in [FINDINGS §37.1](FINDINGS.md) found the delay to be nearly identical on joints with very different motor gains. A real physical lag would differ between them. The leading explanation is that the 0.033 seconds is the rate limiter itself rather than the motors. One clamp sitting below all six joints would produce exactly one shared constant. That is what the data shows.

⛔ That explanation is not proven. Separating a clamp from a real delay needs one bench run with `max_speed` raised well above the speeds being commanded, and that run has never happened. Section 8 has it as the one measurement that would settle this.

## 7. Every piece of code involved

The mirror path, in the order a pass visits it.

| what it does | file and line | worth knowing |
|---|---|---|
| The whole mirror explanation, in the code | [src/yam/mirror.py](../src/yam/mirror.py) top of file | Written for a reader, including why `copy` is the default and why engagement is staged |
| Copy or reflect the leader's angles | [src/yam/mirror.py:206](../src/yam/mirror.py) `follower_target` | Six joints only. The jaws are deliberately left alone |
| The speed and tolerance defaults | [src/yam/mirror.py:68](../src/yam/mirror.py) to line 75 | `align_speed` 0.30, `follow_speed` 1.0, engage tolerance 0.05, `max_gap` 0.35 |
| Why the gap limit differs per joint | [src/yam/mirror.py:107](../src/yam/mirror.py) `GAP_WEIGHTS` | The elbow moves the tip 0.418 m per radian and the gripper twist 0.051, so one threshold for all six was wrong |
| The rate limit that causes most of the lag | [src/yam/mirror.py:532](../src/yam/mirror.py) to line 536 | Three lines. `step = rate × dt`, then the command moves at most that far towards the target |
| The catch-up bias for the standing droop | [src/yam/mirror.py:506](../src/yam/mirror.py) onwards | Four guards, each written out with the failure it prevents |
| Where the loop calls all of it | [apps/teleop_session.py:4176](../apps/teleop_session.py) to line 4200 | Reads both arms, calls `step`, and narrates a stop |
| The two limits below everything | [src/yam/robot.py:904](../src/yam/robot.py) and [line 906](../src/yam/robot.py) | The speed budget and the following-error clamp, in two lines |
| Why those two numbers are what they are | [src/yam/robot.py:46](../src/yam/robot.py) to line 75 | `SAFE_MAX_SPEED` 1.0 and `SAFE_MAX_LAG` 0.25, with the reasoning and the hardware history |
| Velocity feedforward | [src/yam/robot.py:914](../src/yam/robot.py) onwards | The one lever aimed at the delay rather than at the limits |
| An arm that lags on purpose, for testing | [src/yam/fake/arm.py](../src/yam/fake/arm.py) | Built from the measured law, so a test can see a following error at all |

The measurements behind every number above.

| what was measured | where it is written down |
|---|---|
| The follower's own top speed, 2.64 rad/s on the gripper twist, and both of your stopped runs | [FINDINGS §62.0](FINDINGS.md) |
| Your two-centimetre sphere, explained and turned into millimetres | [FINDINGS §64.0](FINDINGS.md) |
| Why the per-joint gap limits exist | [FINDINGS §64.1](FINDINGS.md) |
| The following-error law, `0.04 to 0.10 rad + 0.033 s × speed` | [ROADMAP §8.2](ROADMAP.md) item 11 and [FINDINGS §59.0](FINDINGS.md) |
| The doubt about what the 0.033 seconds is | [FINDINGS §37.1](FINDINGS.md) |
| Where teleop latency comes from, in one place | [FINDINGS §66.1](FINDINGS.md) |
| Why the catch-up term is weak on hardware | [FINDINGS §67.1](FINDINGS.md) |
| The static-friction floor, and that only the motor gain touches it | [FINDINGS §69.2](FINDINGS.md) |
| The four speed limits and which one binds what | [FINDINGS §58.3](FINDINGS.md) and [§65.0](FINDINGS.md) |
| The CAN round trip, 3.12 ms mean for seven motors | [HISTORY.md](HISTORY.md) |
| The loop's own rate and its worst pass | [PERFORMANCE.md](PERFORMANCE.md) section 2 |

## 8. What is still unknown

Three things, in the order they would be worth measuring.

1. ⭐ Whether the 0.033 seconds is the rate limiter or the motors. One run with `--max-speed` set well above the speeds actually commanded would separate them. If the delay shrinks, it was the clamp. If it stays, it is the hardware. ⛔ This raises a speed limit, so it is your decision and your bench time.
2. What the follower's real top speed is on each joint. Only the gripper twist has been measured, at about 2.64 rad/s, and it was measured by accident during a run that stopped. Mirror mode produces this data every time it runs and nothing collects it. That is [ROADMAP §8.2](ROADMAP.md) item 37.
3. Whether the loop's rate on the Linux station changes any of this. A faster loop makes each per-pass step smaller in time but more frequent, so the rad/s ceiling is unchanged. What does change is `MAX_PLANNED_JOINT_SPEED`, a constant that multiplies a per-pass step by an assumed 100 passes a second. [PERFORMANCE.md](PERFORMANCE.md) section 2 has the measurement and it is on your decision list.

---

**Where to go next**

- [COMMANDS.md](COMMANDS.md) for the keys and flags named here, including `i` for mirror
- [PERFORMANCE.md](PERFORMANCE.md) for the loop itself and what a pass costs
- [ARCHITECTURE.md](ARCHITECTURE.md) section 2 if a word in section 2 above was still unfamiliar
- [FINDINGS.md](FINDINGS.md) section 62 for the two mirror runs quoted here

*Written 2026-08-20, from the code as it stands and from measurements taken on your own arms between 2026-08-13 and 2026-08-18. The software timings in sections 3 and 4 were taken on the Mac on 2026-08-20, and `uv run apps/bench_loop.py` re-takes all of them on any machine in about half a minute, with no hardware attached.*
