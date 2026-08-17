#!/usr/bin/env python3
"""Do the tests in scripts/test_fake_arm.py actually have teeth?

    uv run scripts/falsify_fake_arm.py

⭐⭐ WHY THIS IS IN THE REPO RATHER THAN A THING SOMEBODY DID ONCE. A test suite
written beside the code it tests passes **by construction**, and this week that
signature produced three tests in a row that proved nothing: command counts a
sequential park also satisfies, a "slow" follower that kept up, and an alignment where
the leader closed the gap itself (docs/FINDINGS.md §57, §58).

⭐ So this breaks `src/fake_arm.py` five different ways and insists the right test fails
each time. **It found a real blind test on its first run**: the lag-clip test asserted
only `gap <= max_lag`, which an arm that is not blocked at all satisfies trivially, so
it passed with the blocking code deleted. That test is two-sided now.

⛔ It monkey-patches the class in memory and restores it, then re-runs the real suite to
prove it left no damage. It never writes to `src/`.
"""

import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "src"))
sys.path.insert(0, str(REPO / "scripts"))

import fake_arm  # noqa: E402
import numpy as np  # noqa: E402
import test_fake_arm as T  # noqa: E402

ORIG_INIT = fake_arm.FakeArm.__init__
ORIG_STEP = fake_arm.FakeArm.step
ORIG_CMD = fake_arm.FakeArm.command_joint_pos
ORIG_STATES = fake_arm.FakeArm.states


def restore():
    fake_arm.FakeArm.__init__ = ORIG_INIT
    fake_arm.FakeArm.step = ORIG_STEP
    fake_arm.FakeArm.command_joint_pos = ORIG_CMD
    fake_arm.FakeArm.states = ORIG_STATES


def run(name):
    """Run one test by name. True = passed."""
    try:
        getattr(T, name)()
        return True
    except Exception:
        return False


def sabotage(label, patch, should_fail):
    restore()
    patch()
    results = {n: run(n) for n in should_fail}
    restore()
    caught = [n for n, ok in results.items() if not ok]
    missed = [n for n, ok in results.items() if ok]
    verdict = "CAUGHT" if not missed else "MISSED"
    print(f"\n{verdict}: {label}")
    for n in caught:
        print(f"    ✓ failed as it should: {n}")
    for n in missed:
        print(f"    ⛔ STILL PASSED, so it proves nothing: {n}")
    return not missed


# 1. Remove static friction. The constant term of the measured law disappears, so the
#    error at every speed drops below the measured band's floor.
def no_deadband():
    def init(self, *a, **k):
        ORIG_INIT(self, *a, **k)
        self.deadband = 0.0
    fake_arm.FakeArm.__init__ = init


# 2. Remove the lag. tau tiny means the joint reaches its target within a cycle, so the
#    speed-proportional term disappears.
def no_lag():
    def init(self, *a, **k):
        ORIG_INIT(self, *a, **k)
        self.tau = 1e-6
    fake_arm.FakeArm.__init__ = init


# 3. Teleport, which is exactly what the OLD fakes do. This is the defect the whole
#    module exists to avoid, so it had better be caught.
def teleport():
    def cmd(self, q):
        q = np.asarray(q, dtype=float)
        if len(q) != self.n:
            raise ValueError("wrong length")
        self.cmd = q.copy()
        self.q = q.copy()
        self.commands.append(q.copy())
        self.cycles += 1
    fake_arm.FakeArm.command_joint_pos = cmd


# 4. Misspell the temperature field. getattr's default means this reads as 0 °C rather
#    than raising, which is how a thermal guard gets silently disarmed.
def misspelled_temp():
    def states(self):
        out = ORIG_STATES(self)
        for s in out:
            s.temp_mos = 0.0
            s.temp_rotor = 0.0
        return out
    fake_arm.FakeArm.states = states


# 5. Ignore blocks, so a joint sails through an obstacle.
def ignore_blocks():
    def step(self, dt):
        blocked, self.blocked = self.blocked, {}
        ORIG_STEP(self, dt)
        self.blocked = blocked
    fake_arm.FakeArm.step = step


ALL = [
    ("static friction removed (deadband = 0)", no_deadband,
     ["test_the_fake_reproduces_the_MEASURED_lag_law"]),
    ("the lag removed (tau ~ 0)", no_lag,
     ["test_the_fake_reproduces_the_MEASURED_lag_law",
      "test_the_error_GROWS_with_speed_rather_than_staying_flat"]),
    ("the arm teleports to its command, like the OLD fakes", teleport,
     ["test_a_command_is_FOLLOWED_over_time_not_teleported",
      "test_the_fake_reproduces_the_MEASURED_lag_law"]),
    ("the temperature field misspelled, so it reads 0 °C", misspelled_temp,
     ["test_the_state_fields_satisfy_the_REAL_motor_temperatures"]),
    ("blocked joints ignored, so the arm passes through objects", ignore_blocks,
     ["test_a_blocked_joint_STOPS_and_the_following_error_grows",
      "test_SafeRobot_s_lag_clip_holds_the_command_near_a_BLOCKED_arm"]),
]

if __name__ == "__main__":
    print("Falsifying scripts/test_fake_arm.py — each sabotage must make a test fail.")
    ok = all(sabotage(*a) for a in ALL)
    restore()
    print(f"\n{'✓ every sabotage was caught' if ok else '⛔ AT LEAST ONE TEST IS BLIND'}")
    # After restoring, the real suite must still pass — proof the harness left no damage.
    print(f"suite after restore: {'PASS' if T.main() == 0 else 'FAIL'}")
    sys.exit(0 if ok else 1)
