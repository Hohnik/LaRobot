import time

import matplotlib

matplotlib.use("TkAgg")  # backend
import matplotlib.pyplot as plt

from robot.inputs.spacemouse import SpaceMouse
from robot.kinematics.cartesian_target import CartesianTarget

fig = plt.figure(figsize=(6, 6))
ax = fig.add_subplot(projection="3d")
plt.ion()
plt.show()

with SpaceMouse() as sm:
    target = CartesianTarget()
    now = time.perf_counter()
    prev = None

    try:
        while plt.fignum_exists(fig.number):
            prev, now = now, time.perf_counter()

            velocities, btns = sm.read()
            target.integrate(velocities, abs(now - prev))

            ax.clear()
            p, R = target.position, target.rotation

            for col, color, label in zip(R.T, ("r", "g", "b"), ("x", "y", "z")):
                ax.quiver(*p, *(col * 0.1), color=color)
                ax.text(*(p + col * 0.12), s=label, color=color)

            ax.set_xlim(-0.2, 0.8)
            ax.set_ylim(-0.5, 0.5)
            ax.set_zlim(0.0, 0.7)
            ax.set_xlabel("x")
            ax.set_ylabel("y")
            ax.set_zlabel("z")
            ax.set_title(f"p = [{p[0]:+.3f} {p[1]:+.3f} {p[2]:+.3f}]")
            plt.pause(0.03)

    except KeyboardInterrupt:
        pass
