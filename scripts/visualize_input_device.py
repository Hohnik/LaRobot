import time

import matplotlib

matplotlib.use("TkAgg")  # backend
import matplotlib.pyplot as plt

from robot.inputs.spacemouse import CartesianTarget, SpaceMouseReader, open_spacemice

fig = plt.figure(figsize=(6, 6))
ax = fig.add_subplot(projection="3d")
plt.ion()
plt.show()

dev = open_spacemice()[0]
with SpaceMouseReader(dev) as sm:
    target = CartesianTarget()
    t_prev = time.monotonic()

    try:
        while plt.fignum_exists(fig.number):
            t_now = time.monotonic()
            dt = t_now - t_prev
            t_prev = t_now

            twist, btns = sm.get_twist()
            target.integrate(twist, dt)

            ax.clear()
            p, R = target.point, target.rotation

            for col, color, label in zip(R.T, ("r", "g", "b"), ("x", "y", "z")):
                ax.quiver(*p, *(col * 0.1), color=color)
                ax.text(*(p + col * 0.12), label, color=color)

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
