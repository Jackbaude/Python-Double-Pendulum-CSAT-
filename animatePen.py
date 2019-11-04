"""
    Jack Baude
    - Animation script
    October 2019
    CSAT
"""

import numpy as np
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation
from doublePen import DoublePen

# the pendulum object
p1 = DoublePen(1, 1, 1, 1, 180.0, 270.0)


fig, ax = plt.subplots()
plt.axis("off")
xdata, ydata = [], []
mass_two_xs = []
mass_two_ys = []
# animating the double pendulum arm
(ln,) = plt.plot([], [], "-o", alpha=1)
# animating the second path of the pendulum or the path of the tail
(mass_two,) = plt.plot([], [], lw=2, alpha=0.5)

# the init for the animation FuncAnimation
def init():
    ax.set_xlim(-(p1.l1 + p1.l2), (p1.l1 + p1.l2))
    ax.set_ylim(-(p1.l1 + p1.l2), (p1.l1 + p1.l2))

    return (ln, mass_two)


# runs every frame that is made
def update(frame):
    points = p1.calculate((frame))
    xs = points[0]
    ys = points[1]
    # finds the cords of the second mass so we can trace them
    mass_two_x = points[0][2]
    mass_two_y = points[1][2]
    xdata.clear()
    ydata.clear()
    # add the cords of mass 2 to an array so we can graph them
    mass_two_xs.append(mass_two_x)
    mass_two_ys.append(mass_two_y)
    xdata.append(xs)
    ydata.append(ys)
    ln.set_data(xdata, ydata)
    mass_two.set_data(mass_two_xs, mass_two_ys)
    return (ln, mass_two)


# animates the double pendulum
ani = FuncAnimation(
    fig,
    update,
    frames=np.linspace(0, p1.totalTime, 1200),
    interval=0,
    init_func=init,
    blit=True,
)
# saves the animation to double_pendulum.mp4, with 60fps
ani.save("double_pendulum_trace3.mp4", fps=60)
# plays the animation afterword through matplotlib
plt.show()
