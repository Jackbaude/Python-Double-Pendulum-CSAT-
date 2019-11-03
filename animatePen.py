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

#the pendulum object
p1 = DoublePen(1, 1, 1, 1, 180.0, 16.0)


fig, ax = plt.subplots()
xdata, ydata = [], []
ax.grid()
(ln,) = plt.plot([], [],'-o',alpha=1)

#the init for the animation FuncAnimation
def init():
    ax.set_xlim(-(p1.l1 + p1.l2), (p1.l1 + p1.l2))
    ax.set_ylim(-(p1.l1 + p1.l2), (p1.l1 + p1.l2))
    return (ln,)

#runs every frame that is made
def update(frame):
    points = p1.calculate((frame))
    xs = points[0]
    ys = points[1]
    xdata.clear()
    ydata.clear()
    xdata.append(xs)
    ydata.append(ys)
    ln.set_data(xdata, ydata)
    return (ln,)

#animates the double pendulum
ani = FuncAnimation(
    fig,
    update,
    frames=np.linspace(0, p1.totalTime, 1200),
    interval=0,
    init_func=init,
    blit=True,
)
#saves the animation to double_pendulum.mp4, with 60fps
ani.save('double_pendulum.mp4', fps=60,)
#plays the animation afterword through matplotlib
plt.show()
