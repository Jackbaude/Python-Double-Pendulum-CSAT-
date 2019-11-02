import numpy as np
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation
from doublePen import DoublePen


p1 = DoublePen(1, 1, 1, 1, 180.0, 16.0)


fig, ax = plt.subplots()
xdata, ydata = [], []

(ln,) = plt.plot([], [],)


def init():
    ax.set_xlim(-(p1.l1 + p1.l2), (p1.l1 + p1.l2))
    ax.set_ylim(-(p1.l1 + p1.l2), (p1.l1 + p1.l2))
    return (ln,)


def update(frame):
    # float(frame)
    points = p1.calculate(float(frame))
    xs = points[0]
    ys = points[1]
    xdata.clear()
    ydata.clear()
    xdata.append(xs)
    ydata.append(ys)
    ln.set_data(xdata, ydata)
    return (ln,)




ani = FuncAnimation(
    fig,
    update,
    frames=np.arange(1, p1.time_stamp(9.12)[1], 0.1),
    interval=0,
    init_func=init,
    blit=True,
)
plt.show()
