"""
Jack Baude double pendulum prediction

- predicts where a double pendulm will be in space with time
  given the mass of the two bobs, lengths of the pendulum arms
  and the intial starting postion
"""



import matplotlib.pyplot as plt
import numpy as np
from numpy import cos, sin
import scipy.integrate as integrate


class DoublePen:
    # Constants
    # dt is the spasce between time points that we are taking the dervative of
    dt = 0.00001
    totalTime = 20
    # the intial velocity of the pendulum
    v1 = 0.0
    v2 = 0.0
    # Gravity velocity
    G = 9.81

    def __init__(self, mass1, mass2, length1, length2, intTheta1, intTheta2):
        self.m1 = mass1
        self.m2 = mass2
        self.l1 = length1
        self.l2 = length2
        self.startTheta1 = intTheta1
        self.startTheta2 = intTheta2

    # double pendulum equation that takes in the intial state of the pen as a array
    # and t as a numpy arange
    def equation(self, state, t):
        """
            theta1 = state[0]
            theta2 = state[2]
            intVm1 = state[1]
            intVm2 = state[3]
            """
        # makes a empty array of zeros
        dydx = np.zeros_like(state)
        # sets the dydx[1] equal to angular velocity of mass 1
        dydx[0] = state[1]

        del_ = state[2] - state[0]
        den1 = (self.m1 + self.m2) * self.l1 - self.m2 * self.l1 * cos(del_) * cos(del_)
        dydx[1] = (
            self.m2 * self.l1 * state[1] * state[1] * sin(del_) * cos(del_)
            + self.m2 * self.G * sin(state[2]) * cos(del_)
            + self.m2 * self.l1 * state[3] * state[3] * sin(del_)
            - (self.m1 + self.m2) * self.G * sin(state[0])
        ) / den1
        # derivative of dydx
        dydx[2] = state[3]

        den2 = (self.l1 / self.l1) * den1
        dydx[3] = (
            -self.m2 * self.l1 * state[3] * state[3] * sin(del_) * cos(del_)
            + (self.m1 + self.m2) * self.G * sin(state[0]) * cos(del_)
            - (self.m1 + self.m2) * self.l1 * state[1] * state[1] * sin(del_)
            - (self.m1 + self.m2) * self.G * sin(state[2])
        ) / den2

        return dydx

    # calculates the postion of the double pendulum
    def calculate(self, tcal):
        time_stamp = self.time_stamp(tcal)
        ts = time_stamp[0]
        amountOfTimePoints = time_stamp[1]
        cords = self.getMassCords()
        print("\n{} sample of {} time samples".format(ts, amountOfTimePoints))
        print("The points of the orgin, m1, and m2 are as follows")
        print(
            "(0,0) , ({},{}) , ({},{})".format(
                cords[0][ts], cords[1][ts], cords[2][ts], cords[3][ts]
            )
        )
        xs = [0, cords[0][ts], cords[2][ts]]
        ys = [0, cords[1][ts], cords[3][ts]]
        return[xs,ys]

    def plot(self,tcal):
        points = self.calculate(tcal)
        xs = points[0]
        ys = points[1]
        print(xs)
        print(ys)
        plt.plot(xs,ys)
        plt.axis([-2, 2, -2, 2])
        plt.title("Double Pendulum Prediction")
        plt.ylabel("Double Pendulm {} seconds in".format(tcal))
        plt.grid()
        plt.show()

    # returns the x and y points of both mass 1 and mass 2
    def getMassCords(self):
        state = [self.startTheta1, self.v1, self.startTheta2, self.v2]
        dydt = integrate.odeint(self.equation, state, self.getTimeArray())
        x1 = self.l1 * sin(dydt[:, 0])
        y1 = -(self.l1) * cos(dydt[:, 0])
        x2 = self.l2 * sin(dydt[:, 2]) + x1
        y2 = -(self.l2) * cos(dydt[:, 2]) + y1

        return [x1, y1, x2, y2]
    #return the time stamp or the nth point for nth time
    def time_stamp(self, tcal):
        # tcal is the time we are calculating for
        tPercent = tcal / self.totalTime
        # we need the amount of time points so we can find the nth time point in the array that the deriv returns so we can calcualte where the pen is in space
        amountOfTimePoints = self.totalTime / self.dt
        # by multiplying the amount of time points by the percentage of points we can get the index we are looking for in the list that deriv returns
        ts = int((amountOfTimePoints * tPercent) - self.dt)
        return ts, amountOfTimePoints

    # changes the totaltime and the deltaT or the space between time samples
    def changeTime(self, tt, deltaT):
        dt = deltaT
        totalTime = tt

    # returns a time array
    def getTimeArray(self):
        t = np.arange(0.0, self.totalTime, self.dt)
        return t


def main():
    p1 = DoublePen(1, 1, 1, 1, 120.0, 0)
    p1.plot(11.23)


if __name__ == "__main__":
    main()
