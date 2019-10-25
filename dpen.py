""" Jack Baude double pendulum prediction """
import matplotlib.pyplot as plt
import numpy as np
import scipy.integrate as integrate
"""Constants of the pen"""
# Pendulum rod lengths (m), bob masses (kg).
L1, L2 = 1, 1
M1, M2 = 1, 1
#graivity
G = 9.81
#making these sin and cos methods makes for cleaner code below
def sin(x):
    return np.sin(x)
def cos(x):
    return np.cos(x)


def kinematics(state):
    #acceleration is the second derivative
    x1 = (-theta1**2 * L1 * sin(theta1)) + (L1 * cos(theta1))
    y1 = (theta1**2 * L1 * cos(theta1)) + (L1 * sin(theta1))
    x2 = (x1) - (theta2**2 * L2 * sin(theta2)) + (theta2 * L2 * cos(theta2))
    y2 = (y1) + (theta2**2 * L2 * cos(theta2)) + (theta2 * L2 * sin(theta2))
def equations(state, t):
    """angular velocity of the pendulum"""
    #state of [0] is the theta 1 or the angle of the first pendulum
    #state of [1] is the angular velocirty of mass 1
    #state of [2] is the theta 2 of the 2nd angle of the 2nd pendulum
    #state of [3] is the angular velocity of the mass of 2
    theta1 = state[0]
    theta2 = state[2]
    intVM1 = state[1]
    intVM2 = state[3]

    theta1V = (-G * (2 * M1 + M2) * sin(theta1)) - (M2 * G * sin(theta1 - theta2)) - (2 * sin(theta1 - theta2) * M2) * (theta2**2 * L2 + theta1**2 * L1 * cos(theta1 - theta2))
    theta2V = (2 * sin(theta1 - theta2)) * (theta1**2 * L1 * (M1 + M2)) + (G * (M1 + M2) * cos(theta1)) + (theta2**2 * L2 * M2 * cos(theta1 - theta2))
    denom = (2 * M1 + M2 - M2 * cos((2 * theta1) - (2 * theta2)))
    theta1V /= (L1 * denom)
    theta2V /= (L2 * denom)
    finalVs = [theta1V, theta2V]
    return finalVs

def deriv(state, t):
    #state of [0] is the theta 1 or the angle of the first pendulum
    #state of [1] is the angular velocirty of mass 1
    #state of [2] is the theta 2 of the 2nd angle of the 2nd pendulum
    #state of [3] is the angular velocity of the mass of 2
    """
    theta1 = state[0]
    theta2 = state[2]
    intVM1 = state[1]
    intVM2 = state[3]
    """
    #makes a empty array
    dydx = np.zeros_like(state)
    #sets the dydx[1] equal to angular velocity of mass 1
    dydx[0] = state[1]
    #sets del_ to the
    del_ = state[2] - state[0]
    den1 = (M1 + M2)*L1 - M2*L1*np.cos(del_)*np.cos(del_)
    dydx[1] = (M2*L1*state[1]*state[1]*np.sin(del_)*np.cos(del_) +
               M2*G*np.sin(state[2])*np.cos(del_) +
               M2*L2*state[3]*state[3]*np.sin(del_) -
               (M1 + M2)*G*np.sin(state[0]))/den1
    #derivative of dydx
    dydx[2] = state[3]

    den2 = (L2/L1)*den1
    dydx[3] = (-M2*L2*state[3]*state[3]*np.sin(del_)*np.cos(del_) +
               (M1 + M2)*G*np.sin(state[0])*np.cos(del_) -
               (M1 + M2)*L1*state[1]*state[1]*np.sin(del_) -
               (M1 + M2)*G*np.sin(state[2]))/den2

    return dydx

#this is the time spacing because the odeint method needs this time spacing because we are taking the dirvative in respect to time
dt = 0.00001
t = np.arange(0.0, 20, dt)

#the inital state of the angles of the pen
theta1 = 120.0
theta2 = 0.0
#intial veloity of the masses
v1 = 0
v2 = 0
state = [theta1, v1, theta2, v2]
dydt = integrate.odeint(deriv, state, t)



x1 = L1*sin(dydt[:, 0])

y1 = -L1*cos(dydt[:, 0])

x2 = L2*sin(dydt[:, 2]) + x1

y2 = -L2*cos(dydt[:, 2]) + y1

#print(deriv(state, t))
#print (dydt)


print (x1)
print (y1)
print (x2)
print (y2)
#Where in time would you like to look at the pendulum
tIN = 19
amountOfTimePoints = t / dt
TS = tIN * amountOfTimePoints
#plotting the pen
#solves time for a point of the x2 and y2
def solveForPoint2(x, y):
    pass


plt.plot([0, x1[TS], x2[TS]], [0, y1[TS], y2[TS]])
plt.axis([-2, 2, -2, 2])

plt.ylabel("double pen")
plt.grid()
plt.show()
