""" 
Jack Baude double pendulum prediction

- predicts where a double pendulm will be in space with time
  given the mass of the two bobs, lengths of the pendulum arms
  and the intial starting postion 
"""
import matplotlib.pyplot as plt
import numpy as np
import scipy.integrate as integrate

class doublePen:
    #Constants
    #dt is the spasce between time points that we are taking the dervative of 
    dt = 0.00001
    totalTime = 20
    #the intial velocity of the pendulum
    v1 = 0.0
    v2 = 0.0
    #Gravity velocity
    G = 9.81
    def __init__(self, mass1, mass2, length1, length2, intTheta1, intTheta2):
        self.M1 = mass1
        self.M2 = mass2
        self.L1 = length1
        self.L2 = length2
        self.startTheta1 = intTheta1
        self.startTheta2 = intTheta2
        
    
    #methods to make the equations easier to read
    def sin(self,x):
        return np.sin(x)
    def cos(self,x):
        return np.cos(x)
    #double pendulum equation that takes in the intial state of the pen as a array
    #and t as a numpy arange
    def equation(self,state, t):
            """
            theta1 = state[0]
            theta2 = state[2]
            intVM1 = state[1]
            intVM2 = state[3]
            """
            #makes a empty array of zeros
            dydx = np.zeros_like(state)
            #sets the dydx[1] equal to angular velocity of mass 1
            dydx[0] = state[1]
        
            del_ = state[2] - state[0]
            den1 = (self.M1 + self.M2)*self.L1 - self.M2*self.L1*self.cos(del_)*self.cos(del_)
            dydx[1] = (self.M2*self.L1*state[1]*state[1]*self.sin(del_)*self.cos(del_) +
                       self.M2*self.G*self.sin(state[2])*self.cos(del_) +
                       self.M2*self.L1*state[3]*state[3]*self.sin(del_) -
                       (self.M1 + self.M2)*self.G*self.sin(state[0]))/den1
            #derivative of dydx
            dydx[2] = state[3]

            den2 = (self.L1/self.L1)*den1
            dydx[3] = (-self.M2*self.L1*state[3]*state[3]*self.sin(del_)*self.cos(del_) +
                       (self.M1 + self.M2)*self.G*self.sin(state[0])*self.cos(del_) -
                       (self.M1 + self.M2)*self.L1*state[1]*state[1]*self.sin(del_) -
                       (self.M1 + self.M2)*self.G*self.sin(state[2]))/den2
                       
            return dydx
            
    #calculates the postion of the double pendulum
    def calculate(self, tcal):
        #calling the getTimeArray method to get the t stamp information
        #tcal is the time we are calculating for
        tPercent = tcal/self.totalTime
        #we need the amount of time points so we can find the nth time point in the array that the deriv returns so we can calcualte where the pen is in space
        amountOfTimePoints = self.totalTime / self.dt
        #by multiplying the amount of time points by the percentage of points we can get the index we are looking for in the list that deriv returns
        TS = int((amountOfTimePoints * tPercent) - self.dt)
        cords = self.getMassCords()
        print ("\n{} sample of {} time samples" .format(TS, amountOfTimePoints))
        print("The points of the orgin, m1, and m2 are as follows")
        print ("(0,0) , ({},{}) , ({},{})".format(cords[0][TS], cords[1][TS], cords[2][TS],cords[3][TS]))
        plt.plot([0, cords[0][TS], cords[2][TS]], [0, cords[1][TS], cords[3][TS]])
        plt.axis([-2, 2, -2, 2])
        plt.title("Double Pendulum Prediction")
        plt.ylabel("Double Pendulm {} seconds in".format(tcal))
        plt.grid()
        plt.show()
    
    #returns the x and y points of both mass 1 and mass 2
    def getMassCords(self):
        state = [self.startTheta1, self.v1, self.startTheta2, self.v2]
        dydt = integrate.odeint(self.equation, state, self.getTimeArray())
        x1 = self.L1*self.sin(dydt[:, 0])
        y1 = -(self.L1)*self.cos(dydt[:, 0])
        x2 = self.L2*self.sin(dydt[:, 2]) + x1
        y2 = -(self.L2)*self.cos(dydt[:, 2]) + y1

        return [x1, y1, x2, y2]
    #changes the totaltime and the deltaT or the space between time samples
    def changeTime(self, tt, deltaT):
        dt = deltaT
        totalTime = tt
    #returns a time array 
    def getTimeArray(self):
        t = np.arange(0.0, self.totalTime, self.dt)
        return (t)


p1 = doublePen(1,1,1,1,120.0,0)
p1.calculate(9.134)


    