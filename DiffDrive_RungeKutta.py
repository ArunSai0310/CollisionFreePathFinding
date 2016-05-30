from __future__ import division
"""
****Arun Sai Sangawar Vijay**
*********800890154***********
"""
__author__ = 'ArunSai'

import numpy as np
import math

class RungeKutta(object):

    # initializes the left and right velocities, initial point, radius of the wheels, length between the wheels
    def __init__(self,pt,ul,ur):
        self.point = pt
        self.ul = ul
        self.ur = ur

        # RK4 Integration step - size
        self.h = 0.1

        # RRT epsilon step size
        self.epsilon = 0.5
        self.radius = 2
        self.length = 5

    # function to find the state transition equation
    def function(self,left_vel,right_vel,q_theta):
        x_dot = (self.radius/2) * (right_vel+left_vel) * math.cos(q_theta[2])
        y_dot = (self.radius/2) * (right_vel+left_vel) * math.sin(q_theta[2])
        theta_dot = (self.radius/self.length) * (right_vel - left_vel)

        return np.array([x_dot,y_dot,theta_dot])

    # Runge - Kutta 4th order calculation method
    def calculate(self,left_vel,right_vel):

        q = np.array(self.point)
        h_temp = self.h
        i = 1
        while h_temp <= self.epsilon:
            k_1 = self.function(left_vel,right_vel,q)
            k_2 = self.function(left_vel,right_vel,(q+0.5*self.h*k_1))
            k_3 = self.function(left_vel,right_vel,(q+0.5*self.h*k_2))
            k_4 = self.function(left_vel,right_vel,(q+self.h*k_3))

            # Main equation of runge - kutta 4
            q = q + ((1/6) * (k_1+2*k_2+2*k_3+k_4) * self.h)

            h_temp += self.h
            i += 1

        return tuple(q)

    # Returns three possible points for the differential constraints
    def runge_kutta(self):
        #If ul = ur > 0, then the robot moves forward
        straight_pt = self.calculate(self.ul,self.ur)
        # If -ul = ur != 0, then the robot rotates counter clockwise i.e. like a left turn
        left_pt = self.calculate(-self.ul,self.ur)
        # If ul = ?ur != 0, then the robot rotates clockwise i.e. like a right turn
        right_pt = self.calculate(self.ul,-self.ur)

        return [straight_pt, left_pt, right_pt]

