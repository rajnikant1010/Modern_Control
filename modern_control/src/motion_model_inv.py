#!/usr/bin/env python

import rospy
from  math import *
import rospkg
import numpy as np
import scipy.linalg as la
import matplotlib.pyplot as plt
import matplotlib.animation as animation

from scipy import integrate
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from numpy.linalg import inv
from modern_control.msg import motion_model_inv
from modern_control.msg import iter_info

g = 9.8  # acceleration due to gravity, in m/s^2
l = 1.0  # length of pendulum 1 in m
#L2 = 1.0  # length of pendulum 2 in m
M1 = 2.0  # cart mass
M2 = 1.0  # pendulum mass
u = 0.1

# th1 and th2 are the initial angles (degrees)
# w10 and w20 are the initial angular velocities (degrees per second)
y = 1
theta = 0.2
ydot = 0.05
thetadot = 0.0

def derivs(state, t):

    dydx = np.zeros_like(state)
    Mq = np.matrix([[(M1+M2), M2*l*cos(theta)], [M2*l*cos(theta), M2*np.power(l,2)]])
    xst = np.matrix([[u+M2*l*np.power(thetadot,2)*sin(theta)], [M2*g*l*sin(theta)]])
    
    dydx[0] = state[2]

    dydx[1] = state[3]

    xdot = np.multiply(inv(Mq), xst)

    dydx[2] = xdot.item(0)

    dydx[3] = xdot.item(1) 
   
    return dydx

# initial state
state = [y, theta, ydot, thetadot]

# create a time array from 0..100 sampled at 0.05 second steps
dt = 0.5
time_end = 20
t = np.arange(0.0, time_end, dt)
        
if __name__ == '__main__':
    rospy.init_node('motion_model_inv', anonymous=True)
    x_new = integrate.odeint(derivs, state, t)
    a = x_new.shape
    r = rospy.Rate(20)
    while not rospy.is_shutdown():
	for i in xrange(0, int(time_end/dt-1)):
		out = motion_model_inv()
		out2 = iter_info()
		out.y = x_new[:,0].item(i)
		out.theta = x_new[:,1].item(i)
		out.ydot = x_new[:,2].item(i)
		out.thetadot = x_new[:,3].item(i)
		out2.dt = dt
		out2.end_time = time_end
		out2.i = i
		pub1 = rospy.Publisher('/states_inv', motion_model_inv, queue_size = 15)
		pub2 = rospy.Publisher('/iter_inform', iter_info, queue_size = 15)
		pub1.publish(out)
		pub2.publish(out2)
		r.sleep()
    rospy.loginfo("Motion model Node Has Shutdown.")
    rospy.signal_shutdown(0)



