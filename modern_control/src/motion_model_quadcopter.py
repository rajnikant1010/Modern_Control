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
from modern_control.msg import motion_model_quad
from modern_control.msg import iter_info

pn = 0.2
pe = 0
h = -0.5
u = 0
v = 0
w = 0
phi = 0
theta = 0
psi = 0
p = 0
q = 0
r = 0

f_x = 0.0
f_y = 0.0
f_z = 0.0
tau_phi = 0.0
tau_theta = 0.0
tau_psi = 2.0

Jx = 0.114700
Jy = 0.057600
Jz = 0.171200

m = 1.56

def derivs(state, t):

    dydx = np.zeros_like(state)

    u = state[3]
    v = state[4]
    w = state[5]

    phi = state[6]
    theta = state[7]
    psi = state[8]
    
    p = state[9]
    q = state[10]
    r = state[11]
    
    c_theta = cos(theta)
    c_psi = cos(psi)
    c_phi = cos(phi)

    s_theta = sin(theta)
    s_psi = sin(psi)
    s_phi = sin(phi)

    t_theta = tan(theta)
    t_psi = tan(psi)
    t_phi = tan(phi)

    J1 = ((Jy-Jz)/Jx)
    J2 = ((Jz-Jx)/Jy)
    J3 = ((Jx-Jy)/Jz)

    dydx[0] = c_theta*c_psi*u + (s_phi*s_theta*c_psi - c_phi*s_psi)*v + (c_phi*s_theta*c_psi + s_phi*s_psi)*w  
 
    dydx[1] = c_theta*s_psi*u + (s_phi*s_theta*s_psi + c_phi*c_psi)*v + (c_phi*s_theta*s_psi - s_phi*c_psi)*w

    dydx[2] = s_theta*u + (-s_phi*c_theta)*v + (-c_phi*c_theta)*w

    dydx[3] = (r*v - q*w) + (f_x/m)

    dydx[4] = (p*w - r*u) + (f_y/m)

    dydx[5] = (q*u - p*v) + (f_z/m)

    dydx[6] = p + s_phi*t_theta*q + c_phi*t_theta*r

    dydx[7] = c_phi*q + (-s_phi)*r

    dydx[8] = (s_phi/c_theta)*q + (c_phi/c_theta)*r

    dydx[9] = J1*q*r + (1/Jx)*tau_phi
 
    dydx[10] = J2*p*r + (1/Jy)*tau_theta

    dydx[11] = J3*p*q + (1/Jz)*tau_psi
   
    return dydx

# initial state
state = [pn, pe, h, u, v, w, phi, theta, psi, p, q, r]

# create a time array from 0..100 sampled at 0.05 second steps
dt = 0.01
time_end = 40
t = np.arange(0.0, time_end, dt)
        
if __name__ == '__main__':
    rospy.init_node('motion_model_quadcopter', anonymous=True)
    x_new = integrate.odeint(derivs, state, t)
    a = x_new.shape
    r1 = rospy.Rate(5)
    while not rospy.is_shutdown():
	for i in xrange(0, int(time_end/dt-1)):
		out = motion_model_quad()
		out2 = iter_info()
		out.pn = x_new[:,0].item(i)
		out.pe = x_new[:,1].item(i)
		out.h = x_new[:,2].item(i)
		out.u = x_new[:,3].item(i)
		out.v = x_new[:,4].item(i)
		out.w = x_new[:,5].item(i)
		out.phi = x_new[:,6].item(i)
		out.theta = x_new[:,7].item(i)
		out.psi = x_new[:,8].item(i)
		out.p = x_new[:,9].item(i)
		out.q = x_new[:,10].item(i)
		out.r = x_new[:,11].item(i)
		
		out2.dt = dt
		out2.end_time = time_end
		out2.i = i

		pub1 = rospy.Publisher('/states_quad', motion_model_quad, queue_size = 50)
		pub2 = rospy.Publisher('/iter_inform', iter_info, queue_size = 15)
		pub1.publish(out)
		pub2.publish(out2)

		r1.sleep()
    rospy.loginfo("Motion model Node Has Shutdown.")
    rospy.signal_shutdown(0)



