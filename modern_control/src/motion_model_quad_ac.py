#!/usr/bin/env python

import rospy
import rospkg
import numpy as np
import scipy.linalg as la
import matplotlib.pyplot as plt
import matplotlib.animation as animation

from math         import *
from scipy        import integrate
from numpy        import sin, cos, tan
from std_msgs.msg import String
from numpy.linalg import inv
from mod_ctrl.msg import motion_model_quad
from mod_ctrl.msg import time_info
from mod_ctrl.msg import attitude_ctrl

# Mass of UAV
m     =  1.56    #kg
g     =  9.80665 # m/s^2

# Inertia
Jx    =  0.114700
Jy    =  0.057600
Jz    =  0.171200

# Quad States
pn    =  1.0
pe    =  2.0
h     = -3.0
u     =  0.0
v     =  0.0
w     =  0.0
phi   =  0.0
theta =  0.0
psi   =  0.0
p     =  0.0
q     =  0.0
r     =  0.0

# Forces and Torques

F 	  =  m*g
tau_phi   =  0.00
tau_theta =  0.00
tau_psi   =  0.00


# initial state
state = [pn, pe, h, u, v, w, np.radians(phi), np.radians(theta), np.radians(psi), p, q, r]

def callback1(data1):
    global tau_phi, tau_theta, tau_psi, F
    tau_phi   = data1.tau_phi
    tau_theta = data1.tau_theta
    tau_psi   = data1.tau_psi
    F         = data1.F
    # print tau_psi

def derivs(state, t):

    global tau_phi, tau_theta, tau_psi, F

    # print tau_psi

    dydx = np.zeros_like(state)
    
    pn    = state[0]
    pe    = state[1]
    pd    = state[2]
    u     = state[3]
    v     = state[4]
    w     = state[5]
    phi   = state[6]
    theta = state[7]
    psi   = state[8]
    p     = state[9]
    q     = state[10]
    r     = state[11]

    J1 = ((Jy-Jz)/Jx)
    J2 = ((Jz-Jx)/Jy)
    J3 = ((Jx-Jy)/Jz)

    a11   = cos(theta)*cos(psi)
    a12   = sin(phi)*sin(theta)*cos(psi) - cos(phi)*sin(psi)
    a13   = cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi)

    a21   = cos(theta)*sin(psi)
    a22   = sin(phi)*sin(theta)*sin(psi) + cos(phi)*cos(psi)
    a23   = cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi)

    a31   = sin(theta)
    a32   = -sin(phi)*cos(theta)
    a33   = -cos(phi)*cos(theta)

    f_x   = -m*g*sin(theta)
    f_y   =  m*g*cos(theta)*sin(phi)
    f_z   =  m*g*cos(theta)*cos(phi) - F

    dydx[0]  = a11*u + a12*v + a13*w
    dydx[1]  = a21*u + a22*v + a23*w
    dydx[2]  = a31*u + a32*v + a33*w
    dydx[3]  = (r*v - q*w)   + (f_x/m)
    dydx[4]  = (p*w - r*u)   + (f_y/m)
    dydx[5]  = (q*u - p*v)   + (f_z/m)
    dydx[6]  = p + sin(phi)*tan(theta)*q + cos(phi)*tan(theta)*r
    dydx[7]  = cos(phi)*q - sin(phi)*r
    dydx[8]  = (sin(phi)/cos(theta))*q + (cos(phi)/cos(theta))*r
    dydx[9]  = J1*q*r + (tau_phi/Jx)
    dydx[10] = J2*p*r + (tau_theta/Jy)
    dydx[11] = J3*p*q + (tau_psi/Jz)

    return dydx


# create a time array from 0..100 sampled at 0.05 second steps
dt = 0.05
endtime = 0.1
t = np.arange(0.0, endtime, dt)
# print t
        
def main():
    global state
    rospy.init_node('motion_model_quad', anonymous=True)
  
    # x_new = integrate.odeint(derivs, state, t)

    r1 = rospy.Rate(5)
    while not rospy.is_shutdown():
        sub1 = rospy.Subscriber("/atti_adj" , attitude_ctrl, callback1)

	x_new = integrate.odeint(derivs, state, t)
	state = x_new[int(endtime/dt-1),:]

	# print state

	for i in xrange(0, int(endtime/dt-1)):
		out = motion_model_quad()
		out.pn    = x_new[:,0].item(i)
		out.pe    = x_new[:,1].item(i)
		out.h     = x_new[:,2].item(i)
		out.u     = x_new[:,3].item(i)
		out.v     = x_new[:,4].item(i)
		out.w     = x_new[:,5].item(i)
		out.phi   = x_new[:,6].item(i)
		out.theta = x_new[:,7].item(i)
		out.psi   = x_new[:,8].item(i)
		out.p     = x_new[:,9].item(i)
		out.q     = x_new[:,10].item(i)
		out.r     = x_new[:,11].item(i)
		

		tinfo = time_info()
	        tinfo.dt      = dt
	        tinfo.endtime = endtime
	        tinfo.i       = i

		# print tau_psi

		pub1 = rospy.Publisher('/quad_states', motion_model_quad, queue_size = 15)
		pub2 = rospy.Publisher('/quad_func_time', time_info, queue_size = 15)
		pub1.publish(out)
		pub2.publish(tinfo)
		r1.sleep()

    rospy.loginfo("Motion model Node Has Shutdown.")
    rospy.signal_shutdown(0)

if __name__ == '__main__':
    main()

