#!/usr/bin/env python

import sys
import time
import rospy
import rospkg
import numpy                as np
import scipy.linalg         as la
import matplotlib.pyplot    as plt
import matplotlib.animation as animation

from math         import *
from scipy        import integrate
from numpy        import sin, cos, tan
from std_msgs.msg import String
from numpy.linalg import inv
from mod_ctrl.msg import motion_model_quad
from mod_ctrl.msg import time_info
from mod_ctrl.msg import attitude_ctrl

start_time   = time.time()
elapsed_time = start_time
flag         = 0

# Commanded States

phi_c   =  (0.0)*(np.pi)/180
theta_c =  (0.0)*(np.pi)/180
psi_c   =  (30.0)*(np.pi)/180

# Mass of UAV
m       =  1.56    #kg
g       =  9.80665 # m/s^2

# Inertia
Jx      =  0.114700
Jy      =  0.057600
Jz      =  0.171200

# Quad States (Actual)
pn      =  0.0
pe      =  0.0
h       =  0.0
u       =  0.0
v       =  0.0
w       =  0.0
phi     =  0.0
theta   =  0.0
psi     =  0.0
p       =  0.0
q       =  0.0
r       =  0.0

# Initialize Time Information
dt 	 =  1
endtime  =  0
i        =  0
tSize    =  10


# Initialize Errors in Actual and Commanded States
err_phi   =  (phi_c   - phi  )
err_theta =  (theta_c - theta)
err_psi   =  (psi_c   - psi  )

def callback1(data1):
    global phi, theta, psi, p, q, r, phi_c, theta_c, psi_c, err_phi, err_theta, err_psi
    phi       =  data1.phi
    theta     =  data1.theta
    psi       =  data1.psi
    p 	      =  data1.p
    q 	      =  data1.q
    r 	      =  data1.r

    err_phi   = (phi_c   - phi  )
    err_theta = (theta_c - theta)
    err_psi   = (psi_c   - psi  )
    # print (180*psi)/np.pi


def callback2(data2):
    global dt, endtime, i, tSize
    dt        = data2.dt
    endtime   = data2.endtime
    i         = data2.i
    tSize     = int(endtime/dt)
    # print dt, endtime, i

def commndedAngles():
    global phi_c, theta_c, psi_c, start_time, elapsed_time
    elapsed_time = time.time() - start_time

    if (elapsed_time < 10.0):
	phi_c   =  (0.0)*(np.pi)/180
	theta_c =  (-10.0)*(np.pi)/180
	psi_c   =  (0.0)*(np.pi)/180
    elif (elapsed_time < 20.0):
	phi_c   =  (0.0)*(np.pi)/180
	theta_c =  (10.0)*(np.pi)/180
	psi_c   =  (0.0)*(np.pi)/180
    elif (elapsed_time < 30.0):
	phi_c   =  (5.0)*(np.pi)/180
	theta_c =  (0.0)*(np.pi)/180
	psi_c   =  (0.0)*(np.pi)/180
    elif (elapsed_time < 40.0):
	phi_c   =  (-5.0)*(np.pi)/180
	theta_c =  (0.0)*(np.pi)/180
	psi_c   =  (0.0)*(np.pi)/180
    elif (elapsed_time < 50.0):
	phi_c   =  (0.0)*(np.pi)/180
	theta_c =  (0.0)*(np.pi)/180
	psi_c   =  (45.0)*(np.pi)/180
    else:
	phi_c   =  (0.0)*(np.pi)/180
	theta_c =  (0.0)*(np.pi)/180
	psi_c   =  (0.0)*(np.pi)/180

def main():
    global phi_c, theta_c, psi_c, flag, elapsed_time

    try:
	flag    = int(sys.argv[1])
    except:
	flag    = 0

    print flag

    if (flag == 1):
        try:
            phi_c   =  sys.argv[2]
	    phi_c   =  (float(phi_c))*(np.pi)/180
        except:
            phi_c   =  (0.0)*(np.pi)/180
        try:
            theta_c =  sys.argv[3]
	    theta_c =  (float(theta_c))*(np.pi)/180
        except:
            theta_c =  (0.0)*(np.pi)/180
        try:
            psi_c   =  sys.argv[4]
	    psi_c   =  (float(psi_c))*(np.pi)/180
        except:
            psi_c   =  (0.0)*(np.pi)/180
#    elif (flag == 2):
#	commndedAngles()
#    else:
#	phi_c   =  (0.0)*(np.pi)/180
#	theta_c =  (0.0)*(np.pi)/180
#	psi_c   =  (30.0)*(np.pi)/180


    zeta          = 0.707

    err_max_phi   = 30*(pi/180)
    err_max_theta = 30*(pi/180)
    err_max_psi   = 2*pi

    tau_phi_max   = 0.01
    tau_theta_max = 0.01
    tau_psi_max   = 1.0

    kp_phi        = tau_phi_max/err_max_phi
    kp_theta      = tau_theta_max/err_max_theta
    kp_psi        = tau_psi_max/err_max_psi

    wn_phi        = 1*sqrt(kp_phi/Jx)
    wn_theta      = 1*sqrt(kp_theta/Jy)
    wn_psi        = 1*sqrt(kp_psi/Jz)

    kd_phi        = 2*zeta*wn_phi*Jx
    kd_theta      = 2*zeta*wn_theta*Jy
    kd_psi        = 2*zeta*wn_psi*Jz

    ki_phi        = 0.01
    ki_theta      = 0.01
    ki_psi	  = 0.01


    rospy.init_node('attitude_controller', anonymous=True)
    r1 = rospy.Rate(5)
    while not rospy.is_shutdown():

	if (flag == 2):
	    commndedAngles()

	print (180*phi_c)/np.pi, (180*theta_c)/np.pi, (180*psi_c)/np.pi, elapsed_time

	sub1 = rospy.Subscriber("/quad_states" , motion_model_quad, callback1)
	sub2 = rospy.Subscriber("/quad_func_time", time_info        , callback2)

        tau_phi   = kp_phi*err_phi     - kd_phi*p     + ki_phi*dt*err_phi
	tau_theta = kp_theta*err_theta - kd_theta*q   + ki_theta*dt*err_theta
	tau_psi   = kp_psi*err_psi     - kd_psi*r     + ki_psi*dt*err_psi
        thrust    = m*g/(cos(theta)*cos(phi))

	atti_ctrl           = attitude_ctrl()
	atti_ctrl.tau_phi   = tau_phi
	atti_ctrl.tau_theta = tau_theta
	atti_ctrl.tau_psi   = tau_psi
        atti_ctrl.F         = thrust

	# print (180*err_phi)/np.pi, tau_phi
	# print (180*err_theta)/np.pi, tau_theta
	# print (180*err_psi)/np.pi, tau_psi

	pub  = rospy.Publisher('/atti_adj', attitude_ctrl, queue_size = 10)
        pub.publish(atti_ctrl)

        r1.sleep()

    rospy.loginfo("Attitude Controller Node Has Shutdown.")
    rospy.signal_shutdown(0)

if __name__ == '__main__':

    main()

