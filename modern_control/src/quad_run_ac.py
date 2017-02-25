#!/usr/bin/env python

# Double pendulum formula translated from the C code at
# http://www.physics.usyd.edu.au/~wheat/dpend_html/solve_dpend.c

import rospy
import rospkg
import numpy                       as np
import matplotlib.pyplot           as plt
import scipy.integrate             as integrate
import matplotlib.animation        as animation
import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib                  as mpl

from math                 import *
from numpy                import sin, cos, tan
from mpl_toolkits.mplot3d import Axes3D
from mod_ctrl.msg         import motion_model_quad
from mod_ctrl.msg         import time_info

pn       =  0.0025
pe       =  0
pd       =  -0.005
phi      =  0
theta    =  0
psi      =  0

dt 	 =  1
endtime  =  0
i        =  0
tSize    =  10


def callback1(data1):
    global pn, pe, pd, phi, theta, psi
    pn       =  data1.pn
    pe       =  data1.pe
    pd       = -data1.h
    phi      =  data1.phi
    theta    =  data1.theta
    psi      =  data1.psi
    # print y
    # print theta


def callback2(data2):
    global dt, endtime, i, tSize
    dt        = data2.dt
    endtime   = data2.endtime
    i         = data2.i
    tSize     = int(endtime*1000/dt)
    # print dt, endtime, i


def init():
    line.set_data([], [])
    line.set_3d_properties([])
    # line2.set_data([], [])
    time_text.set_text('')
    return line, time_text

def rotMat(phi, theta, psi):
    R_roll  = np.matrix([[1, 0, 0], [0, cos(phi), sin(phi)], [0, -sin(phi), cos(phi)]])

    R_pitch = np.matrix([[cos(theta), 0, -sin(theta)], [0, 1, 0], [sin(theta), 0, cos(theta)]])

    #R_yaw  = np.matrix([[cos(psi), sin(psi), 0], [-sin(psi), cos(psi), 0], [0, 0, 1]])
    R_yaw   = np.matrix([[cos(psi),-sin(psi), 0], [ sin(psi), cos(psi), 0], [0, 0, 1]])
    
    Rot     = np.dot(np.dot(R_roll,R_pitch),R_yaw)
    #Rot     = np.transpose(Rot)
    return Rot

def animate(i):    
    #thisn  = [6,-1,-1, 6]
    #thise  = [0,-1, 1, 0]
    #thisd  = [0, 0, 0, 0]
    
    rMat   = rotMat(phi, theta, psi)
    vert   = np.matrix([[6,-1,-1,6],[0,-1,+1,0], [0, 0, 0, 0]])
    #vert   = np.matrix([[pn+6,pn-1,pn-1,pn+6],[pe,pe-1,pe+1,pe],[pd,pd,pd,pd]]) # WRONG

    newVer = np.dot(rMat,vert)

    newX   = newVer[1,:] + pe
    newX   = np.squeeze(np.asarray(newX))

    newY   = newVer[0,:] + pn
    newY   = np.squeeze(np.asarray(newY))

    newZ   = newVer[2,:] + pd
    newZ   = np.squeeze(np.asarray(newZ))

    # print newX

    # line.set_data(thise, thisn)
    # line.set_3d_properties(thisd)

    line.set_data(newX, newY)
    line.set_3d_properties(newZ)

    time_text.set_text(time_template % (i*dt))
    return line, time_text


# Attaching 3D axis to the figure
fig = plt.figure()
ax  = p3.Axes3D(fig)
ax.grid()

line, = ax.plot([], [], [], 'o-', lw=2)


# Setting the axes properties
ax.set_xlim3d([-10.0, 10.0])
ax.set_xlabel('X')

ax.set_ylim3d([-10.0, 10.0])
ax.set_ylabel('Y')

ax.set_zlim3d([-10.0, 10.0])
ax.set_zlabel('Z')

ax.set_title('3D Test')

time_template = 'time = %.1fs'
time_text     = ax.text(0.05, 0.9, 5.0,'', transform=ax.transAxes)

def main():
    rospy.init_node('quad_run_ac', anonymous=True)
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
	sub  = rospy.Subscriber("/quad_states" , motion_model_quad, callback1)
	sub2 = rospy.Subscriber("/quad_func_time", time_info        , callback2)

        ani1 = animation.FuncAnimation(fig, animate, np.arange(1, tSize), interval=25, blit=True, init_func=init)

	# ani2 = animation.FuncAnimation(fig, animate2, np.arange(1, tSize), interval=25, blit=True, init_func=init)

        # ani.save('double_pendulum.mp4', fps=15)
        plt.show()
	#r.sleep()

if __name__ == '__main__':
    main()
