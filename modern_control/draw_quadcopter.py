#!/usr/bin/env python

import rospy
from  math import *
import rospkg
import numpy as np
import scipy.linalg as la
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib as mpl
import mpl_toolkits.mplot3d.axes3d as p3

from scipy import integrate
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from modern_control.msg import motion_model_quad
from modern_control.msg import iter_info
from mpl_toolkits.mplot3d import Axes3D

pn = 0.0025
pe = 0
pd = -0.005
u = 0
v = 0
w = 0
phi = 0
theta = 0
psi = 0
p = 0
q = 0
r = 0

dt = 1
end_time = 0
i = 0
tsize = 2

fig = plt.figure()
ax = p3.Axes3D(fig)
ax.grid()
line, = ax.plot([], [], [], 'o-', lw=2)

ax.set_xlim3d([-10.0, 10.0])
ax.set_xlabel('X')

ax.set_ylim3d([-10.0, 10.0])
ax.set_ylabel('Y')

ax.set_zlim3d([-10.0, 10.0])
ax.set_zlabel('Z')

def callback(data):
	global pn, pe, pd, phi, theta, psi
	pn = data.pn
	pe = data.pe
	pd = -data.h
	phi = data.phi
	theta = data.theta
	psi = data.psi
	
def callback2(data2):
	global dt, end_time, i
	dt = data2.dt
	time_end = data2.end_time
	i = data2.i
	time_tot = int(time_end/dt)
	
def init():
    	line.set_data([], [])
        line.set_3d_properties([])
    	return line,

def rotMat(phi, theta, psi):
    R_roll  = np.matrix([[1, 0, 0], [0, cos(phi), sin(phi)], [0, -sin(phi), cos(phi)]])

    R_pitch = np.matrix([[cos(theta), 0, -sin(theta)], [0, 1, 0], [sin(theta), 0, cos(theta)]])

    #R_yaw  = np.matrix([[cos(psi), sin(psi), 0], [-sin(psi), cos(psi), 0], [0, 0, 1]])
    R_yaw   = np.matrix([[cos(psi),-sin(psi), 0], [ sin(psi), cos(psi), 0], [0, 0, 1]])
    
    Rot     = np.dot(np.dot(R_roll,R_pitch),R_yaw)
    #Rot     = np.transpose(Rot)
    return Rot

def animate(i):
    thise  = [pe  , pe-1, pe+1, pe  ]
    thisn  = [pn+6, pn-1, pn-1, pn+6]
    thisd  = [pd  , pd  , pd  , pd  ]
    
    rMat   = rotMat(phi, theta, psi)
    #vert   = np.matrix([[pe, pe-1, pe+1, pe], [pn+6, pn-1, pn-1, pn+6], [pd, pd, pd, pd]])
    vert   = np.matrix([[pn+6, pn-1, pn-1, pn+6],[pe, pe-1, pe+1, pe], [pd, pd, pd, pd]])

    test1  = np.matrix([[1,2,3],[4,5,6],[7,8,9]])
    test2  = np.matrix([[1,0,0,1],[0,2,0,0],[0,0,1,2]])

    newVer = np.dot(rMat,vert)
    #newVer = np.dot(test1,test2)

    newX   = newVer[1,:]
    newX   = np.squeeze(np.asarray(newX))

    newY   = newVer[0,:]
    newY   = np.squeeze(np.asarray(newY))

    newZ   = newVer[2,:]
    newZ   = np.squeeze(np.asarray(newZ))

    # print newX

    # line.set_data(thise, thisn)
    # line.set_3d_properties(thisd)

    line.set_data(newX, newY)
    line.set_3d_properties(newZ)

    return line,

if __name__ == '__main__':
    rospy.init_node('draw_quadcopter', anonymous=True)
    r1 = rospy.Rate(5)
    while not rospy.is_shutdown():
	sub1 = rospy.Subscriber("/states_quad", motion_model_quad, callback)
	sub2 = rospy.Subscriber("/iter_inform", iter_info, callback2)
	ani = animation.FuncAnimation(fig, animate, np.arange(1, tsize), interval=25, blit=True, init_func=init)
	plt.show()
	#r.sleep()
    rospy.loginfo("Motion model Node Has Shutdown.")
    rospy.signal_shutdown(0)


