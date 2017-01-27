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
from modern_control.msg import motion_model_dp
from modern_control.msg import iter_info

G = 9.8  # acceleration due to gravity, in m/s^2
L1 = 1.0  # length of pendulum 1 in m
L2 = 1.0  # length of pendulum 2 in m
M1 = 1.0  # mass of pendulum 1 in kg
M2 = 1.0  # mass of pendulum 2 in kg
mot = 40 # motion model integrator size

x1       =  0
y1       = -1
x2       =  0
y2       = -2

dt = 1
time_end = 0
i = 0
time_tot = 2

fig = plt.figure()
ax = fig.add_subplot(111, autoscale_on=False, xlim=(-2, 2), ylim=(-2, 2))
ax.grid()
line, = ax.plot([], [], 'o-', lw=2)

time_template = 'time = %.1fs'
time_text = ax.text(0.05, 0.9, '', transform=ax.transAxes)

def callback(data):
	global x1, x2, y1, y2
	x1 = L1*sin(data.theta1)
	y1 = -L1*cos(data.theta1)
	x2 = L2*sin(data.theta2) + x1
	y2 = -L2*cos(data.theta2) + y1

def callback2(data2):
	global dt, time_end, i
	dt = data2.dt
	time_end = data2.end_time
	#print time_end
	i = data2.i
	time_tot = int(time_end/dt)
	print time_tot

def init():
    	line.set_data([], [])
    	time_text.set_text('')
    	return line, time_text

def animate(i):
    	thisx = [0, x1, x2]
    	thisy = [0, y1, y2]
	line.set_data(thisx, thisy)
    	time_text.set_text(time_template % (i*dt))
    	return line, time_text

if __name__ == '__main__':
    rospy.init_node('draw_pendulum', anonymous=True)
    r = rospy.Rate(10)
    rospy.loginfo("hello")
    while not rospy.is_shutdown():
	sub1 = rospy.Subscriber("/states_dp", motion_model, callback)
	sub2 = rospy.Subscriber("/iter_inform", iter_info, callback2)
	#rospy.loginfo("hello")
	ani = animation.FuncAnimation(fig, animate, np.arange(1, time_tot), interval=25, blit=True, init_func=init)
	plt.show()
	#r.sleep()
    rospy.loginfo("Motion model Node Has Shutdown.")
    rospy.signal_shutdown(0)








	



