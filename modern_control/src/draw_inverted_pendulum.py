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
from modern_control.msg import motion_model_inv
from modern_control.msg import iter_info

g = 9.8  # acceleration due to gravity, in m/s^2
l = 1.0  # length of pendulum 1 in m
#L2 = 1.0  # length of pendulum 2 in m
M1 = 2.0  # cart mass
M2 = 1.0  # pendulum mass
u = 0.0

# th1 and th2 are the initial angles (degrees)
# w10 and w20 are the initial angular velocities (degrees per second)
y = 1
theta = 0.2
ydot = 0.05
thetadot = 0.0

gap = 0.5
width = 2
height = 1

dt = 1
time_end = 0
i = 0
time_tot = 2

fig = plt.figure()
ax = fig.add_subplot(111, autoscale_on=False, xlim=(-5, 5), ylim=(-5, 5))
ax.grid()
line1, = ax.plot([], [], 'o-', lw=2)
line2, = ax.plot([], [], '-', lw=2)

time_template = 'time = %.1fs'
time_text = ax.text(0.05, 0.9, '', transform=ax.transAxes)

def callback(data):
	global y, theta
	y = data.y
	theta = data.theta
	print y

def callback2(data2):
	global dt, time_end, i
	dt = data2.dt
	time_end = data2.end_time
	#print time_end
	i = data2.i
	time_tot = int(time_end/dt)
	#print time_tot

def init():
    	line1.set_data([], [])
	line2.set_data([], [])
    	time_text.set_text('')
    	return line1, line2, time_text

def animate1(i):
    	thisx = [y, y+l*sin(theta)]
    	thisy = [gap+height, gap + height + l*cos(theta)]
	line1.set_data(thisx, thisy)
    	time_text.set_text(time_template % (i*dt))
    	return line1, time_text

def animate2(i):
    	thisx = [y-width/2, y+width/2, y+width/2, y-width/2, y-width/2]
    	thisy = [gap, gap, gap+height, gap+height, gap]
	line2.set_data(thisx, thisy)
    	time_text.set_text(time_template % (i*dt))
    	return line2, time_text

if __name__ == '__main__':
    rospy.init_node('draw_inverted_pendulum', anonymous=True)
    r = rospy.Rate(20)
    #rospy.loginfo("hello")
    while not rospy.is_shutdown():
	sub1 = rospy.Subscriber("/states_inv", motion_model_inv, callback)
	sub2 = rospy.Subscriber("/iter_inform", iter_info, callback2)
	#rospy.loginfo("hello")
	ani1 = animation.FuncAnimation(fig, animate1, np.arange(1, time_tot), interval=25, blit=True, init_func=init)
	ani2 = animation.FuncAnimation(fig, animate2, np.arange(1, time_tot), interval=25, blit=True, init_func=init)
	plt.show()
	#r.sleep()
    rospy.loginfo("Motion model Node Has Shutdown.")
    rospy.signal_shutdown(0)








	



