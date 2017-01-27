#!/usr/bin/env python
# license removed for brevity
import rospy
from  math import *
import rospkg
import numpy as np
import scipy.linalg as la
import matplotlib.pyplot as plt
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

n=4 #number of waypoints
xw =np.array([10, 10, 5.54, 5.54] )
yw =np.array([5.54, 10, 10, 5.54] )
psidot =0;
xn = 0
yn = 0
count=0
wpcounter=0
#siData=np.zeros([1,100000])


def callback(data):
    global psidot, xn, yn, count  
    #global siData
    global wpcounter
    global xw, yw
    xn = data.x
    yn = data.y
    psi = data.theta
    psi_wrap = ( psi + np.pi) % (2 * np.pi ) - np.pi
    #psi_wrap = np.arctan2(np.sin(psi), np.cos(psi))
    x_diff = xw.item((wpcounter)) - xn
    y_diff = yw.item((wpcounter)) - yn
    psid = np.arctan2(y_diff,x_diff)
    kp = 1;
    psidot = kp*(psid-psi)
    #siData[0,count]=psidot
    count=count+1
    #print(n)
    #pub.publish(psidot)

if __name__ == '__main__':
    #print(psidot)
    rospy.init_node('control', anonymous=True)
    r = rospy.Rate(1)
    while not rospy.is_shutdown():
        #print(siData)
	#sub = rospy.Subscriber("/turtle1/pose_turtle", Pose, callback) #motion model
	sub = rospy.Subscriber("/turtle1/pose", Pose, callback) #turtlesim
        cmd = Twist()
        cmd.linear.x = 0.5
        cmd.linear.y = 0
        cmd.linear.z = 0
        cmd.angular.x = 0
        cmd.angular.y = 0
        cmd.angular.z = psidot
        print(wpcounter)
        if abs(xw.item((wpcounter)) - xn) < 0.5 and abs(yw.item((wpcounter)) - yn) < 0.5:
            wpcounter=wpcounter+1
            if wpcounter == 4:
		wpcounter=0			

            #rospy.loginfo("Reached Waypoint - shutting down")
    	    #rospy.signal_shutdown(0)

        pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size = 10)
        #rospy.loginfo(cmd.angular.z) ## prints out whatever we want to 
        pub.publish(cmd)
        #rospy.spin()
        r.sleep()
    rospy.loginfo("Controller Node Has Shutdown.")
    rospy.signal_shutdown(0)
    
