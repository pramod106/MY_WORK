#!/usr/bin/env python
from __future__ import division
import math 
import rospy
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry
import numpy as np
import time
from tf.transformations import euler_from_quaternion, quaternion_from_euler
np.seterr(divide='ignore', invalid='ignore')
global roll , pitch , yaw , xact, yact, theta_reference , xref ,yref

roll =0 
pitch = 0
yaw = 0
xact =0 
yact =0
theta_reference = 0
xref = 0 
yref = 0 

#***************************GPS DATA CALLBACK*****************************************************************************************************************
def datagps(data):
    global roll , pitch , yaw ,xact , yact
    roll = 0
    pitch=0
    yaw = 0 
    xact = 0
    yact = 0
    orintation_q = data.pose.pose.orientation
    orientation_list = [orintation_q.x, orintation_q.y,orintation_q.z,orintation_q.w]
    (roll , pitch, yaw) = euler_from_quaternion(orientation_list)
    print yaw
    xact =  data.pose.pose.position.x
    yaw = data.pose.pose.position.y
#****************************REF DATA CALLBACK****************************************************************************************************************
def dataref(pos):
    global xref , yref, theta_reference
    xref = pos.position.x
    yref = pos.position.y
    theta_reference = pos.position.z
    
#*************************************************************************************************************************************************************

if __name__ == "__main__":
    #************** Gain values *****************
    c1 = 6
    c2 = 1
    c3 = 40
    #********************************************
    global xact , yact , xref, yref, yaw, theta_reference
    wheelBase = 2.995
    vref = 10.0
    radius = 48 
    omega_ref = vref/radius 
    rospy.init_node('controller',anonymous=True)
    rospy.Subscriber('/agent_0/odom', Odometry, datagps)
    rospy.Subscriber('/agent_0/odom_ref', Pose , dataref)
    pub_speed = rospy.Publisher('/agent_0/cmd_vel',Twist,queue_size=10)
    speed_car = Twist()
    while not rospy.is_shutdown():
        xe = float(math.cos(yaw)*(xref - xact) + math.sin(yaw)*(yref - yact))
        ye = float(-(math.sin(yaw)*(xref - xact)) + math.cos(yaw)*(yref - yact)) 
        theta_e = theta_reference - yaw
        print xe , ye
        control_velocity = vref + ((c1*xe)/(math.sqrt( 1 + (pow(xe,2)) + (pow(ye,2)))))
        
        controlled_omega = max( min(omega_ref + ( ((c2*vref*2.7)*(ye*math.cos(theta_e/2) - xe*math.sin(theta_e/2) ))/( math.sqrt( 1 + (pow(xe,2)) + (pow(ye,2)))) ) + c3*math.sin(theta_e/2) ,0.7157) , -0.7157)

        delta = max(min(math.atan((wheelBase*controlled_omega)/(control_velocity)),0.261799),-0.261799)
        speed_car.linear.x = control_velocity*3.6
        speed_car.angular.z = delta #round(delta,2) # steering angle 
        pub_speed.publish(speed_car)
        time.sleep(0.05)



