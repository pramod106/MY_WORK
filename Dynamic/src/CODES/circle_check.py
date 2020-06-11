#!/usr/bin/env python
from __future__ import division
import math 
import rospy
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry
import numpy as np
import time
import matplotlib.pyplot as plt 
# from std_msgs.msg import Float64MultiArray
np.seterr(divide='ignore', invalid='ignore')
from tf.transformations import euler_from_quaternion, quaternion_from_euler
global xgps, ygps
global roll, pitch, yaw, xacts, yact

def datagps(data):
    global roll , pitch , yaw ,xact , yact ,vact
    roll, pitch, yaw = 0.0, 0.0, 0.0 
    xact, yact = 0.0, 0.0
    vact = 0.0
    orintation_q = data.pose.pose.orientation
    orientation_list = [orintation_q.x, orintation_q.y,orintation_q.z,orintation_q.w]
    (roll , pitch, yaw) = euler_from_quaternion(orientation_list)
    xact =  data.pose.pose.position.x
    yact = data.pose.pose.position.y
    vact = data.twist.twist.linear.x 

if __name__ == "__main__":
    global xact, yact, yaw, vact
    xact, yact, yaw = 10.0, 0.0, 0.0
    vact = 0.0 
    vref = 8
    radius = 5 
    omega_ref_old = vref/radius

    rospy.init_node('check',anonymous=True) 
    rospy.Subscriber('/agent_0/odom', Odometry, datagps)
    pub_speed = rospy.Publisher('/agent_0/cmd_vel',Twist,queue_size=10)
    speed_car = Twist()
    # time.sleep(2)
    time_init = rospy.get_time()

    while not rospy.is_shutdown():

        global xact, yact, yaw, vact
        theta_act = yaw                                # actual Theta 

        speed_car.linear.x = vref                 # Control linear velocity 
        speed_car.angular.z = omega_ref_old       # Control angular velocity
        pub_speed.publish(speed_car)                     
        print xact, yact 
        time.sleep(0.05)