#!/usr/bin/env python
from __future__ import division
import math 
import rospy
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry
import numpy as np
import time
np.seterr(divide='ignore', invalid='ignore')
from tf.transformations import euler_from_quaternion, quaternion_from_euler
global xgps, ygps, get_time, roll, pitch, yaw, xact, yact

c1, c2, c3 = 2, 0.01, 1.1

#********************************************** GPS Callback Function ****************************************************************************************************
def datagps(data):
    global roll , pitch , yaw ,xact , yact, vact, wact
    roll, pitch, yaw, xact, yact = 0.0, 0.0, 0.0, 0.0, 0.0
    orintation_q = data.pose.pose.orientation
    orientation_list = [orintation_q.x, orintation_q.y,orintation_q.z,orintation_q.w]
    (roll , pitch, yaw) = euler_from_quaternion(orientation_list)
    xact =  data.pose.pose.position.x
    yact = data.pose.pose.position.y
#*************************************************************************************************************************************************************************

if __name__ == "__main__":
    global xact, yact, yaw, vact, wact 
    xact, yact, yaw, vact, wact  = 60, 0.0, 0.0, 0.0 , 0.0
    x_ref, y_ref = 60.0 , 0.0 
    vref, major_x, minor_y = 8, 60.0, 40.0  
    x_refold, y_refold, theta_old = 60.0, 0.0, 0.0
    rospy.init_node('check',anonymous=True) 
    rospy.Subscriber('/agent_0/odom', Odometry, datagps)
    pub_speed = rospy.Publisher('/agent_0/cmd_vel',Twist,queue_size=10)
    r = rospy.Rate(50)
    speed_car = Twist()
    rospy.sleep(5.0)
    time_init = rospy.get_time()

    while True:
        time_curr = rospy.get_time()
        deltat = time_curr - time_init
        theta_dot_up = vref/( math.sqrt( pow(major_x ,2) + pow(minor_y,2) - pow(x_ref,2) - pow(y_ref,2) ))
        theta_up = theta_old + deltat*theta_dot_up
        global xact, yact, yaw, vact, wact

        x_ref = major_x*math.cos(theta_up)
        y_ref = minor_y*math.sin(theta_up)
        
        theta_act = yaw
        if theta_act< 0:
            theta_act += 2*math.pi
        
        theta_ref_new = math.atan2( (y_ref - y_refold), (x_ref - x_refold) )
        if theta_ref_new<0:
            theta_ref_new += 2*math.pi
        #*************************************************************************************************
        xe = float(math.cos(theta_act)*(x_ref - xact) + math.sin(theta_act)*(y_ref - yact))        
        ye = float(-(math.sin(theta_act)*(x_ref - xact)) + math.cos(theta_act)*(y_ref - yact))
        theta_e = theta_ref_new - theta_act
        #*************************************************************************************************
        # print x_ref, y_ref, xact, yact  
        

        if theta_e < -3.14:
            theta_e += 2*math.pi
        elif theta_e > 3.14:
            theta_e -= 2*math.pi
        print xe, ye, theta_e, vact, wact, time_curr
        control_velocity = (vref) + ((c1*xe)/(math.sqrt( 1 + (pow(xe,2)) + (pow(ye,2)))))
        
        omega_ref_old = ( (major_x*minor_y)*theta_dot_up )/( pow(major_x,2) + pow(minor_y ,2) - pow(x_ref,2) - pow(y_ref,2) )

        controlled_omega = omega_ref_old + ( ((c2*vref)*(ye*math.cos(theta_e/2) - xe*math.sin(theta_e/2) ))/( math.sqrt( 1 + (pow(xe,2)) + (pow(ye,2)))) ) + c3*math.sin(theta_e/2)
        
        #*************************************************************************************************
        speed_car.linear.x = control_velocity
        speed_car.angular.z = controlled_omega
        pub_speed.publish(speed_car)

        y_refold = y_ref
        x_refold = x_ref      
        theta_old = theta_up
        time_init = time_curr
        r.sleep()