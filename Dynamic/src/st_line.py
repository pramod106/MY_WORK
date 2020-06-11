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
    global roll , pitch , yaw ,xact , yact
    roll, pitch, yaw, xact, yact = 0.0, 0.0, 0.0, 0.0, 0.0
    orintation_q = data.pose.pose.orientation
    orientation_list = [orintation_q.x, orintation_q.y,orintation_q.z,orintation_q.w]
    (roll , pitch, yaw) = euler_from_quaternion(orientation_list)
    xact =  data.pose.pose.position.x
    yact = data.pose.pose.position.y
#*************************************************************************************************************************************************************************

if __name__ == "__main__":
    global xact, yact, yaw 
    xact, yact, yaw  = 0, 0.0, 0.0
    x_ref, y_ref = 0.0 , 0.0 
    omega_ref_old = 0.0
    vref, major_x, minor_y = 8, 0.0, 0.0  
    x_refold, y_refold, theta_old = 0.0, 0.0, 0.0 
    x_init, y_init = 0.0, 0.0
    rospy.init_node('check',anonymous=True) 
    rospy.Subscriber('/agent_0/odom', Odometry, datagps)
    pub_speed = rospy.Publisher('/agent_0/cmd_vel',Twist,queue_size=10)
    speed_car = Twist()
    time_init = rospy.get_time()

    while True:
        time_curr = rospy.get_time()
        deltat = time_curr - time_init
        global xact, yact, yaw

        x_ref = x_init + vref*(deltat)  # Ref x coordinate     
        y_ref = y_init 
        
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

        print x_ref, y_ref, xact, yact, xe, ye, theta_e, time_curr
        if theta_e < 0:
            theta_e += 2*math.pi
        elif theta_e > 0:
            theta_e -= 2*math.pi
        print theta_e
        control_velocity = (vref) + ((c1*xe)/(math.sqrt( 1 + (pow(xe,2)) + (pow(ye,2)))))      
        
        controlled_omega = omega_ref_old + ( ((c2*vref)*(ye*math.cos(theta_e/2) - xe*math.sin(theta_e/2) ))/( math.sqrt( 1 + (pow(xe,2)) + (pow(ye,2)))) ) + c3*math.sin(theta_e/2)
        
        #*************************************************************************************************
        speed_car.linear.x = control_velocity
        speed_car.angular.z = controlled_omega
        pub_speed.publish(speed_car)

        y_refold = y_ref
        x_refold = x_ref      
        # theta_old = theta_up
        # time_init = time_curr
        time.sleep(0.064)