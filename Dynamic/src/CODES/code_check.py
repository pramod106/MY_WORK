#!/usr/bin/env python
from __future__ import division
import math 
import rospy
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry
import numpy as np
import time
import matplotlib.pyplot as plt 
np.seterr(divide='ignore', invalid='ignore')
from tf.transformations import euler_from_quaternion, quaternion_from_euler
#**********************************GPS callback ***************************************************************************************************************************
def datagps( data):
    global roll, pitch, yaw, xact, yact
    roll = 0
    pitch=0
    yaw = 0 
    xact = 0
    yact = 0
    orintation_q = data.pose.pose.orientation
    orientation_list = [orintation_q.x, orintation_q.y,orintation_q.z,orintation_q.w]
    (roll , pitch, yaw) = euler_from_quaternion(orientation_list)
    xact =  data.pose.pose.position.x
    yact = data.pose.pose.position.y

#********************************** MAIN Loop *****************************************************************************************************************************
if __name__ == "__main__":
    global xact, yact, yaw 
    xact, yact, yaw = 80, 0, 0
    vref = 0
    radius = 100 
    omega_ref_old = vref/radius
    omega_ref = 0
    xe, ye, theta_e = 0, 0, 0
    theta = 0
    x_init, y_init = 0, 0 
    rospy.init_node( 'code_check', anonymous=True)
    rospy.Subscriber( '/agent_0/odom', Odometry, datagps)
    pub_speed = rospy.Publisher('/agent_0/cmd_vel', Twist, queue_size=10) 
    speed_car = Twist()
    time_init = rospy.get_time()
    delt = 0.0

    while not rospy.is_shutdown():
        time_curr = rospy.get_time()
        delt = time_curr - time_init
        global xact, yact, yaw 
        theta_new = theta + delt*( omega_ref_old)
        x_new = radius*math.cos(theta_new)
        y_new = radius*math.sin(theta_new)
        theta_act = yaw
        theta_ref_new = math.atan2((y_new - yact), (x_new - xact))

        #**********************************Error calculation ***************************************************************************************************************
        xe = xact - x_new
        ye = yact - y_new
        theta_e = theta_act - theta_ref_new

        #**********************************Control laws ********************************************************************************************************************
        # control_velocity = vref*math.cos(theta_e) + K_x*xe #1st
        # control_velocity = (vref) + ((c1*xe)/(math.sqrt( 1 + (pow(xe,2)) + (pow(ye,2))))) #2nd
        control_velocity = -K_I*math.cos(theta_e)*math.sqrt(pow(xe,2) + pow(ye,2)) # law from research paper  
        
        # controlled_omega = omega_ref_old + (vref)*(K_y*ye + K_theta*math.sin(theta_e)) #1st
        # controlled_omega = omega_ref_old + ( ((c2*vref)*(ye*math.cos(theta_e/2) - xe*math.sin(theta_e/2) ))/( math.sqrt( 1 + (pow(xe,2)) + (pow(ye,2)))) ) + c3*math.sin(theta_e/2)#2nd 
        controlled_omega = omega_ref - K_TH*(theta_e) # law from research paper

        #**********************************Published values*****************************************************************************************************************
        speed_car.linear.x = control_velocity
        speed_car.angular.z = controlled_omega
        pub_speed.publish(speed_car)

        #**********************************Updation ************************************************************************************************************************       
        theta = theta_new
        theta_ref_old = theta_ref_new
        time_init = time_curr
        time.sleep(0.064)


