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
global xgps
global ygps
global get_time
#***************2nd******************
c1 = 1
c2 = 0.0001
c3 = 0.01
#****************1st*******************
K_x = 0.5
K_y = 0.0025
K_theta = 0.025
#****************3rd ******************
k_x3 = 20 
k_theta3 = 5
k_y3 = 40 

#********************************************** GPS Callback Function ****************************************************************************************************
def gps(data):
    global xgps
    global ygps
    global get_time
    # global current_speed
    xgps=0
    ygps=0
    get_time = 0 
    # current_speed = 0
    xgps = data.pose.pose.position.x
    ygps = data.pose.pose.position.y
    get_time = data.pose.pose.position.z
    # current_speed = data.twist.twist.linear.x
#*************************************************************************************************************************************************************************
if __name__ == "__main__":
    global xgps
    global ygps
    xgps = 50
    ygps = 0
    global get_time  
    get_time = 0
    y_act_old = 0
    x_act_old = 50
    theta_act = 0.0 
    wheelBase = 2.995
    x_old = 50
    y_old = 0 
    tu_old = 0 
    vref = 10
    radius = 50
    omega_ref_old = vref/radius 
    xe = 0.0
    ye =0.0
    theta_e = 0.0
    theta_ref_old = 1.5708
    theta_ref_new = 1.5708
    theta_act_old = 1.5708 
    theta_new = 1.5708
    y_act = 0.0  
    x_act = 50
    rospy.init_node('step',anonymous=True) 
    rospy.Subscriber('/odom',Odometry , gps)
    pub_speed = rospy.Publisher('/car_speed',Twist,queue_size=10)
    speed_car = Twist()
    # speed_car.linear.x = 5
    # speed_car.angular.z = math.atan2(wheelBase,50)
    # pub_speed.publish(speed_car)
    while(True):
        global get_time
        global flag
        tu = get_time
        # print tu 
        deltat = tu - tu_old                
        global xgps
        global ygps
        x_act = round(xgps,7)
        # print x_act
        y_act = round(ygps,7)
        # print deltat
        print x_act , y_act
        if((deltat == 0)):
            # theta_act = theta_act_old
            x_new = x_old
            y_new = y_old
        elif( deltat > 0 and (x_act-x_act_old) > 0.00001 ):
            theta_new = theta_ref_old + deltat*(omega_ref_old)
            x_new = radius*math.cos(theta_new)
            y_new = radius*math.sin(theta_new)
            
            # print ((x_act - x_act_old)/(deltat))
            # x_new = x_old + deltat*(-y_old*omega_ref_old)
            # y_new = y_old + deltat*(x_old*omega_ref_old)            
            # plt.plot(y_new , x_new , color= 'green')
            # plt.grid()
            # plt.pause(0.0001)
            # print x_new , y_new , x_act, y_act
            theta_ref_new = 1.5708 + math.atan2(y_new,x_new)
            # theta_ref_new = 3.14 - (math.atan((x_new)/(y_new)))
            theta_act = math.atan((y_act - y_act_old)/(x_act-x_act_old))

            # print theta_act
            # theta_act = 1.5708 + math.atan2(y_act,x_act)
            # theta_act = 3.14 - (math.atan((x_act)/(y_act)))
            # print (math.degrees(theta_new))
            # print (math.degrees(theta_ref_new))
            # print (math.degrees(theta_act))            
            # omega_ref = (theta_ref_new - theta_ref_old)/deltat    
        #************************************************************
        xe = x_new - x_act
        # xe = float(math.cos(theta_act)*(x_new - x_act) + math.sin(theta_act)*(y_new - y_act))
        ye = y_new - y_act
        # ye = float(-(math.sin(theta_act)*(x_new - x_act)) + math.cos(theta_act)*(y_new - y_act))
        theta_e = theta_ref_new - theta_act
        #************************************************************        
        # print xe , ye , theta_e
        # print "*************************"
        # control_velocity = vref*math.cos(theta_e) + K_x*xe #1st
        control_velocity = (vref) + ((c1*xe)/(math.sqrt( 1 + (pow(xe,2)) + (pow(ye,2))))) #2nd
        # control_velocity = vref*math.cos(theta_e) + k_x3*xe # 3rd 
        # print control_velocity
        # controlled_omega = max(min(omega_ref_old + (vref)*(K_y*ye + K_theta*math.sin(theta_e)),0.7157 ),-0.7157) #1st
        # controlled_omega = omega_ref_old + (vref)*(K_y*ye + K_theta*math.sin(theta_e))#1st method
        # controlled_omega = max(min(omega_ref_old + ( ((c2*vref)*(ye*math.cos(theta_e/2) - xe*math.sin(theta_e/2) ))/( math.sqrt( 1 + (pow(xe,2)) + (pow(ye,2)))) ) + c3*math.sin(theta_e/2) ,0.7157) , -0.7157) #2nd
        # controlled_omega = max(min((omega_ref_old + k_theta3*math.cos(theta_e) + vref*k_y3*ye*(math.sin(theta_e)/(theta_e)) ) ,0.7157) -0.7157 ) #3rd 
        controlled_omega = omega_ref_old + ( ((c2*vref)*(ye*math.cos(theta_e/2) - xe*math.sin(theta_e/2) ))/( math.sqrt( 1 + (pow(xe,2)) + (pow(ye,2)))) ) + c3*math.sin(theta_e/2)#2nd 
        # print controlled_omega
        # delta = max(min(math.atan((wheelBase*controlled_omega)/(control_velocity)),0.349066),-0.349066) # delta = steering angle
        delta = math.atan((wheelBase*controlled_omega)/(control_velocity))
        # delta = 0.261799
        # print math.degrees(delta) 
        # print "******************************"
        speed_car.linear.x = control_velocity 
        speed_car.angular.z = delta
        pub_speed.publish(speed_car)
        # x_old = x_new
        # y_old = y_new
        theta_ref_old = theta_new
        # print theta_act
        # print "*********************"
        x_act_old = x_act
        y_act_old = y_act
        tu_old = tu
        theta_act_old = theta_act  
          
        # theta_ref_old = theta_ref_new
        # omega_ref_old = omega_ref
        time.sleep(0.032)  