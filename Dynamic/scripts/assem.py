#!/usr/bin/env python
from __future__ import division
import math 
import rospy
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry
import numpy as np
from std_msgs.msg import Float64MultiArray
import time
np.seterr(divide='ignore', invalid='ignore')
from tf.transformations import euler_from_quaternion, quaternion_from_euler
global xgps, ygps, get_time, roll, pitch, yaw, xact, yact

# c1, c2, c3 = 2, 0.01, 1 
K_I, K_TH = 25, 0.1

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
    global xact, yact, yaw , vact, wact 
    xact, yact, yaw, vact, wact = 0.0, 0.0, 0.0, 0.0, 0.0
    vref = 8
    Radius = 40.0
    xact, yact, theta_old = Radius , 0.0, 0.0
    x_ref, y_ref, theta_up = Radius , 0.0, 0.0

    rospy.init_node('check',anonymous=True) 
    rospy.Subscriber('/agent_0/odom', Odometry, datagps)
    pub_speed = rospy.Publisher('/agent_0/cmd_vel',Twist,queue_size=10)
    pub_graph_data = rospy.Publisher('/graph_data', Float64MultiArray, queue_size= 100)
    graph_data = Float64MultiArray()
    speed_car = Twist()
    rospy.sleep(5.0)
    time_init = rospy.get_time()

    while True:
        global xact, yact, yaw, vact, wact
        time_curr = rospy.get_time()
        deltat = time_curr - time_init
        theta_dot_up =  vref/Radius
        theta_up = theta_old + deltat*theta_dot_up        

        x_ref = Radius*math.cos(theta_up)
        y_ref = Radius*math.sin(theta_up)
        
        theta_act = yaw
        if theta_act< 0:
            theta_act += 2*math.pi
        
        theta_ref_new = math.atan2( (y_ref - yact), (x_ref - xact) )
        if theta_ref_new<0:
            theta_ref_new += 2*math.pi
        
        #*************************************************************************************************
        xe = float(x_ref - xact)       
        ye = float(y_ref - yact)
        theta_e = theta_ref_new - theta_act
        #*************************************************************************************************
        
        if theta_e < -3.14:
            theta_e += 2*math.pi
        elif theta_e > 3.14:
            theta_e -= 2*math.pi
        
        print x_ref, y_ref, xact, yact, xe, ye, theta_e, time_curr

        control_velocity = K_I*math.cos(theta_e)*math.sqrt(pow(xe,2) + pow(ye,2)) 
        
        omega_ref = vref/Radius

        controlled_omega = omega_ref + K_TH*(theta_e)
        
        graph_data.data = [vref, control_velocity, vact, omega_ref, controlled_omega, wact , time_curr]   # Data Publish to plot
        pub_graph_data.publish(graph_data)

        #*************************************************************************************************
        speed_car.linear.x = control_velocity
        speed_car.angular.z = controlled_omega
        pub_speed.publish(speed_car) 

        theta_old = theta_up
        time_init = time_curr
        rospy.sleep(0.020)