# This code is about the generation of 5th order poly curve. Once Curve is ready then with that controller I am implementing the trajectory tracking for car on the genertated trajectory.

#!/usr/bin/env python
from __future__ import division
import math 
import rospy
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
import numpy as np
import time
np.seterr(divide='ignore', invalid='ignore')
from tf.transformations import euler_from_quaternion, quaternion_from_euler
global xgps, ygps, get_time, roll, pitch, yaw, xact, yact, vact, wact
epsilon = 1e-05
global x_last_2, x_last, x_cur, y_last_2, y_last, y_cur
x_last_2, x_last, x_cur, y_last_2, y_last, y_cur = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
#*****************************************************************************************************************************************
def datagps(data):                           # call back function for gps data
    global yaw, xact , yact, vact, wact
    roll, pitch = 0.0, 0.0
    orintation_q = data.pose.pose.orientation
    orientation_list = [orintation_q.x, orintation_q.y,orintation_q.z,orintation_q.w]
    (roll , pitch, yaw) = euler_from_quaternion(orientation_list)
    if data.pose.pose.position.x and data.pose.pose.position.y != 0:
        xact =  data.pose.pose.position.x
        yact = data.pose.pose.position.y
        vact = data.twist.twist.linear.x
        wact = data.twist.twist.angular.z 
#*****************************************************************************************************************************************
def menger(x,y):      # This function is use for calculating the curvature at point
    global x_last_2, x_last, x_cur, y_last_2, y_last, y_cur
    x_last_2, x_last, x_cur, y_last_2, y_last, y_cur = x_last, x_cur, x, y_last, y_cur, y
    area = (0.5 * (x_last_2 * (y_last - y_cur) + x_last * (y_cur - y_last_2) + x_cur * (y_last_2 - y_last)))
    s1 = math.sqrt((x_last - x_last_2) ** 2 + (y_last - y_last_2) ** 2)
    s2 = math.sqrt((x_cur - x_last) ** 2 + (y_cur - y_last) ** 2)
    s3 = math.sqrt((x_cur - x_last_2) ** 2 + (y_cur - y_last_2) ** 2)

    # print area, s1, s2, s3
    if s1 <= epsilon:
        return 0.
    if s2 <= epsilon:
        return 0
    if s3 <= epsilon:
        return 0.
    else:
        return 4. * area /(s1 * s2 * s3)
#*****************************************************************************************************************************************
c1, c2, c3 = 2, 0.01, 1.1          # controller gains

if __name__ == "__main__":
    global xact, yact, yaw , vact, wact
    
    x_init, y_init = -50.0, 0.0
    xact, yact , yaw, vact, wact = -50.0, 0.0, 0.0, 0.0, 0.0
    T_i, T_f, tu = 0.0, 4.0, 0.0
    vref, omega_ref = 8.0, 0
    x_ref, y_ref, x_refold, y_refold = -50.0, 0.0, -50.0, 0.0 
    x_set_init, vx_i, ax_i, xt_f, vx_f, ax_f = 0.0 , vref, 0.0, 30.0, vref, 0.0
    y_set_init, vy_i, ay_i, yt_f, vy_f, ay_f = 0.0 , 0.0, 0.0, 4.0, 0.0, 0.0
    rospy.init_node('Fifth_order',anonymous=True) 
    rospy.Subscriber('/agent_0/odom', Odometry, datagps)         # Subscriber for gps data
    pub_speed = rospy.Publisher('/agent_0/cmd_vel',Twist,queue_size=10)      # Publisher for liner Velocity and angular velocity 
    speed_car = Twist()
    pub_graph_data = rospy.Publisher('/graph_data', Float64MultiArray, queue_size= 100) # for plotting the data
    graph_data = Float64MultiArray()
    rospy.sleep(5.0)  
    time_init = rospy.get_time()
    while (x_ref < 0.0):
        global xact, yact, yaw , vact, wact 
        time_curr = rospy.get_time()
        delt = time_curr - time_init
        x_ref = x_init + vref*(delt)
        y_ref = 0.0
        theta_ref_new = math.atan2( (y_ref - y_refold), (x_ref - x_refold) )      # calculation of reference orientation angle
        if theta_ref_new<0:
            theta_ref_new += 2*math.pi

        theta_act = yaw                 # calculation of actual orienation angle ( this is comming from webots wrapper node)
        if theta_act< 0: 
            theta_act += 2*math.pi

        xe = float(math.cos(theta_act)*(x_ref - xact) + math.sin(theta_act)*(y_ref - yact))        # calculation of error values  
        ye = float(-(math.sin(theta_act)*(x_ref - xact)) + math.cos(theta_act)*(y_ref - yact))
        theta_e = theta_ref_new - theta_act

        if theta_e < -math.pi:
            theta_e+=2*math.pi
        elif theta_e > math.pi:
            theta_e-=2*math.pi
        
        control_velocity = (vref) + ((c1*xe)/(math.sqrt( 1 + (pow(xe,2)) + (pow(ye,2)))))   # controlled linear velocity of car
        
        controlled_omega = omega_ref + (((c2*vref)*(ye*math.cos(theta_e/2) - xe*math.sin(theta_e/2) ))/( math.sqrt( 1 + (pow(xe,2)) + (pow(ye,2)))) ) + c3*math.sin(theta_e/2) #Controlled angular velocity of car

        graph_data.data = [float(x_ref), float(y_ref), xact, yact]   # Data Publish to plot
        pub_graph_data.publish(graph_data)

        print float(x_ref), float(y_ref), xact, yact, xe, ye, theta_e, vref, control_velocity, vact, omega_ref, controlled_omega, wact, time_curr
        
        speed_car.linear.x = control_velocity
        speed_car.angular.z = controlled_omega
        pub_speed.publish(speed_car)  
        # time_init = time_curr
        rospy.sleep(0.064)
    
    x_refold , y_refold = 0.0 , 0.0 
    time_init = rospy.get_time()
    if (x_ref > 0.0):
        #************************ Calculation Curve's Coeff ******************************************************
        g1 = np.matrix([[x_set_init], [vx_i], [ax_i], [xt_f], [vx_f], [ax_f] ])
        g2 = np.matrix([[y_set_init], [vy_i],[ay_i], [yt_f], [vy_f], [ay_f] ])
        P = np.matrix([[1,T_i,pow(T_i,2),pow(T_i,3),pow(T_i,4),pow(T_i,5)],
                      [0,1,2*T_i, 3*pow(T_i,2),4*pow(T_i,3),5*pow(T_i,4)],
                      [0,0,2,6*T_i,12*pow(T_i,2),20*pow(T_i,3)],
                      [1,T_f,pow(T_f,2), pow(T_f,3), pow(T_f,4),pow(T_f,5)],
                      [0,1,2*T_f,3*pow(T_f,2), 4*pow(T_f,3), 5*pow(T_f,4)],
                      [0,0,2,6*T_f,12*pow(T_f,2),20*pow(T_f,3)]])
        a_coff = np.linalg.pinv(P)*g1
        b_coff = np.linalg.pinv(P)*g2
        #*********************************************************************************************************
        while tu < T_f:   # comparing the simulation time with estimated time
            global xact, yact, yaw, vact, wact
            time_curr = rospy.get_time()
            tu = time_curr - time_init
            
            # ***********************Ref coordinates calculation***************************************************
            x_ref = np.dot( np.matrix([1, tu, pow(tu,2), pow(tu,3), pow(tu,4), pow(tu,5)]), a_coff )  
            y_ref = np.dot( np.matrix([1, tu, pow(tu,2), pow(tu,3), pow(tu,4), pow(tu,5)]), b_coff )
            #******************************************************************************************************
            theta_ref_new = math.atan2( (y_ref - y_refold), (x_ref - x_refold) )
            if theta_ref_new<0:
                theta_ref_new += 2*math.pi

            theta_act = yaw
            if theta_act< 0:
                theta_act += 2*math.pi

            xe = float(math.cos(theta_act)*(x_ref - xact) + math.sin(theta_act)*(y_ref - yact))        
            ye = float(-(math.sin(theta_act)*(x_ref - xact)) + math.cos(theta_act)*(y_ref - yact))
            theta_e = theta_ref_new - theta_act

            if theta_e < -math.pi:
                theta_e+=2*math.pi
            elif theta_e > math.pi:
                theta_e-=2*math.pi
            kk = menger(x_ref, y_ref)
            omega_ref = kk * vref
            control_velocity = (vref) + ((c1*xe)/(math.sqrt( 1 + (pow(xe,2)) + (pow(ye,2)))))   # controller 1
            
            controlled_omega = omega_ref + (((c2*vref)*(ye*math.cos(theta_e/2) - xe*math.sin(theta_e/2) ))/( math.sqrt( 1 + (pow(xe,2)) + (pow(ye,2)))) ) + c3*math.sin(theta_e/2) #Controller 1

            graph_data.data = [float(x_ref), float(y_ref), xact, yact]   # Data Publish to plot
            pub_graph_data.publish(graph_data)

            print float(x_ref), float(y_ref), xact, yact, xe, ye, theta_e, vref, control_velocity, vact, float(omega_ref), float(controlled_omega), wact, time_curr
            
            speed_car.linear.x = control_velocity
            speed_car.angular.z = controlled_omega
            pub_speed.publish(speed_car)  # Publishing the values through the publisher to webots wrapper node
            # time_init = time_curr
            rospy.sleep(0.064)            