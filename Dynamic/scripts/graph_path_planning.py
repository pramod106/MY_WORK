#!/usr/bin/env python
from __future__ import division
import math 
import rospy
from std_msgs.msg import Float64MultiArray
import matplotlib.pyplot as plt
x_ref, y_ref, x_traj, y_traj = [], [], [], []

def gpsdata(data):
    global x_ref, y_ref
    
    xr = data.data[0]
    yr = data.data[1]

    
    x_ref.append(xr)
    y_ref.append(yr)

def trajdata(data):
    global x_traj, y_traj

    xtr = data.data[0]
    ytr = data.data[1]

    x_traj.append(xtr)
    y_traj.append(ytr)
    
def print_data():
    global x_ref, y_ref, x_traj, y_traj
    title_font = {'fontname':'Arial', 'size':'35', 'color':'black', 'weight':'normal',
    'verticalalignment':'bottom'} # Bottom vertical alignment for more space
    axis_font = {'fontname':'Arial', 'size':'20'}

    plt.clf() 
    plt.axis([-20, 20, 80, 200])  
    
    plt.plot(y_traj, x_traj, label = "possible Traj", color = "green", linewidth= 0.5)

    x1, y1 = [7.5, 7.5], [0, 200]
    x2, y2 = [-2.5, -2.5], [0, 200]
    x3, y3 = [2.5,2.5], [ 0, 200 ]
    plt.plot(x1, y1, x2, y2, color = "black")
    plt.plot(x3, y3, color='green', linestyle='dashed')
    plt.plot(y_ref, x_ref, label = "ref path ", color = "red")
    plt.plot(x_act, y_act, label = "actual path", color = "green")
    plt.title("Curvature = 0.05, Detection Distance = 25m , Vel =  5 m/s, ", **title_font) 
    plt.plot(0, 103, 'bo')
    plt.legend()
    plt.pause(0.02)     

if __name__ == "__main__":  
    plt.figure()
    rospy.init_node('graph_path_planning',anonymous=True)
    rospy.Subscriber('/graph_data_traj', Float64MultiArray,trajdata)
    rospy.Subscriber('/graph_data', Float64MultiArray,gpsdata)
    while True:
        # print vr, vc, vcar, wr, wc, wcar, time
        print_data()
        rospy.sleep(0.5)