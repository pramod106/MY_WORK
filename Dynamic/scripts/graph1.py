#!/usr/bin/env python
from __future__ import division
import math 
import rospy
from std_msgs.msg import Float64MultiArray
import matplotlib.pyplot as plt
x_ref, y_ref, x_act, y_act= [], [], [], []

def gpsdata(data):
    global x_ref, y_ref, x_act, y_act
    
    xr = data.data[0]
    yr = data.data[1]
    xac = data.data[2]
    yac = data.data[3]
    
    x_ref.append(xr)
    y_ref.append(yr)
    x_act.append(xac)
    y_act.append(yac)
    
def print_data():
    global x_ref, y_ref, x_act, y_act
    title_font = {'fontname':'Arial', 'size':'35', 'color':'black', 'weight':'normal',
    'verticalalignment':'bottom'} # Bottom vertical alignment for more space
    axis_font = {'fontname':'Arial', 'size':'20'}

    plt.clf()  
    plt.axis([-100, 100,0,400])   # axis limit
    # plt.plot(0, 103, 'bo')        # point as obtsacle position
    x1, y1 = [7.5, 7.5], [0, 400]
    x2, y2 = [-2.5, -2.5], [0, 400]
    x3, y3 = [2.5,2.5], [ 0, 400 ]
    plt.plot(x1, y1, x2, y2, color = "black")
    plt.plot(x3, y3, color='green', linestyle='dashed')
    plt.plot(y_ref, x_ref, label = "ref path ", color = "red")             # I done some changes here swap x_ref with y_ref or vice versa
    plt.plot(y_act, x_act, label = "actual path", color = "green")
    plt.title("Curvature = 0.025, Detection Distance = 30m , Vel =  5 m/s, ", **title_font)
    plt.legend()
    plt.pause(0.02)     

if __name__ == "__main__":  
    plt.figure()
    rospy.init_node('graph',anonymous=True)
    rospy.Subscriber('/graph_data', Float64MultiArray,gpsdata)
    while True:
        # print vr, vc, vcar, wr, wc, wcar, time
        print_data()
        # rospy.sleep(0.064)