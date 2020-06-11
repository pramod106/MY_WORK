#!/usr/bin/env python
from __future__ import division
import math 
import rospy
from std_msgs.msg import Float64MultiArray
import matplotlib.pyplot as plt
x_ref, y_ref, x_act, y_act, vcar, wcar, vcont, wcont, tim= [], [], [], [], [], [], [], [], []

def gpsdata(data):
    global wcar, wcont, tim
    
    car_omega = data.data[5]
    car_con_omega = data.data[7]
    time = rospy.get_time()
    
   
    wcar.append(car_omega)
    wcont.append(car_con_omega)
    tim.append(time)
    
def print_data():
    global wcar, wcont, tim

    plt.clf()
    plt.plot(tim, wcar, color = "red")
    plt.plot(tim, wcont, color = "green")
    
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