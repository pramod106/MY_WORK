#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray

class SumoSupervisorPlugin:
    def __init__(self, supervisor, traci, net):
        self.traci = traci
        self.runned = 0
    def run(self, duration):
        self.runned = self.runned + duration
        rospy.init_node('plugin',anonymous=True)
        self.pub_dyn_data = rospy.Publisher('/dyn_obs', Float64MultiArray, queue_size= 100) # for plotting the data
        self.dyn_data = Float64MultiArray()
        # print self.traci.vehicle.getIDList()
        for i in self.traci.vehicle.getIDList():
            x = self.traci.vehicle.getPosition(i)[1]
            y = self.traci.vehicle.getPosition(i)[0]
            vel = self.traci.vehicle.getSpeed(i)
            self.dyn_data.data = [vel,x,y]
            self.pub_dyn_data.publish(self.dyn_data)
            print(vel,abs(x),abs(y))
            # rospy.sleep(0.016)