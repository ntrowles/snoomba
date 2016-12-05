#!/usr/bin/python

import rospy
import std_msgs

class Mapper:
    def __init__(self):
        #Create node
        self.node = rospy.init_node('Mappder', anonymous=True)

        #Create BoundaryInfo, DistanceSensor, and GpsInfo listeners
        self.biSub = rospy.Subscriber('BoundaryInfo', std_msgs.msg.String, self.biCallback)
        self.dsSub = rospy.Subscriber('DistanceSensor', std_msgs.msg.String, self.dsCallback)
        self.giSub = rospy.Subscriber('GpsInfo', std_msgs.msg.String, self.giCallback)

        #Create Talker for MapInfo
        self.miPub = rospy.Publisher('MapInfo', std_msgs.msg.String, queue_size=10)
        
        # Sustain node
        rospy.spin()

    def biCallback(self, data):
        #Handle boundary info update
        print("biCallback unimplemented")
    
    def dsCallback(self, data):
        #Handle distance sensor info update
        print("dsCallback unimplemented")

    def giCallback(self, data):
        #Handle gps info update
        print("giCallback unimplemented")
    
