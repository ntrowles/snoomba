#!/usr/bin/python

import rospy
import std_msgs

import sensor_msgs.msg

class Mapper:
    def __init__(self):
        print("Starting 'Mapper' Node")
        #Create node
        self.node = rospy.init_node('Mapper', anonymous=True)

        #Create BoundaryInfo, DistanceSensor, and GpsInfo listeners
        self.biSub = rospy.Subscriber('BoundaryInfo', std_msgs.msg.String, self.biCallback)
        self.dsSub = rospy.Subscriber('DistanceSensor', std_msgs.msg.String, self.dsCallback)
        self.giSub = rospy.Subscriber('GpsInfo', std_msgs.msg.String, self.giCallback)

        #Create Talker for MapInfo and scan
        self.miPub = rospy.Publisher('MapInfo', std_msgs.msg.String, queue_size=10)
        self.scanPub = rospy.Publisher('scan', sensor_msgs.msg.LaserScan, queue_size=10)

        # Initialize list of points

        self.ranges = []
        self.intensities = []
        self.dir = 0 # 0=none, 1=up, 2=down
        
        # Sustain node
        rospy.spin()

    def biCallback(self, data):
        #Handle boundary info update
        print("biCallback unimplemented")
    
    def dsCallback(self, data):
        #Handle distance sensor info update
        print("dsCallback partially implemented")
        datalist = data.data.split(" ")
        if(datalist[0] == "PC"):
            print("Add polar coordinate function called")
            self.add_polar_coordinates(int(datalist[1]), float(datalist[2]))
        

    def giCallback(self, data):
        #Handle gps info update
        print("giCallback partially unimplemented")
        # Parse and send to Gui over MapInfo topic
        # Parse data to get parameters
        datalist = data.data.split(" ")
        if(datalist[0] == "GLL"):
            print("Update Position function requested")
            if (len(datalist)==6 and isfloat(datalist[1]) and isfloat(datalist[3])):
                # Extract parameters and send them to Gui
                print("Sending parameters to Gui node over MapInfo topic")
                #self.motorControllerDriver(int(datalist[1]), int(datalist[2]), float(datalist[3]), float(datalist[4]))
                pubMsg = "UpdatePosition "
                lat = float(datalist[1])/100
                if(datalist[2] == "S"):
                    lat *= -1
                pubMsg += str(lat)
                lon = float(datalist[3])/100
                if(datalist[4] == "W"):
                    lon *= -1
                pubMsg += (" " + str(lon) + " " + str(datalist[5]))
                self.miPubMessage(pubMsg)
            else:
                print("Incorrect number of commands and/or command types sent to mapper")
        else:
            print("Ignoring command: " + data)

    def miPubMessage(self, msg):
        self.miPub.publish(msg)

    def scanPubMessage(self, msg):
        self.scanPub.publish(msg)

    def add_polar_coordinates(self, pos, dis):
        print(pos)
        print(dis)
        if((pos==0 or pos==360) and self.dir!=0):
            self.publishLaserScan()

        if(pos==0):
            self.dir=1
        elif(pos==360):
            self.dir=2

        if(self.dir==1):
            self.ranges.append(dis)
        elif(self.dir==2):
            self.ranges.insert(0, dis)
        
    #https://gist.github.com/atotto/c47bc69a48ed38e86947b5506b8e0e61
    def publishLaserScan(self):
        laser_scan = sensor_msgs.msg.LaserScan()
        laser_scan.header.stamp = rospy.Time.now()
        laser_scan.header.frame_id = 'snoomba_frame'
        laser_scan.angle_min = -3.14
        laser_scan.angle_max = 3.14
        laser_scan.angle_increment = 6.28 / 90
        laser_scan.range_min = 0.0
        laser_scan.range_max = 4000

        laser_scan.ranges = self.ranges
        laser_scan.intensities = self.intensities

        self.ranges = []
        self.intensities = []

        print(laser_scan)
        self.scanPubMessage(laser_scan)


############################################
# from http://stackoverflow.com/questions/736043/checking-if-a-string-can-be-converted-to-float-in-python
def isfloat(value):
    try:
        float(value)
        return True
    except ValueError:
        return False

############################################        

if __name__ == '__main__':
    mapper = Mapper()
    
