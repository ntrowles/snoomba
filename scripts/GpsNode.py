#!/usr/bin/python

import rospy
import std_msgs

import pynmea2

import serial

import sys, signal

def signal_handler(signal, frame):
    print("\nProgram Exiting")
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

class GpsNode:
    def __init__(self):
        # Create node
        self.node = rospy.init_node('Gps', anonymous=True)
        self.giPub = rospy.Publisher('GpsInfo', std_msgs.msg.String, queue_size=10)

        # Setup serial ports
        self.serial = serial.Serial("/dev/ttyACM0")
        self.serial.baudrate = 9600
        self.serial.flushInput()
        self.serial.flushOutput()

        # Setup NMEA stream reader
        self._nmea_reader = pynmea2.NMEAStreamReader()


        #Loop
        while(True):
            #print("looping")
            self.loop()

    #http://stackoverflow.com/questions/19908167/reading-serial-data-in-realtime-in-python
    def loop(self):
        data_stream = self.serial.read(16)
        for msg in self._nmea_reader.next(data_stream):
            print("Message: " + str(msg))

            try:
                msg_parsed = pynmea2.NMEASentence.parse(str(msg))
                print("Parsed Msg: " + str(msg_parsed))
            except: #parsing issue occassionally occurs, can't find exception name, if found update
                print("Shit hit the fan") # TODO less retarded error handling

            class_type = type(msg_parsed)
            #print(class_type)
            if(isinstance(msg_parsed, pynmea2.types.talker.GLL)):
                pubMsg = "GLL " + str(msg_parsed.lat) + " " + str(msg_parsed.lat_dir) + " " + str(msg_parsed.lon) + " " + str(msg_parsed.lon_dir) + " " + str(msg_parsed.faa_mode)
                self.giPubMessage(pubMsg)

#            elif(isinstance(msg_parsed, pynmea2.types.talker.GGA)):
#                pubMsg = "GGA " + str(msg_parsed.types.talker.GGA)

            

##            
##        #bytes_to_read = self.serial.inWaiting()
##        #if(bytes_to_read == 0):
##        #    return
##        print(bytes_to_read)
##        data_raw = self.serial.read(16)
##        print("Bytes to Read: " + str(data_raw))
##
##        data_array = str.splitlines(data_raw)
##        print(str(len(data_array)) + str(data_array) )
##
##        #for data_line in data_array:
##        #print("data_line: " + str(data_line))
##        for msg in self._nmea_reader.next(data_line):
##            #data_parsed = pynmea2.NMEASentence.parse(data_line)
##            print("parsed data: " + str(msg))

    def giPubMessage(self, msg):
        self.giPub.publish(msg)

if __name__ == '__main__':
    gpsNode = GpsNode()
