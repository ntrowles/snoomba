#!/usr/bin/python

import rospy
import RPi.GPIO as GPIO
import std_msgs
from lidar_lite import Lidar_Lite

import sys, signal

def signal_handler(signal, frame):
    print("\nProgram Exiting")
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

class LidarNode:
    def __init__(self):
        # Initialize ROS node
        self.node = rospy.init_node('LidarNode', anonymous=True, disable_signals=False)
        self.dsPub = rospy.Publisher('DistanceSensor', std_msgs.msg.String, queue_size=10)

        # Initialize lidar lite helper class
        self.lidar_lite = Lidar_Lite()
        self.errno = self.lidar_lite.connect(1)

        # GPIO setup
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(32, GPIO.OUT)
        GPIO.setup(33, GPIO.OUT)

        # Servo control example at http://razzpisampler.oreilly.com/ch05.html
        self.pwm0 = GPIO.PWM(32, 100)
        self.pwm1 = GPIO.PWM(33, 100)

        self.pwm0.start(5)
        self.pwm1.start(5)
                                     

        # Loop
        # help from http://stackoverflow.com/questions/18994912/ending-an-infinite-while-loop
        while(True):
            self.loop()

    # Ported from arduino version:
    # Lynxmotion BotBoarduino LIDAR Lite Sweep Scan
    # Version: 1.0.0
    # By: scharette
    # Date: 2015-03-31
    def loop(self):
        for pos in range (0, 360, 4):
            self.set_angle(pos)
            distance = self.lidar_lite.getDistance()
            print(str(distance))
            self.dsPubMessage('PC ' + str(pos) + ' ' + str(distance))
        for pos in range (360, 0, -4):
            self.set_angle(pos)
            distance = self.lidar_lite.getDistance()
            print(str(distance))
            self.dsPubMessage('PC ' + str(pos) + ' ' + str(distance))

    # from http://razzpisampler.oreilly.com/ch05.html
    def set_angle(self, angle):
        duty = (float(angle) / 2.0) / 9.5 + 5
        print(str(duty))
        self.pwm0.ChangeDutyCycle(duty)
        self.pwm1.ChangeDutyCycle(duty)

    def dsPubMessage(self, msg):
        self.dsPub.publish(msg)

        
if __name__ == '__main__':
    lidarNode = LidarNode()
