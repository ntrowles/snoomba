#!/usr/bin/python

import rospy
import RPi.GPIO as GPIO
import std_msgs

class MotorController:
    def __init__ (self):
        # Create Node
        self.node = rospy.init_node('MotorControllerNode', anonymous=True, disable_signals=False)

        # Create System Commands listener
        self.scSub = rospy.Subscriber('SystemCommands', std_msgs.msg.String, self.scCallback)
        
        # Initialize GPIO
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(29, GPIO.OUT)
        GPIO.setup(31, GPIO.OUT)

        GPIO.setup(32, GPIO.OUT)
        GPIO.setup(33, GPIO.OUT)

        self.pwm0=GPIO.PWM(32, 25000)
        self.pwm1=GPIO.PWM(33, 25000)

        self.pwm0.start(0)
        self.pwm1.start(0)
        
        # Sustain node
        rospy.spin()

    # Method for handling new messages received on 'SystemCommands'
    def scCallback(self, data):
        # Parse data to get parameters
        datalist = data.data.split(" ")
        if(datalist[0] == "SetMotor"):
            print("Set Motor function requested")
            if (len(datalist)==5 and datalist[1].isdigit() and datalist[2].isdigit() and datalist[3].isdigit() and datalist[4].isdigit()):
                # Extract parameters and send them to gpio controller function
                print("Sending parameters to motor controller driver")
                self.motorControllerDriver(int(datalist[1]), int(datalist[2]), float(datalist[3]), float(datalist[4]))
            else:
                print("Incorrect number of commands and/or command types sent to motor controller")
        else:
            print("Ignoring command: " + data)


    def motorControllerDriver(self, dir1, dir2, pwm1, pwm2):
        
        if(dir1==0):
            GPIO.output(29, GPIO.LOW)
        elif(dir1==1):
            GPIO.output(29, GPIO.HIGH)
        else:
            print("Invalid direction, not changing dir1")

        if(dir2==0):
            GPIO.output(31, GPIO.LOW)
        elif(dir2==1):
            GPIO.output(31, GPIO.HIGH)
        else:
            print("Invalid direction, not changing dir2")

        if(pwm1>=0 and pwm1<=100):
            self.pwm0.ChangeDutyCycle(pwm1)
        else:
            print("Invalid duty-cycle, not changing pwm1")

        if(pwm2>=0 and pwm2<=100):
            self.pwm1.ChangeDutyCycle(pwm2)
        else:
            print("Invalid duty-cycle, not changing pwm2")
            
if __name__ == '__main__':
    motorController = MotorController() 
