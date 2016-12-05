#!/usr/bin/python

import Tkinter
import rospy
import std_msgs
from GuiLayout import GuiLayout

#import GuiListener
#import GuiTalker

class Gui:
    def __init__ (self):
        # Create node
        self.node = rospy.init_node('GUINode', anonymous=True, disable_signals=False)

        # Create Listener for incoming map information from Mapper Node
        self.miSub = rospy.Subscriber('MapInfo', std_msgs.msg.String, self.miCallback)
        
        # Create Talkers for system commands and boundary info
        self.scPub = rospy.Publisher('SystemCommands', std_msgs.msg.String, queue_size=10)
        self.biPub = rospy.Publisher('BoundaryInfo', std_msgs.msg.String, queue_size=10)
        
        # Create Window
        window = Tkinter.Tk()
        window.geometry('{}x{}'.format(1024, 768))
        
        # Initialize GUI
        # self.layout = GuiLayout(window, self)
        self.window = window
        window.title("Snoomba GUI")
        
        self.label = Tkinter.Label(window, text="Snoomba GUI")
        self.label.pack()
        
        
        self.testButton = Tkinter.Button(window, text="Test", command=self.scPubMsg)
        self.testButton.pack()
        
        
        # Start window
        window.mainloop()
        
        # Delete node when exiting program
        #window.protocol('WM_DELETE_WINDOW', self.closeNode())
        
        # Sustain node
        # rospy.spin()



    def miCallback(self, data):
        rospy.loginfo("Map-Info received" + data.data)
         

    def closeNode(self):
        rospy.signal_shutdown("Application Closed")

    def scPubMsg(self, msg="SetMotor 1 0 40 30"):
        print("Button Pressed!")
        self.scPub.publish(msg)

if __name__ == '__main__':
    gui = Gui()
