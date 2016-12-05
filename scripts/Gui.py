#!/usr/bin/python

import Tkinter
import rospy
import std_msgs
import Image
from PIL import ImageTk
import urllib
from cStringIO import StringIO
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

        
        self.topleftframe = Tkinter.Frame(window, bg="red", height=360, width=480)
        self.topleftframe.grid(row=0, column=0)

        image = self.googleApiRetrieveStaticImage(42.2742,-71.8082,480, 360, 19)
        photoImage = ImageTk.PhotoImage(image)
        self.map = Tkinter.Label(image=photoImage)
        self.map.grid(row=1, column=0)
        

        #self.label = Tkinter.Label(window, text="Snoomba GUI")
        #self.label.pack()
        
        
        #self.testButton = Tkinter.Button(window, text="Test", command=self.scPubMsg)
        #self.testButton.pack()
        
        
        # Start window
        window.mainloop()
        
        # Delete node when exiting program
        #window.protocol('WM_DELETE_WINDOW', self.closeNode())
        
        # Sustain node
        # rospy.spin()

    def miCallback(self, data):
        rospy.loginfo("Map-Info received" + data.data)

    def scPubMsg(self, msg="SetMotor 1 0 40 30"):
        print("Button Pressed!")
        self.scPub.publish(msg)

    def googleApiRetrieveStaticImage(self, lat, lon, width, height, zoom):
        urlList = []
        urlList.append("https://maps.googleapis.com/maps/api/staticmap?center=")
        urlList.append(str(lat))
        urlList.append(",")
        urlList.append(str(lon))
        urlList.append("&size=")
        urlList.append(str(width))
        urlList.append("x")
        urlList.append(str(height))
        urlList.append("&zoom=")
        urlList.append(str(zoom))
        url = ''.join(urlList)
        buffer = StringIO(urllib.urlopen(url).read())
        image = Image.open(buffer)
        return image

if __name__ == '__main__':
    gui = Gui()
