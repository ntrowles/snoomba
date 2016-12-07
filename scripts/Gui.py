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

        # Create top left frame for GUI (user selected map)
        self.topleftframe = Tkinter.Frame(window, bg="red", height=360, width=480)
        self.topleftframe.grid(row=0, column=2)
        # Create canvas in frame
        self.canvas = Tkinter.Canvas(window, width=480, height=360)
        self.canvas.grid(row=0, column=0)
        # Set boundaries for canvas
        self._canvas_x_min=0
        self._canvas_y_min=0
        self._canvas_x_max=480
        self._canvas_y_max=360

        # Google maps image
        image = self.googleApiRetrieveStaticImage(42.2742,-71.8082,480, 360, 19)
        photoImage = ImageTk.PhotoImage(image)

        # Draw picture
        self.canvas.create_image((240,180), image=photoImage, tags="static")

        # Create objects to drag
        self._drag_data = {"x": 0, "y": 0, "item": None}
        self._corners = []
        self._corners.append(self._create_token(100,100))
        self._corners.append(self._create_token(100,110))
        self._corners.append(self._create_token(110,100))
        self._corners.append(self._create_token(110,110))

        self.canvas.tag_bind("fenceCorner", "<ButtonPress-1>", self.onCornerClickEvent)
        self.canvas.tag_bind("fenceCorner", "<B1-Motion>", self.onCornerMotionEvent)
        self.canvas.tag_bind("fenceCorner", "<ButtonRelease-1>", self.onCornerReleaseEvent)

        #self.canvas.bind("<B1-Motion>", self.clickEvent)

        # Image in label
        self.map = Tkinter.Label(image=photoImage)
        self.map.grid(row=1, column=0)
        

        self.label = Tkinter.Label(window, text="Snoomba GUI")
        self.label.grid(row=0, column=1)        
        
        self.testButton = Tkinter.Button(window, text="Test", command=self.scPubMsg)
        self.testButton.grid(row=1, column=1)
        
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

    # The following four functions are modified from code found at http://stackoverflow.com/questions/6740855/board-drawing-code-to-move-an-oval/6759351#6789351
    def _create_token(self, x, y):
        return self.canvas.create_oval(x-5, y-5, x+5, y+5, outline="black", fill="black", tags="fenceCorner")

    def onCornerClickEvent(self, event):
        #print(self._drag_data["item"].tag_names)
        tags = self.canvas.gettags(self.canvas.find_closest(event.x, event.y)[0])
        print(tags)
        if any('fenceCorner' in s for s in tags):
            self._drag_data["item"] = self.canvas.find_closest(event.x, event.y)[0]
            self._drag_data["x"] = event.x
            self._drag_data["y"] = event.y

    def onCornerMotionEvent(self, event):
        # Skip if no item is selected
        if(self._drag_data["item"] == "None"):
            print("Incorrectly selected item glitch")
            return

        # Prevent from moving beyond boundaries
        if (event.x >= self._canvas_x_min+5 and event.x <= self._canvas_x_max-5):
            delta_x = event.x - self._drag_data["x"]
        else:
            delta_x = 0

        if (event.y >= self._canvas_y_min+5 and event.y <= self._canvas_y_max-5):
            delta_y = event.y - self._drag_data["y"]
        else:
            delta_y = 0

        self.canvas.move(self._drag_data["item"], delta_x, delta_y)
        self._drag_data["x"] = event.x
        self._drag_data["y"] = event.y

    def onCornerReleaseEvent(self, event):
        self._drag_data["item"] = "none"
        self._drag_data["x"] = 0
        self._drag_data["y"] = 0

    def drawLines(self):
        self.canvas.create_line(self.canvas.coords(self._corners[0]), self.canvas.coords(self._corners[1]))
        print("partially implemented method")
        

if __name__ == '__main__':
    gui = Gui()
