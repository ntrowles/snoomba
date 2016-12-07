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
        #self.topleftframe = Tkinter.Frame(window, bg="red", height=360, width=480)
        #self.topleftframe.grid(row=0, column=2)

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
        self.mapPic = self.canvas.create_image((240,180), image=photoImage, tags="static")

        # Create objects to drag
        self._drag_data = {"x": 0, "y": 0, "item": None}
        self._corners = []
        self._corners.append(self._create_token(200,140))
        self._corners.append(self._create_token(200,220))
        self._corners.append(self._create_token(280,220))
        self._corners.append(self._create_token(280,140))

        self.canvas.tag_bind("fenceCorner", "<ButtonPress-1>", self.onCornerClickEvent)
        self.canvas.tag_bind("fenceCorner", "<B1-Motion>", self.onCornerMotionEvent)
        self.canvas.tag_bind("fenceCorner", "<ButtonRelease-1>", self.onCornerReleaseEvent)

        # Create fill and lines
        self.fillFence()
        #self.drawLines()


##        #Add input text box for selecting address
##        self.addressframe = Tkinter.Frame(window, bg="red", height=60, width=480)
##        self.addressframe.grid(row=1, column=0)

        self.gpsframe = Tkinter.Frame(window, bg="blue", height = 200, width = 480)
        self.gpsframe.grid(row =1, column=0)
        
        self.address_prompt = Tkinter.Text(self.gpsframe, height=1, width=20)
        self.address_prompt.grid(row=0, column=0)
        self.address_prompt.insert(Tkinter.END, "Enter Address: ")
        
        self.address_entry = Tkinter.Entry(self.gpsframe)
        self.address_entry.grid(row = 0, column = 1, columnspan = 2)
        
        self.address_submit = Tkinter.Button(self.gpsframe, text="Search Address", command=self.addressButtonClick)
        self.address_submit.grid(row=0, column = 3)
        
        #Add input text box for selecting gps coordinates

        
        self.gps_lat_prompt = Tkinter.Text(self.gpsframe, height=1, width=20)
        self.gps_lat_prompt.grid(row=1, column=0)
        self.gps_lat_prompt.insert(Tkinter.END, "Enter Latitude: ")
        
        self.gps_lat_entry = Tkinter.Entry(self.gpsframe)
        self.gps_lat_entry.grid(row = 1, column = 1, columnspan = 2)
        
        self.gps_lon_prompt = Tkinter.Text(self.gpsframe, height=1, width=20)
        self.gps_lon_prompt.grid(row=2, column=0)
        self.gps_lon_prompt.insert(Tkinter.END, "Enter Longitude: ")
        
        self.gps_lon_entry = Tkinter.Entry(self.gpsframe)
        self.gps_lon_entry.grid(row = 2, column = 1, columnspan = 2)
        self.gps_submit = Tkinter.Button(self.gpsframe, text="Search GPS Coordinates", command=self.latlonButtonClick)
        
        self.gps_submit.grid(row=1, column = 3, rowspan=2)


        #Console Frame
        #self.consoleframe = Tkinter.Frame(window, bg="black", height = 360, width = 480)

        # ------------------------
        # Image in label
        #self.map = Tkinter.Label(image=photoImage)
        #self.map.grid(row=1, column=0)
        #self.label = Tkinter.Label(window, text="Snoomba GUI")
        #self.label.grid(row=0, column=1)        
        #self.testButton = Tkinter.Button(window, text="Test", command=self.scPubMsg)
        #self.testButton.grid(row=1, column=1)
        #------------------------------
        
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
        return self.canvas.create_oval(x-7, y-7, x+7, y+7, outline="black", fill="black", tags="fenceCorner")

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

        # move lines
        self.refillFence()
        #self.dragLines()

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

        # Move lines (temporary, should be updated during onCornerMotion)
        #self.refillFence()
        #self.dragLines()

    def drawLines(self):
        self._lines = []
        numCorners = len(self._corners)
        for i in range(0, numCorners-1):
            print(self.canvas.coords(self._corners[i]))
            print(self.canvas.coords(self._corners[i+1]))
            print(len(self.canvas.coords(self._corners[i])))
            self._lines.append(self.canvas.create_line(self.findCenterCoords(self.canvas.coords(self._corners[i])), self.findCenterCoords(self.canvas.coords(self._corners[i+1]))))
        self._lines.append(self.canvas.create_line(self.findCenterCoords(self.canvas.coords(self._corners[numCorners-1])), self.findCenterCoords(self.canvas.coords(self._corners[0]))))

        # print("partially implemented method")

    def dragLines(self):
        for line in self._lines:
            self.canvas.delete(line)
        self.drawLines()
        #print("unimplemented method")

    def fillFence(self):
        self._triangles = []
        ## Create 012 Triangle
        corner0Coord = self.findCenterCoords(self.canvas.coords(self._corners[0]))
        corner1Coord = self.findCenterCoords(self.canvas.coords(self._corners[1]))
        corner2Coord = self.findCenterCoords(self.canvas.coords(self._corners[2]))
        #self._triangles.append(self.canvas.create_polygon((corner0Coord, corner1Coord, corner2Coord), fill="yellow", disableoutline=True))
        ## Create 023 Triangle
        corner3Coord = self.findCenterCoords(self.canvas.coords(self._corners[3]))
        #self._triangles.append(self.canvas.create_polygon((corner0Coord, corner2Coord, corner3Coord), fill="yellow", disableoutline=True))
        # Create polygon
        self._triangles.append(self.canvas.create_polygon((corner0Coord, corner1Coord, corner2Coord, corner3Coord), fill="yellow", outline="black"))
          
    def refillFence(self):
        for triangle in self._triangles:
            self.canvas.delete(triangle)
        self.fillFence()

    def findCenterCoords(self, coords):
        x0 = coords[0]
        y0 = coords[1]
        x1 = coords[2]
        y1 = coords[3]
        return (x0+x1)/2, (y0+y1)/2

    def addressButtonClick(self):
        print("unimplemented method: address Button Click")

        # Replace background picture with new picture given address, update self.pic_gps_coords

    def latlonButtonClick(self):
        #print("unimplemented method: latlon Button Click")

        # Replace background picutre with new picture given latitude and longitude, update self.pic_gps_coords
        latStr = self.gps_lat_entry.get()
        lonStr = self.gps_lon_entry.get()

        #TODO error handling for non float entries
        lat = float(latStr)
        lon = float(lonStr)
        print("lat: %d, lon: %d", lat, lon)
        self.updateImage(lat, lon)
        
    def updateImage(self, lat, lon):
        self.canvas.delete(self.mapPic)

        # Google maps image
        image = self.googleApiRetrieveStaticImage(lat, lon,480, 360, 19)
        photoImage = ImageTk.PhotoImage(image)

        # Draw picture
        self.mapPic = self.canvas.create_image((240,180), image=photoImage, tags="static")
        print("picture drawn")
        

if __name__ == '__main__':
    gui = Gui()
