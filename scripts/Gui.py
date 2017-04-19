#!/usr/bin/python

import Tkinter
import rospy
import std_msgs
import nav_msgs
import sensor_msgs
import Image
from PIL import ImageTk
import urllib

# from http://stackoverflow.com/questions/7490491/capture-embedded-google-map-image-with-python-without-using-a-browser
import Image, urllib, StringIO
from math import log, exp, tan, atan, pi, ceil

from cStringIO import StringIO
from GuiLayout import GuiLayout

import requests
import json

from lidar_lite import Lidar_Lite


#import GuiListener
#import GuiTalker


EARTH_RADIUS = 6378137
EQUATOR_CIRCUMFERENCE = 2 * pi * EARTH_RADIUS
INITIAL_RESOLUTION = EQUATOR_CIRCUMFERENCE / 256.0
ORIGIN_SHIFT = EQUATOR_CIRCUMFERENCE / 2.0


class GPS_Point:
    def __init__ (self, lat, lon, mode):
        self.lat = lat
        self.lon = lon
        self.mode = mode

    def __str__(self):
        return "Lat: " + str(self.lat) + ", Lon: " + str(self.lon) + ", Mode: " + str(self.mode)


class Gui:
    def __init__ (self):
        # Create node
        self.node = rospy.init_node('GUINode', anonymous=True, disable_signals=False)

        # Create Listener for incoming map information from Mapper Node
        self.miSub = rospy.Subscriber('MapInfo', std_msgs.msg.String, self.miCallback)
        
        # Create Talkers for system commands and boundary info
        self.scPub = rospy.Publisher('SystemCommands', std_msgs.msg.String, queue_size=10)
        self.biPub = rospy.Publisher('BoundaryInfo', std_msgs.msg.String, queue_size=10)

        # Create GPS Globals
        self.gps_buffer = [0] * 250
        self.gps_buffer_index = 0
        self.lat_offset = 0.10918536
        self.lon_offset = -0.321672352
        
        # Create Window
        window = Tkinter.Tk()
        window.geometry('{}x{}'.format(1024, 768))
        # Enforce window size - http://www.stackoverflow.com/questions/563827/hot-to-stop-tkinter-frame-from-shrinking-to-fit-its-contents
        # FEATURE make dynamic eventually, temporary fix
        
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

        # Initialize self.zoomval to be 19
        self.zoomval = 19
        # Google maps image
        self.latval = 42.2742
        self.lonval = -71.8082
        image = self.googleApiRetrieveStaticImage(42.2742,-71.8082,480, 360, self.zoomval)
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

        self.gpsframe = Tkinter.Frame(window, bg="orange", height = 200, width = 480)
        self.gpsframe.grid(row =1, column=0)
        self.gpsframe.grid_propagate(0)
        
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




        #Map Editor Frame
        self.mapeditorframe = Tkinter.Frame(window, bg="red", height = 560, width = 200)
        self.mapeditorframe.grid(row=0, column=1, rowspan=2)
        self.mapeditorframe.grid_propagate(0)

        # Zoom in button
        self.me_zoominbutton = Tkinter.Button(self.mapeditorframe, text="Zoom in", command=self.zoominButtonClick)
        self.me_zoominbutton.grid(row=0, column=0)

        # Zoom out button
        self.me_zoomoutbutton = Tkinter.Button(self.mapeditorframe, text="Zoom out", command = self.zoomoutButtonClick)
        self.me_zoomoutbutton.grid(row=1, column=0)

        # Zoom number display
        self.me_zoomvaldisp = Tkinter.Text(self.mapeditorframe, height=1, width = 2)
        self.me_zoomvaldisp.grid(row=2, column=0)
        self.me_zoomvaldisp.insert(Tkinter.END, str(self.zoomval))

        # Console Frame
        self.consoleframe = Tkinter.Frame(window, bg="black", height=560, width = 344)
        self.consoleframe.grid(row=0, column=2, rowspan=2)
        self.consoleframe.grid_propagate(0)

        # Command History
        self.cf_history_text = Tkinter.Text(self.consoleframe, height = 24, width = 36, bg="black", fg="green")
        self.cf_history_text.grid(row=0, column=0, columnspan = 2)

        # Command Line Entry
        self.cf_command_entry = Tkinter.Entry(self.consoleframe, bg="black", fg="green")
        self.cf_command_entry.grid(row=1, column=0)

        # Command Line Button
        self.cf_command_button = Tkinter.Button(self.consoleframe, text="Enter Command", command=self.commandEntryButtonClick)
        self.cf_command_button.grid(row=1, column=1)


        # System Command Module
        self.systemframe = Tkinter.Frame(window, bg="blue", height=250, width=480)
        self.systemframe.grid(row = 2, column = 0)
        self.systemframe.grid_propagate(0)

        # Start Button
        self.sys_start_button = Tkinter.Button(self.systemframe, text="Start", bg="green", command=self.sysStartButtonClick)
        self.sys_start_button.grid(row=0, column=0)

        # Stop Button
        self.sys_stop_button = Tkinter.Button(self.systemframe, text="Stop", bg="red", command=self.sysStopButtonClick)
        self.sys_stop_button.grid(row=1, column=0)

        # GPS Test Button
        self.sys_gpstest_button = Tkinter.Button(self.systemframe, text="GPS Test", bg="blue", command=self.sysGpstestButtonClick)
        self.sys_gpstest_button.grid(row=2, column=0)

        # Lidar_Lite instance (for testing only, not needed in final code)
        #self.lidar_lite = Lidar_Lite()
        #errno = self.lidar_lite.connect(1)
        #print(errno)
        ## TODO fix i2c setup
        #distance = self.lidar_lite.getDistance()
        #print(distance)
        
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
        print("Map-Info received" + data.data)

        # Parse data to get parameters
        datalist = data.data.split(" ")
        if(datalist[0] == "UpdatePosition"):
            print("Update Position function requested")
            if (len(datalist)==4 and isfloat(datalist[1]) and isfloat(datalist[2])):
                # Extract parameters and send them to gpio controller function
                print("Updating position buffer")
                self.gps_buffer_index = self.wrap_index(self.gps_buffer_index + 1)
                self.gps_buffer[self.gps_buffer_index] = GPS_Point(float(datalist[1]) + self.lat_offset, float(datalist[2]) + self.lon_offset, str(datalist[3]))
            else:
                print("Incorrect number of commands and/or command types sent to motor controller")
        else:
            print("Ignoring command: " + data)

    def scPubMsg(self, msg):
        #print("Button Pressed!")
        self.scPub.publish(msg)

    def biPubMsg(self, msg="No Message"):
        self.biPub.publish(msg)

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
        urlList.append("&maptype=")
        urlList.append("hybrid")
        url = ''.join(urlList)
        print(url)
        buffer = StringIO(urllib.urlopen(url).read())
        image = Image.open(buffer)
        print(image)
        return image

    def googleApiRetrieveCoords(self, address):
        urlList=[]
        urlList.append("https://maps.google.com/maps/api/geocode/json?address=")

        address_partlist = address.split(" ")
        address_query_url = "+".join(address_partlist)
        urlList.append(address_query_url)
        url = ''.join(urlList)
        print(url)
        #TODO make api call to url
        response = requests.get(url)
        print(response.status_code)
        json_data = response.json()
        json_text = response.text
        print(json_data)
        print(json_text)
        json_dict = json.loads(json_text)
        print(json_dict["status"])
        results = json_dict["results"]
        results0 = results[0]
        geometry = results0["geometry"]
        location = geometry["location"]
        lat = location["lat"]
        lon = location["lng"]
        print("lat: " + str(lat) + ", lon: " + str(lon)) 
        return (lat, lon)

    def updateMapPicFromAddress(self, address):
        (lat, lon) = self.googleApiRetrieveCoords(address)
        self.updateImage(lat, lon)

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
        
        gps_coords = self.pixelstogpscoord(event.x, event.y)
        print(gps_coords)
        msg="UpdateCorner " + str(self._corners.index(self._drag_data["item"])) + " " + str(gps_coords[0]) + " " + str(gps_coords[1])
        print("Publishing Message on 'BoundaryInfo'  - " + msg)
        self.biPubMsg(msg)


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
        #print("unimplemented method: address Button Click")
        address = self.address_entry.get()
        #self.googleApiRetrieveCoords(address)
        self.updateMapPicFromAddress(address)
        # Replace background picture with new picture given address, update self.pic_gps_coords

    def latlonButtonClick(self):
        #print("unimplemented method: latlon Button Click")

        # Replace background picutre with new picture given latitude and longitude, update self.pic_gps_coords
        latStr = self.gps_lat_entry.get()
        lonStr = self.gps_lon_entry.get()

        #TODO error handling for non float entries
        try:
            lat = float(latStr)
            lon = float(lonStr)
        except ValueError:
            print("Lat or Lon value not entered correctly")
            return
        print("lat: %d, lon: %d", lat, lon)
        self.updateImage(lat, lon)
        self.latval = lat;
        self.lonval = lon;
        
    def updateImage(self, lat, lon):
        

        # Google maps image
        image = self.googleApiRetrieveStaticImage(lat, lon,480, 360, self.zoomval)
        photoImage = ImageTk.PhotoImage(image)
        self.latval = lat
        self.lonval = lon

        

        # Draw picture
        #newMapPic = self.canvas.create_image((240,180), image=photoImage, tags="static")
        #self.canvas.delete(self.mapPic)
        #self.mapPic = newMapPic

        # stackoverflow.com/questions/19838972/python-tkinter-update-image-in-canvas
        self.photoImage = photoImage
        self.canvas.itemconfig(self.mapPic, image = self.photoImage)
        #self.canvas.update()
        
        print("picture drawn")

        #self.canvas.delete(self.mapPic)

    def zoominButtonClick(self):
        # TODO implement method
        # print("unimplemented method: zoominButtonClick(self))")
        self.zoomval = self.zoomval+1;
        self.updateImage(self.latval, self.lonval)
        self.me_zoomvaldisp.delete("1.0", Tkinter.END)
        self.me_zoomvaldisp.insert(Tkinter.END, str(self.zoomval))
        print("New zoom value")
        print(self.zoomval)

    def zoomoutButtonClick(self):
        # TODO implement method
        # print("unimplemented mehtod: zoomoutButtonClick(self))")
        self.zoomval = self.zoomval-1;
        self.updateImage(self.latval, self.lonval)
        self.me_zoomvaldisp.delete("1.0", Tkinter.END)
        self.me_zoomvaldisp.insert(Tkinter.END, str(self.zoomval))
        print("New zoom value: ")
        print(self.zoomval)

    def commandEntryButtonClick(self):
        # TODO implement method
        print("implementing mehtod: commandEntryButtonClick(self))")
        command = self.cf_command_entry.get()
        self.cf_history_text.insert(Tkinter.END, "\n"+command)

        command_parts = command.split(" ")
        if(command_parts[0] == "pub_msg"):
            print("implementing publ_msg callback")
        else:
            print("improper command: " + command)
        

    def sysStartButtonClick(self):
        # TODO implement method
        print("unimplemented mehtod: sysStartButtonClick(self))")

    def sysStopButtonClick(self):
        # TODO implement method
        print("unimplemented mehtod: sysStopButtonClick(self))")

    def sysGpstestButtonClick(self):
        most_recent_points = [None] * 50 # create list of size 100
        # Copy over 50 most recent points from 
        finLen = 0
        for i in range(0, len(most_recent_points)):
            #if(self.gps_buffer[self.wrap_index(self.gps_buffer_index-i)] #TODO fix error of it sending blank list if 100 samples haven't been taken
            most_recent_points[i] = self.gps_buffer[self.wrap_index(self.gps_buffer_index-i)]
            print("i: " + str(i) + "   " + str(most_recent_points[i]))
        self.displayPoints(most_recent_points)

    def wrap_index(self, index):
        new_index = index % 250
        return new_index

    def displayPoints(self, most_recent_points):
        
        # Create Window
        sub_window = Tkinter.Toplevel()
        sub_window.geometry('{}x{}'.format(1080, 720))
        
        # Initialize GUI
        # self.layout = GuiLayout(window, self)
        sub_window.title("GPS Test")

        # Create top left frame for GUI (user selected map)
        #self.topleftframe = Tkinter.Frame(window, bg="red", height=360, width=480)
        #self.topleftframe.grid(row=0, column=2)

        # Create canvas in frame
        sub_canvas = Tkinter.Canvas(sub_window, width=1080, height=720)
        sub_canvas.grid(row=0, column=0)
        # Set boundaries for canvas
        sub_canvas_x_min=0
        sub_canvas_y_min=0
        sub_canvas_x_max=1080
        sub_canvas_y_max=720


        lat_total = 0.0
        lon_total = 0.0
        lat_min = most_recent_points[0].lat
        lat_max = most_recent_points[0].lat
        lon_min = most_recent_points[0].lon
        lon_max = most_recent_points[0].lon
        # Go thru points
        for i in range(0, len(most_recent_points)):
            cur_gps_point = most_recent_points[i]
            lat_total += cur_gps_point.lat
            lon_total += cur_gps_point.lon

            if(cur_gps_point.lat < lat_min):
                lat_min = cur_gps_point.lat
            elif(cur_gps_point.lat > lat_max):
                lat_max = cur_gps_point.lat

            if(cur_gps_point.lon < lon_min):
                lon_min = cur_gps_point.lon
            elif(cur_gps_point.lon > lon_max):
                lon_max = cur_gps_point.lon

        lat_avg = float(lat_total)/float(len(most_recent_points))
        lon_avg = float(lon_total)/float(len(most_recent_points))

        # Initialize self.zoomval to be 23 -> Start with max zoom, zoom out until all points fit picture
        zoomval = 21
        latval = lat_avg
        lonval = lon_avg
        # Calculate GPS coordinates
        (px_center, py_center) = latlontopixels(latval, lonval, zoomval)
        px_left = px_center - ((sub_canvas_x_min+sub_canvas_x_max)/2)
        px_right = px_center + ((sub_canvas_x_min+sub_canvas_x_max)/2)
        py_up = py_center + ((sub_canvas_y_min+sub_canvas_y_max)/2)
        py_down = py_center - ((sub_canvas_y_min+sub_canvas_y_max)/2)
        (lat_up, lon_left) = pixelstolatlon(px_left, py_up, zoomval)
        (lat_down, lon_right)= pixelstolatlon(px_right, py_down, zoomval)
        #centeroffset_x = px - ((self._canvas_x_min+self._canvas_x_max)/2)
        #centeroffset_y = py - ((self._canvas_y_min+self._canvas_y_max)/2)
        #px_actual = px_center - centeroffset_x
        #py_actual = py_center - centeroffset_y
        #gps_differential = pixelstolatlon(px_actual, py_actual, self.zoomval)
        while(lat_max>lat_up or lat_min<lat_down or lon_min<lon_left or lon_max>lon_right):
            zoomval -= 1
            print("Zoomval: " + str(zoomval) + ", lat_up: " + str(lat_up) + ", lat_down: " + str(lat_down) + ", lon_left: " + str(lon_left) + ", lon_right" + str(lon_right))
            print("lat_max: " + str(lat_max) + ", lat_min: " + str(lat_min) + ", lon_min: " + str(lon_min) + ", lon_max: " + str(lon_max))
            (px_center, py_center) = latlontopixels(latval, lonval, zoomval)
            px_left = px_center - ((sub_canvas_x_min+sub_canvas_x_max)/2)
            px_right = px_center + ((sub_canvas_x_min+sub_canvas_x_max)/2)
            py_up = py_center + ((sub_canvas_y_min+sub_canvas_y_max)/2)
            py_down = py_center - ((sub_canvas_y_min+sub_canvas_y_max)/2)
            (lat_up, lon_left) = pixelstolatlon(px_left, py_up, zoomval)
            (lat_down, lon_right) = pixelstolatlon(px_right, py_down, zoomval)
        
        # Google maps image

        image = self.googleApiRetrieveStaticImage(latval, lonval, 1080, 720, zoomval)
        photoImage = ImageTk.PhotoImage(image)

        # Draw picture
        mapPic = sub_canvas.create_image((540,360), image=photoImage, tags="static")
        # Draw tokens
        for i in range(0, len(most_recent_points)):
            cur_gps_point = most_recent_points[i]
            (px_abs, py_abs) = latlontopixels(cur_gps_point.lat, cur_gps_point.lon, zoomval)
            px = 540 - (px_center - px_abs)
            py = 360 - (py_center - py_abs)

            
            if(cur_gps_point.mode == "A"):
                sub_canvas.create_oval(px-3, py-3, px+3, py+3, outline="red", fill="red", tags="fenceCorner")
            elif(cur_gps_point.mode == "D"):
                sub_canvas.create_oval(px-3, py-3, px+3, py+3, outline="blue", fill="blue", tags="fenceCorner")
            


        sub_window.mainloop()

    def appendToLog(self):
        # TODO implement method
        print("unimplemented mehtod: appendToLog(self))")

###########################################

    def pixelstogpscoord(self, px, py):
        # Calculate GPS coordinates
        (px_center, py_center) = latlontopixels(self.latval, self.lonval, self.zoomval)
        centeroffset_x = px - ((self._canvas_x_min+self._canvas_x_max)/2)
        centeroffset_y = py - ((self._canvas_y_min+self._canvas_y_max)/2)
        px_actual = px_center - centeroffset_x
        py_actual = py_center - centeroffset_y
    

        gps_differential = pixelstolatlon(px_actual, py_actual, self.zoomval)
        return gps_differential
        #upperleft_gps_coords = 

        # Set boundaries for canvas
        #self._canvas_x_min=0
        #self._canvas_y_min=0
        #self._canvas_x_max=480
        #self._canvas_y_max=360

        # Initialize self.zoomval to be 19
        #self.zoomval = 19
        # Google maps image
        #self.latval = 42.2742
        #self.lonval = -71.8082

#    def latlontopixels(lat, lon, zoom):
#        mx = (latlon



# from http://stackoverflow.com/questions/7490491/capture-embedded-google-map-image-with-python-without-using-a-browser
###########################################




def latlontopixels(lat, lon, zoom):
    mx = (lon * ORIGIN_SHIFT) / 180.0
    my = log(tan((90 + lat) * pi/360.0))/(pi/180.0)
    my = (my * ORIGIN_SHIFT) /180.0
    res = INITIAL_RESOLUTION / (2**zoom)
    px = (mx + ORIGIN_SHIFT) / res
    py = (my + ORIGIN_SHIFT) / res
    return px, py

def pixelstolatlon(px, py, zoom):
    res = INITIAL_RESOLUTION / (2**zoom)
    mx = px * res - ORIGIN_SHIFT
    my = py * res - ORIGIN_SHIFT
    lat = (my / ORIGIN_SHIFT) * 180.0
    lat = 180 / pi * (2*atan(exp(lat*pi/180.0)) - pi/2.0)
    lon = (mx / ORIGIN_SHIFT) * 180.0
    return lat, lon



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
    gps_differential = pixelstolatlon(100, 10, 18)
    print("gps differential: " + str(gps_differential))

    #Spawn GUI
    gui = Gui()
    
