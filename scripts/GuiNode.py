#!/usr/bin/python

import Tkinter
from Gui import Gui

class GuiNode:
    def __init__ (self):
        # Create window with GUI, Talker, and Listener
        window = Tkinter.Tk()
        gui = Gui(window)
        
        
        
#         window.protocol('WM_DELETE_WINDOW', close # Delete node when exiting program
#         window.geometry('{}x{}'.format(1024, 768)) # Resize Window
#         window.mainloop()

gui = GuiNode()
        
        
