#!/usr/bin/python

import Tkinter

class GuiLayout:
    def __init__(self, window, gui):
        self.window = window
        window.title("Snoomba GUI")
        self.label = Tkinter.Label(window, text="Snoomba GUI")
        self.label.pack()

        self.testButton = Tkinter.Button(window, text="Test", command=self.scPubTestMsg)
        self.testButton.pack()

    def scPubMsg(self, msg, gui):
        print("Button Pressed!")
        gui.scPub.publish(msg)
