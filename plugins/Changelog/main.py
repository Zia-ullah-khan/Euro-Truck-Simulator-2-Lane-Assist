"""
This is an example of a panel (type="static"), they will be updated if they are open.
If you need to make a plugin that is updated in the bg then check the Plugin example!
"""


from plugins.plugin import PluginInformation
from src.logger import print

PluginInfo = PluginInformation(
    name="Changelog", # This needs to match the folder name under plugins (this would mean plugins\Panel\main.py)
    description="Will show the latest changelog.",
    version="0.1",
    author="Tumppi066",
    url="https://github.com/Tumppi066/Euro-Truck-Simulator-2-Lane-Assist",
    type="static" # = Panel
)

import tkinter as tk
from tkinter import ttk
import src.helpers as helpers
import src.mainUI as mainUI
import src.variables as variables
import src.settings as settings
import os

class UI():
    try: # The panel is in a try loop so that the logger can log errors if they occur
        
        def __init__(self, master) -> None:
            self.master = master # "master" is the mainUI window
            self.exampleFunction()
        
        def destroy(self):
            self.done = True
            self.root.destroy()
            del self

        
        def exampleFunction(self):
            
            try:
                self.root.destroy() # Load the UI each time this plugin is called
            except: pass
            
            self.root = tk.Canvas(self.master, width=600, height=520, border=0, highlightthickness=0)
            self.root.grid_propagate(0) # Don't fit the canvas to the widgets
            self.root.pack_propagate(0)
            
            self.text = tk.Text(self.root, width=600, height=520, border=0, highlightthickness=0)
            
            lineNumber = 0
            for line in variables.CHANGELOG:
                count = 0
                if "#" in line:
                    line = line.replace("#", ">")
                    self.text.insert("end", line)
                else:
                    self.text.insert("end", line)
                
                lineNumber += 1
            
            self.text.config(state="disabled")
            self.text.pack(anchor="center", expand=False)
            
            self.root.pack(anchor="center", expand=False)
            self.root.update()
        
        
        def update(self, data): # When the panel is open this function is called each frame 
            self.root.update()
    
    
    except Exception as ex:
        print(ex.args)