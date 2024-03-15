'''
The main file that runs the programs loop.
'''

# Uncomment for debugging (main function includes a count for how many startup took)
# import sys
# calls = 0
# def trace(frame, event, arg):
#     global calls
#     if event == "call":
#         filename = frame.f_code.co_filename
#         if "Python" not in filename and "frozen" not in filename and "string" not in filename:
#             lineno = frame.f_lineno
#             # Here I'm printing the file and line number, 
#             # but you can examine the frame, locals, etc too.
#             print("%s @ %s" % (filename, lineno))
#             calls += 1
#     return trace
# 
# sys.settrace(trace)

# This section is for modules that I've added later as they might 
# not have been installed yet
import src.settings as settings
from src.logger import print

# Check tkinter tcl version
import tkinter as tk
from tkinter import messagebox
tcl = tk.Tcl()
acceptedVersions = ["8.6.11", "8.6.12", "8.6.13"]
version = str(tcl.call('info', 'patchlevel'))
if version not in acceptedVersions:
    messagebox.showwarning("Warning", f"Your tkinter version ({version} is not >= 8.6.11) is too old. Windows scaling will be broken with this version.")
    print(f"Your tkinter version ({version} is not >= 8.6.11) is too old. Windows scaling will be broken with this version.")

# Load the UI framework
import src.mainUI as mainUI
import sys
mainUI.CreateRoot()
import src.helpers as helpers
splash = helpers.SplashScreen(mainUI.root, totalSteps=4)
splash.updateProgress(text="Initializing...", step=1)
mainUI.root.update()

try:
    if "DXCamScreenCapture" in settings.GetSettings("Plugins", "Enabled"):
        settings.RemoveFromList("Plugins", "Enabled", "DXCamScreenCapture")
        settings.AddToList("Plugins", "Enabled", "BetterCamScreenCapture")
except: pass

import src.console as console
if settings.GetSettings("User Interface", "hide_console", False) == True:
    console.HideConsole()

import os
try:
    import importlib_metadata
except:
    os.system("pip install importlib_metadata")
    import importlib_metadata
listOfRequirementsAddedLater = ["colorama", "bettercam", "matplotlib", "pywebview", "vdf", "deep-translator", "Babel", "PyQt5"]
# Get list of installed modules using importlib
installed = [i.name for i in importlib_metadata.distributions()]
requirementsset = set(listOfRequirementsAddedLater)
installedset = set(installed)
missing = requirementsset - installedset

if missing:
    for modules in missing:
        print("installing" + " " + modules)
        os.system("pip install" + " " + modules)

import src.variables as variables # Stores all main variables for the program
# Check that all requirements from requirements.txt are installed
with open(variables.PATH + r"\requirements.txt") as f:
    requirements = f.read().splitlines()

installed = [i.name for i in importlib_metadata.distributions()]
requirementsset = set(requirements)
installedset = set(installed)
missing = requirementsset - installedset

if missing:
    for modules in missing:
        if "--upgrade --no-cache-dir gdown" in modules:
            pass
        elif "sv_ttk" in modules:
            pass
        elif "playsound2" in modules:
            os.system("pip uninstall -y playsound")
            os.system("pip install playsound2")
        else:
            print("installing" + " " + modules)
            os.system("pip install" + " " + modules)
else:
    pass

# Load the rest of the modules
import time
import json
from src.logger import print
import src.logger as logger
import traceback
import src.translator as translator
import src.controls as controls
import psutil
import cv2
import src.scsLogReader as LogReader
from src.server import SendCrashReport, Ping

helpers.RunEvery(60, lambda: Ping())

logger.printDebug = settings.GetSettings("logger", "debug")
if logger.printDebug == None:
    logger.printDebug = False
    settings.CreateSettings("logger", "debug", False)
    
def GetEnabledPlugins():
    global enabledPlugins
    enabledPlugins = settings.GetSettings("Plugins", "Enabled")
    if enabledPlugins == None:
        enabledPlugins = [""]

panels = []
def FindPlugins(reloadFully=False):
    global plugins
    global panels
    global pluginObjects
    global pluginNames
    global splash
    
    closeAfter = False
    try:
        if splash == None:
            closeAfter = True
            splash = helpers.SplashScreen(mainUI.root, totalSteps=4)
            splash.updateProgress(text="Initializing...", step=1)
    except:
        closeAfter = True
        splash = helpers.SplashScreen(mainUI.root, totalSteps=4)
        splash.updateProgress(text="Initializing...", step=1)
    
    # Update the list of plugins and panels for the hash check
    pluginNames = GetListOfAllPluginAndPanelNames()
    
    # Find plugins
    path = os.path.join(variables.PATH, "plugins")
    plugins = []
    panels = []
    count = len(os.listdir(path))
    index = 0
    for file in os.listdir(path):
        splash.updateProgress(text=f"Loading plugins... {count-index} remaining.", step=2 + (index / count))
        mainUI.root.update()
        index += 1
        if os.path.isdir(os.path.join(path, file)):
            # Check for main.py
            if "main.py" in os.listdir(os.path.join(path, file)):
                # Check for PluginInformation class
                try:
                    pluginPath = "plugins." + file + ".main"
                    plugin = __import__(pluginPath, fromlist=["PluginInformation"])
                    if plugin.PluginInfo.type == "dynamic":
                        if plugin.PluginInfo.name in enabledPlugins:
                            plugins.append(plugin.PluginInfo)
                    else:
                        panels.append(__import__("plugins." + plugin.PluginInfo.name + ".main", fromlist=["UI", "PluginInfo"]))
                except Exception as ex:
                    print(str(ex.args) + f" [{file}]")
                    pass

    pluginObjects = []
    for plugin in plugins:
        pluginObjects.append(__import__("plugins." + plugin.name + ".main", fromlist=["plugin", "UI", "PluginInfo", "onEnable"]))
        pluginObjects[-1].onEnable()
        
    if closeAfter:
        splash.close()
        del splash
        
def ReloadPluginCode():
    FindPlugins()
    # Use the inbuilt python modules to reload the code of the plugins
    import importlib
    import progress.bar as Bar
    with Bar.PixelBar("Reloading plugins...", max=len(pluginObjects)) as progressBar:
        for plugin in pluginObjects:
            try:
                importlib.reload(plugin)
            except Exception as ex:
                print(ex.args)
                pass
            progressBar.next()
            
    with Bar.PixelBar("Reloading panels...", max=len(panels)) as progressBar:
        for panel in panels:
            try:
                importlib.reload(panel)
            except Exception as ex:
                print(ex.args)
                pass
            progressBar.next()
            
    print("Reloading UI root code...")
    try:
        mainUI.DeleteRoot()
        importlib.reload(mainUI)
        mainUI.CreateRoot()
        mainUI.drawButtons()
    except Exception as ex:
        print(ex.args)
        pass
    print("Reloaded UI root code.")

def RunOnEnable():
    for plugin in pluginObjects:
        try:
            plugin.onEnable()
        except Exception as ex:
            print(ex.args)
            pass
        
import concurrent.futures
import threading
def UpdatePlugins(dynamicOrder, data):
    for plugin in pluginObjects:
        try:
            if plugin.PluginInfo.dynamicOrder == dynamicOrder:
                startTime = time.time()

                pluginData = plugin.plugin(data)
                
                if pluginData != None:
                    data = pluginData
                else:
                    print(f"Plugin '{plugin.PluginInfo.name}' returned NoneType instead of a the data variable. Please make sure that you return the data variable.")
                
                endTime = time.time()    
                data["executionTimes"][plugin.PluginInfo.name] = endTime - startTime
                        
        except Exception as ex:
            print(ex.args + f"[{plugin.PluginInfo.name}]")
            pass
    return data

def GetListOfAllPluginAndPanelNames():
    # Find plugins
    path = os.path.join(variables.PATH, "plugins")
    plugins = []
    for file in os.listdir(path):
        if os.path.isdir(os.path.join(path, file)):
            # Check for main.py
            if "main.py" in os.listdir(os.path.join(path, file)):
                # Check for PluginInformation class
                try:
                    pluginName = file
                    plugins.append(pluginName)
                except Exception as ex:
                    print(ex.args)
                    pass
    
    return plugins

pluginNames = GetListOfAllPluginAndPanelNames()

def InstallPlugins():
    global startInstall
    global splash
    
    list = settings.GetSettings("Plugins", "Installed")
    if list == None:
        settings.CreateSettings("Plugins", "Installed", [])
    
    # Find plugins
    path = os.path.join(variables.PATH, "plugins")
    installers = []
    pluginNames = []
    for file in os.listdir(path):
        if os.path.isdir(os.path.join(path, file)):
            # Check for main.py
            if "main.py" in os.listdir(os.path.join(path, file)):
                # Check for PluginInformation class
                try:
                    # Get installers for plugins that are not installed
                    if file not in settings.GetSettings("Plugins", "Installed"):
                        pluginPath = "plugins." + file + ".install"
                        try:
                            pluginNames.append(f"{file}")
                            installers.append(__import__(pluginPath, fromlist=["install"]))
                        except: # No installer
                            pass
                        
                    
                except Exception as ex:
                    print(ex.args)
                    pass
    
    if installers == []:
        return
    
    wasSplash = False
    try:
        splash.close()
        del splash
        wasSplash = True
    except:
        pass
    
    import tkinter as tk
    from tkinter import ttk
    
    # Create a new tab for the installer
    installFrame = ttk.Frame(mainUI.pluginNotebook, width=600, height=520)
    installFrame.pack(anchor=tk.CENTER, expand=True, fill=tk.BOTH)
    mainUI.pluginNotebook.add(installFrame, text="Plugin Installer")
    mainUI.pluginNotebook.select(mainUI.pluginNotebook.tabs()[-1])
    
    ttk.Label(installFrame, text="The app has detected plugins that have not yet been installed.").pack()
    ttk.Label(installFrame, text="Please install them before continuing.").pack()
    ttk.Label(installFrame, text="").pack()
    ttk.Label(installFrame, text="WARNING: Make sure you trust the authors of the plugins.").pack()
    ttk.Label(installFrame, text="If you are at all skeptical then you can see the install script at").pack()
    ttk.Label(installFrame, text="app/plugins/<plugin name>/installer.py").pack()
    ttk.Label(installFrame, text="").pack()
    
    startInstall = False
    def SetInstallToTrue():
        global startInstall
        startInstall = True
    ttk.Button(installFrame, text="Install plugins", command=lambda: SetInstallToTrue()).pack()
    
    ttk.Label(installFrame, text="").pack()
    ttk.Label(installFrame, text="The following plugins require installation: ").pack()
    # Make tk list object
    listbox = tk.Listbox(installFrame, width=75, height=30)
    listbox.pack()
    # Add the plugins there
    for plugin in pluginNames:
        listbox.insert(tk.END, plugin)
    # Center the listbox text
    listbox.config(justify=tk.CENTER)
    
    while not startInstall:
        mainUI.root.update()
    
    # Destroy all the widgets
    for child in installFrame.winfo_children():
        child.destroy()
        
    # Create the progress indicators
    ttk.Label(installFrame, text="\n\n\n\n\n\n\n").pack()
    currentPlugin = tk.StringVar(installFrame)
    currentPlugin.set("Installing plugins...")
    ttk.Label(installFrame, textvariable=currentPlugin).pack()
    bar = ttk.Progressbar(installFrame, orient=tk.HORIZONTAL, length=200, mode='determinate')
    bar.pack(pady=15)
    percentage = tk.StringVar(installFrame)
    ttk.Label(installFrame, textvariable=percentage).pack()
    ttk.Label(installFrame, text="").pack()
    ttk.Label(installFrame, text="This may take a while...").pack()
    ttk.Label(installFrame, text="For more information check the console.").pack()
    
    mainUI.root.update()
    
    
    index = 0
    import progress.bar as Bar
    with Bar.PixelBar("Installing plugins...", max=len(installers)) as progressBar:
        for installer, name in zip(installers, pluginNames):
            sys.stdout.write(f"\nInstalling '{name}'...\n")
            currentPlugin.set(f"Installing '{name}'...")
            bar.config(value=(index / len(installers)) * 100)
            percentage.set(f"{round((index / len(installers)) * 100)}%")
            mainUI.root.update()
            try:
                installer.install()
                settings.AddToList("Plugins", "Installed", name.split(" - ")[0])
            except:
                print(f"Warning. Failed to install '{name}' fully! The plugin might still work though.")
                pass
            index += 1
            os.system("cls")
            progressBar.next()
    
    # Destroy all the widgets
    for child in installFrame.winfo_children():
        child.destroy()
        
    # Remove the tab
    settings.RemoveFromList("Plugins", "OpenTabs", "Plugin Installer")
    variables.RELOADPLUGINS = True
    
    if wasSplash:
        splash = helpers.SplashScreen(mainUI.root, totalSteps=4)
        splash.updateProgress(text="Initializing...", step=1)
    
InstallPlugins()    

def CheckForONNXRuntimeChange():
    change = settings.GetSettings("SwitchLaneDetectionDevice", "switchTo")
    if change != None:
        if change == "GPU":
            splash.updateProgress(text="Uninstalling ONNX...")
            os.system("pip uninstall onnxruntime -y")
            splash.updateProgress(text="Installing ONNX GPU...")
            os.system("pip install onnxruntime-gpu")
        else:
            splash.updateProgress(text="Uninstalling ONNX GPU...")
            os.system("pip uninstall onnxruntime-gpu -y")
            splash.updateProgress(text="Installing ONNX...")
            os.system("pip install onnxruntime")
            
    settings.CreateSettings("SwitchLaneDetectionDevice", "switchTo", None)

def CheckLastKnownVersion():
    lastVersion = settings.GetSettings("User Interface", "version")
    if lastVersion == None:
        settings.UpdateSettings("User Interface", "version", variables.VERSION)
        mainUI.switchSelectedPlugin("plugins.Changelog.main")
        return
    
    if lastVersion != variables.VERSION:
        settings.UpdateSettings("User Interface", "version", variables.VERSION)
        mainUI.switchSelectedPlugin("plugins.Changelog.main")
        return
    
def CloseAllPlugins():
    for plugin in pluginObjects:
        plugin.onDisable()
        del plugin
        

timesLoaded = 0
def LoadApplication():
    global mainUI
    global uiUpdateRate
    global timesLoaded
    global splash

    if timesLoaded > 0:
        try:
            mainUI.DeleteRoot()
            del mainUI
            import src.mainUI as mainUI
            mainUI.CreateRoot()
        except:
            pass
    
    try:
        if splash == None:
            splash = helpers.SplashScreen(mainUI.root, totalSteps=4)
            splash.updateProgress(text="Initializing...", step=1)
    except:
        pass
        
    timesLoaded += 1
        
    CheckForONNXRuntimeChange()

    # Check for new plugin installs
    InstallPlugins()
    
    useSplash = True
    try:
        if splash == None:
            splash = helpers.SplashScreen(mainUI.root, totalSteps=4)
            splash.updateProgress(text="Initializing...", step=1)
    except:
        try:
            splash = helpers.SplashScreen(mainUI.root, totalSteps=4)
            splash.updateProgress(text="Initializing...", step=1)
        except:
            useSplash = False
    
    # Load all plugins 
    if useSplash: splash.updateProgress(text="Loading plugins...", step=2)
    GetEnabledPlugins()
    FindPlugins()
    if useSplash: splash.updateProgress(text="Initializing plugins...", step=3)
    RunOnEnable()
    if useSplash: splash.updateProgress(text="Finishing...", step=4)

    logger.printDebug = settings.GetSettings("logger", "debug")
    if logger.printDebug == None:
        logger.printDebug = False
        settings.CreateSettings("logger", "debug", False)

    # We've loaded all necessary modules
    showCopyrightInTitlebar = settings.GetSettings("User Interface", "TitleCopyright")
    if showCopyrightInTitlebar == None:
        settings.CreateSettings("User Interface", "TitleCopyright", True)
        showCopyrightInTitlebar = True
    
    mainUI.titlePath = "- " + open(settings.currentProfile, "r").readline().replace("\n", "") + " "
    mainUI.UpdateTitle()
    mainUI.root.update()
    mainUI.drawButtons()

    if useSplash: splash.close()
    if useSplash: del splash

    uiUpdateRate = settings.GetSettings("User Interface", "updateRate")
    if uiUpdateRate == None: 
        uiUpdateRate = 0
        settings.CreateSettings("User Interface", "updateRate", 0)

    CheckLastKnownVersion()
    # Show the root window
    mainUI.root.deiconify()
    
    helpers.ShowPopup("\nFound " + str(len(GetListOfAllPluginAndPanelNames())) + " plugins!", "Backend", timeout=3)

LoadApplication()

import hashlib
lastChecksums = {}
def CheckForFileChanges():
    """Will check the plugin main files for changes and reload them if they've changed."""
    global lastChecksums
    # Check if it's the first time running this function
    if lastChecksums == {}:
        for plugin in pluginNames:
            try:
                checksum = hashlib.md5(open(os.path.join(variables.PATH, "plugins", plugin, "main.py"), "rb").read()).hexdigest()
                lastChecksums[plugin] = checksum
            except:
                pass
        return
    
    # Check for changes in the plugins
    for plugin in pluginNames:
        try:
            checksum = hashlib.md5(open(os.path.join(variables.PATH, "plugins", plugin, "main.py"), "rb").read()).hexdigest()
            if checksum != lastChecksums[plugin]:
                print(f"Detected changes in {plugin}...")
                ReloadPluginCode()
                RunOnEnable()
                variables.RELOADPLUGINS = False
                variables.RELOAD = False # Already reloaded
                lastChecksums[plugin] = checksum
                break
        except:
            pass
      
# Check for updates
import src.updater as updater
updater.UpdateChecker()  

data = {}
uiFrameTimer = 0
pluginChangeTimer = time.time()
lastEnableValue = False
if __name__ == "__main__":
    # print(f"Starting took {calls} calls.") # Uncomment for debugging along with the one at the top of the file
    while True:
        # Main Application Loop
        try:
            
            allStart = time.time()
            
            # Remove "last" from the data and set it as this frame's "last"
            try: 
                data.pop("last")
                data = {
                    "last": data, 
                    "executionTimes": {}
                }
            except Exception as ex:
                data = {
                    "last": {},
                    "executionTimes": {}
                }  
            
            if variables.RELOADPLUGINS:
                print("Reloading plugins...")
                ReloadPluginCode()
                RunOnEnable()
                variables.RELOADPLUGINS = False
                variables.RELOAD = False # Already reloaded
            
            if variables.RELOAD:
                print("Reloading application...")
                # Reset the open tabs
                settings.UpdateSettings("User Interface", "OpenTabs", [])
                LoadApplication()
                variables.RELOAD = False
            
            # Update the input manager.
            controlsStartTime = time.time()
            data = controls.plugin(data)
            controlsEndTime = time.time()
            data["executionTimes"]["Control callbacks"] = controlsEndTime - controlsStartTime
            
            # Check for plugin changes (every second)
            pluginChangeTime = time.time()
            if time.time() - pluginChangeTimer > 1:
                pluginChangeTimer = time.time()
                CheckForFileChanges()
            pluginChangeEndTime = time.time()
            data["executionTimes"]["Filesystem Check"] = pluginChangeEndTime - pluginChangeTime
            
            # Check for log file changes
            logCheckTime = time.time()
            data = LogReader.plugin(data)
            logCheckEndTime = time.time()
            data["executionTimes"]["Log Check"] = logCheckEndTime - logCheckTime
            
            
            try:
                if variables.APPENDDATANEXTFRAME != None or variables.APPENDDATANEXTFRAME != [] or variables.APPENDDATANEXTFRAME != {} or variables.APPENDDATANEXTFRAME != "":
                    # Merge the two dictionaries
                    data.update(variables.APPENDDATANEXTFRAME)
                    variables.APPENDDATANEXTFRAME = None
            except: pass
            
            if variables.UPDATEPLUGINS:
                GetEnabledPlugins()
                FindPlugins()
                variables.UPDATEPLUGINS = False
                
            for runner in helpers.runners:
                # [duration, function, time.time(), args, kwargs]
                duration, function, lastRun, args, kwargs = runner
                if time.time() - lastRun > duration:
                    try:
                        function(*args, **kwargs)
                    except Exception as ex:
                        print(ex.args)
                    
                    helpers.runners.remove(runner)
                
            start = time.time()
            popupCount = 0
            for popup in helpers.popups:
                try:
                    popup.update(popupCount)
                    popupCount += 1
                except:
                    try:
                        popup.destroy()
                    except:
                        pass
                    helpers.popups.remove(popup)
                
            popupCount = 0
            for popup in helpers.timeoutlessPopups:
                try:
                    popup.update(popupCount)
                    popupCount += 1
                except:
                    try:
                        popup.destroy()
                    except:
                        pass
                    helpers.timeoutlessPopups.remove(popup)    
                
            end = time.time()
            data["executionTimes"]["Popups"] = end - start
            
            if variables.ENABLELOOP != lastEnableValue:
                lastEnableValue = variables.ENABLELOOP
                helpers.ShowPopup("\nThe main loop is now " + ("enabled" if variables.ENABLELOOP else "disabled") + "!", "Backend", timeout=2)
                
            # Enable / Disable the main loop
            if variables.ENABLELOOP == False:
                start = time.time()
                mainUI.update(data)
                end = time.time()
                data["executionTimes"]["UI"] = end - start
                data = UpdatePlugins("last", data)
                allEnd = time.time()
                data["executionTimes"]["all"] = allEnd - allStart
                try:
                    cv2.destroyWindow("Lane Assist")
                except:
                    pass
                try:
                    cv2.destroyWindow('Traffic Light Detection - B/W')
                except:
                    pass
                try:
                    cv2.destroyWindow('Traffic Light Detection - Red/Yellow/Green')
                except:
                    pass
                try:
                    cv2.destroyWindow('Traffic Light Detection - Final')
                except:
                    pass
                try:
                    cv2.destroyWindow('TruckStats')
                except:
                    pass
                
                variables.FRAMECOUNTER += 1
                
                continue
            
            
            data = UpdatePlugins("before image capture", data)
            data = UpdatePlugins("image capture", data)
            
            data = UpdatePlugins("before lane detection", data)
            data = UpdatePlugins("lane detection", data)
            
            data = UpdatePlugins("before controller", data)
            data = UpdatePlugins("controller", data)
            
            data = UpdatePlugins("before game", data)
            data = UpdatePlugins("game", data)
            
            data = UpdatePlugins("before UI", data)

            # Calculate the execution time of the UI
            start = time.time()
            uiFrameTimer += 1
            if uiFrameTimer > uiUpdateRate:
                mainUI.update(data)
                uiFrameTimer = 0
            end = time.time()
            data["executionTimes"]["UI"] = end - start
            
            data = UpdatePlugins("last", data)
            
            # And then the entire app
            allEnd = time.time()
            data["executionTimes"]["all"] = allEnd - allStart

            # Check if the frame took more than 200ms (5fps)
            if (allEnd - allStart) - data["executionTimes"]["UI"] > 0.2:
                print(f"Frame took {round((allEnd - allStart) * 1000)}ms to execute!")
                # Check if the anomalousFrames folder exists
                if not os.path.exists(os.path.join(variables.PATH, "anomalousFrames")):
                    os.mkdir(os.path.join(variables.PATH, "anomalousFrames"))
                # Save a new text file with the data
                with open(os.path.join(variables.PATH, "anomalousFrames", f"{time.time()}.txt"), "w") as f:
                    # Go throught each key and try and write it
                    for key in data:
                        try:
                            f.write(f"{key}: {data[key]}\n")
                        except:
                            pass
                        
            variables.FRAMECOUNTER += 1
        
        except Exception as ex:
            try:
                if settings.GetSettings("User Interface", "hide_console") == True:
                    console.RestoreConsole()
            except:
                pass
            if ex.args != ('The main window has been closed.', 'If you closed the app this is normal.'):
                import keyboard
                # Press the F1 key to pause the game
                keyboard.press_and_release("F1")
                from tkinter import messagebox
                import traceback
                exc = traceback.format_exc()
                traceback.print_exc()
                # Get the user name
                username = os.getlogin()
                # Send a crash report
                SendCrashReport("Main loop crash.", exc.replace(username, "censored"))
                if not messagebox.askretrycancel("Error", translator.Translate("The application has encountered an error in the main thread!\nPlease either retry execution or close the application (cancel)!\n\n") + exc):
                    break
                else:
                    pass
            else:
                CloseAllPlugins()
                try:
                    if settings.GetSettings("User Interface", "hide_console") == True:
                        console.CloseConsole()
                except:
                    pass
                break
