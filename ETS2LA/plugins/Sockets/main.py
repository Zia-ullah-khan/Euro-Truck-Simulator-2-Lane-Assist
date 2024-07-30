from ETS2LA.plugins.runner import PluginRunner
import multiprocessing
import websockets
import threading
import logging
import asyncio
import json
import time

runner:PluginRunner = None # This will be set at plugin startup
send = ""

async def server(websocket):
    print("Client Connected!")
    while True:
        try:
            await websocket.send(send)
            # Wait for acknowledgment from client
            ack = await websocket.recv()
        except:
            print("Client disconnected.")
            break
        if ack != "ok":
            print(f"Unexpected message from client: {ack}")
    

def position(data):
    send = ""
    send += "x:" + str(data["truckPlacement"]["coordinateX"]) + ";"
    send += "y:" + str(data["truckPlacement"]["coordinateY"]) + ";"
    send += "z:" + str(data["truckPlacement"]["coordinateZ"]) + ";"
    rotationX = data["truckPlacement"]["rotationX"] * 360
    if rotationX < 0: rotationX += 360
    send += "rx:" + str(rotationX) + ";"
    rotationY = data["truckPlacement"]["rotationY"] * 360
    send += "ry:" + str(rotationY) + ";"
    rotationZ = data["truckPlacement"]["rotationZ"] * 360
    if rotationZ < 0: rotationZ += 360
    send += "rz:" + str(rotationZ) + ";"
    return send

def traffic_lights(data):
    try:
        send = "JSONTrafficLights:" + json.dumps(data["TrafficLights"]) + ";"
    except:
        for i in range(0, len(data["TrafficLights"])):
            data["TrafficLights"][i] = data["TrafficLights"][i].json()
        send = "JSONTrafficLights:" + json.dumps(data["TrafficLights"]) + ";"
    return send

def speed(data):
    send = "speed:" + str(data["truckFloat"]["speed"]) + ";"
    return send

lastVehicles = [""]
lastVehicleString = ""
def vehicles(data):
    global lastVehicles, lastVehicleString
    if data["vehicles"] == lastVehicles:
        return lastVehicleString
    
    if data["vehicles"] is not None:
        newVehicles = []
        for vehicle in data["vehicles"]:
            if isinstance(vehicle, dict):
                newVehicles.append(vehicle)
            else:
                try:
                    newVehicles.append(vehicle.json())
                except:
                    try:
                        newVehicles.append(vehicle.__dict__)
                    except:
                        pass
        data["vehicles"] = newVehicles
    
    send = "JSONvehicles:" + json.dumps(data["vehicles"]) + ";"
    lastVehicles = data["vehicles"]
    lastVehicleString = send
    return send

lastSteeringPoints = []
def steering(data):
    try:
        global lastSteeringPoints
        steeringPoints = []
        if data["steeringPoints"] is not None:
            for point in data["steeringPoints"]:
                steeringPoints.append(point)
            lastSteeringPoints = steeringPoints
        else:
            steeringPoints = lastSteeringPoints
            
        
        send = "JSONsteeringPoints:" + json.dumps(steeringPoints) + ";"
        return send
    except:
        return "JSONsteeringPoints:[];"

async def start_server(func):
    async with websockets.serve(func, "localhost", 37522):
        await asyncio.Future() # run forever
        
def run_server_thread():
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(start_server(server))
    loop.run_forever()

def Initialize():
    global TruckSimAPI
    global socket
    
    TruckSimAPI = runner.modules.TruckSimAPI
    TruckSimAPI.Initialize()
    TruckSimAPI.TRAILER = True
    
    socket = threading.Thread(target=run_server_thread)
    socket.start()
    
    print("Visualization sockets waiting for client...")

# Example usage in your server function
def plugin():
    global send
    data = TruckSimAPI.run()
    data["steeringPoints"] = runner.GetData(["Map"])[0]
    data["vehicles"] = runner.GetData(["tags.vehicles"])[0] # Get the cars
    data["TrafficLights"] = runner.GetData(["tags.TrafficLights"])[0] # Get the traffic lights
    
    tempSend = ""
    tempSend += position(data)
    tempSend += speed(data)
    tempSend += vehicles(data)
    tempSend += traffic_lights(data)
    tempSend += steering(data)
    
    send = tempSend