
import time
import numpy as np
import rpyc
import gui
import arduino


import cartGlobal
import odometry

CART_PORT = 20002
lastMessage = time.time()
TIMEOUT = 5                 # stop cart when navManager stopped
obstacleInfo = []

NUM_DISTANCE_SENSORS = 10
NUM_MEASUREMENTS_PER_SCAN = 11
distanceSensors = []
# VL nah
distanceSensors.append({'sensorID': 0, 'direction':'forward', 'position':'left', 'range':'short', 
    'timestamp':time.time(), 'valueIndex':(0,0), 'installed': True, 'servoIndex':0, 'color':'red', 'rotation':-180})
# VL fern
distanceSensors.append({'sensorID': 1, 'direction':'forward', 'position':'left', 'range':'long', 
    'timestamp':time.time(), 'valueIndex':(0,1), 'installed': True, 'servoIndex':0, 'color':'blue', 'rotation':-180})
# VR nah
distanceSensors.append({'sensorID': 2, 'direction':'forward', 'position':'right', 'range':'short', 
    'timestamp':time.time(), 'valueIndex':(1,0), 'installed': True, 'servoIndex':1, 'color':'red', 'rotation':-180})
# VR fern
distanceSensors.append({'sensorID': 3, 'direction':'forward', 'position':'right', 'range':'long', 
    'timestamp':time.time(), 'valueIndex':(1,1), 'installed': True, 'servoIndex':1, 'color':'blue', 'rotation':-180})
# LM nah
distanceSensors.append({'sensorID': 4, 'direction':'left', 'position':'middle', 'range':'short', 
    'timestamp':time.time(), 'valueIndex':(2,0), 'installed': True, 'servoIndex':2, 'color':'red', 'rotation':90})
# RM nah
distanceSensors.append({'sensorID': 5, 'direction':'right', 'position':'middle', 'range':'short', 
    'timestamp':time.time(), 'valueIndex':(2,1), 'installed': True, 'servoIndex':2, 'color':'red', 'rotation':-90})
# HL nah
distanceSensors.append({'sensorID': 6, 'direction':'backward', 'position':'left', 'range':'short', 
    'timestamp':time.time(), 'valueIndex':(3,0), 'installed': True, 'servoIndex':3, 'color':'red', 'rotation':0})
# HL fern
distanceSensors.append({'sensorID': 7, 'direction':'backward', 'position':'left', 'range':'long', 
    'timestamp':time.time(), 'valueIndex':(3,1), 'installed': True, 'servoIndex':3, 'color':'blue', 'rotation':0})
# HR nah
distanceSensors.append({'sensorID': 8, 'direction':'backward', 'position':'right', 'range':'short', 
    'timestamp':time.time(), 'valueIndex':(3,0), 'installed': True, 'servoIndex':4, 'color':'red', 'rotation':0})
# HR fern
distanceSensors.append({'sensorID': 9, 'direction':'backward', 'position':'right', 'range':'long', 
    'timestamp':time.time(), 'valueIndex':(3,1), 'installed': True, 'servoIndex':4, 'color':'blue', 'rotation':0})

distanceList = np.zeros((NUM_DISTANCE_SENSORS, NUM_MEASUREMENTS_PER_SCAN))
                        

class cartCommands(rpyc.Service):

    def exposed_echo(self, text):
        return text + " from cartControl"

    def exposed_rotateRelative(self, angle):
        cartGlobal.log(f"exposed_rotateRelative, angle: {angle}")
        arduino.sendRotateCommand(int(angle))
        gui.controller.updateTargetRotation(cartGlobal.getOrientation() + int(angle))
        
    def exposed_move(self, direction, speed, distance=10):
        cartGlobal.log(f"exposed_move, direction: {direction}, speed: {speed}, distance: {distance}")
        arduino.sendMoveCommand(direction, speed, distance)

    def exposed_stop(self):
        cartGlobal.log("exposed_stop")
        gui.controller.stopCart()

    def exposed_requestCartOrientation(self):
        return str(cartGlobal.getOrientation())

    def exposed_heartBeat(self):
        global lastMessage
        continue_pygame_loop()

        # watchdog for incoming commands from navManager
        # stop cart if we do not get regular messages
        currTime = int(round(time.time() * 1000))
        cartGlobal.log(currTime - lastMessage)
        if currTime - lastMessage > TIMEOUT:
            if cartGlobal.getCartMoving:
                arduino.sendStopCommand()
        else:
            arduino.sendHeartbeat()
            cartGlobal.log("arduino Heartbeat sent")

        lastMessage = int(round(time.time() * 1000))

    def exposed_getObstacleInfo(self):
        return obstacleInfo

    def exposed_getCartInfo(self):
        return cartGlobal.getOrientation(), cartGlobal.currPosX, cartGlobal.currPosY, cartGlobal.getCartMoving()




def updateDistances(Values):
    global distanceList
    sensorID = Values[0]
    numValues = Values[1]

    #cartGlobal.log(f"updateDistancies: {Values}")

    for i in range(numValues):
        try:
            distanceList[sensorID][i] = float(Values[2 + i])
        except ValueError:
            distanceList[sensorID][i] = 0.0

    # update timestamp of last measurement
    # distanceSensors[sensorID].update({'timestamp': time.time})
    
    for key in distanceSensors[sensorID].keys():
        if key == 'timestamp':
            distanceSensors[sensorID][key] = time.time()


if __name__ == '__main__':

    import threading

    # Logging
    cartGlobal.startlog()
    cartGlobal.setMovementBlocked(False)

    guiThread = threading.Thread(target=gui.startGui, args={})
    guiThread.start()
    cartGlobal.log("cart gui started")

    msgThread = threading.Thread(target=arduino.readMessages, args={})
    cartGlobal.log("start msg reader cart-arduino")
    msgThread.start()
    
    camThread = threading.Thread(target=odometry.trackCartMovements, args={})
    cartGlobal.log("start cart tracker")
    camThread.start()

    from rpyc.utils.server import ThreadedServer
    t = ThreadedServer(cartCommands, port=CART_PORT)
    cartGlobal.log("start cart rpyc listener")
    t.start()

