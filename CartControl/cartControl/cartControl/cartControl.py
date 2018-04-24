
import time
import threading
import numpy as np
import rpyc
from rpyc.utils.server import ThreadedServer
import gui
import arduino

import cartGlobal
import odometry


CART_PORT = 20003

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


def cartInit(): 

    cartGlobal.setMovementBlocked(False)

    # start cart control gui and initialization
    guiThread = threading.Thread(target=gui.startGui, args={})
    guiThread.start()
    print("cart gui started")

    # start serial reader for arduino messages
    msgThread = threading.Thread(target=arduino.readMessages, args={})
    print("start msg reader cart-arduino")
    msgThread.start()

    if not cartGlobal.isOdometryRunning():
        # start odometry thread    
        camThread = threading.Thread(target=odometry.trackCartMovements, args={})
        print("start cart tracker")
        camThread.start()
        cartGlobal.setOdometryRunning(True)

    print("gui, message and odometry threads startet")

    # wait for cart startup finished
    print("cart - wait for cart to startup ...")
    timeout = time.time() + 10
    while cartGlobal.arduinoStatus == 0 or time.time() < timeout:
        time.sleep(1)

    if cartGlobal.arduinoStatus == 0:
        print("cart - timeout in startup, terminating")
        raise SystemExit()


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




class cartCommands(rpyc.Service):

    def exposed_startListening(self, logIP, logPort):
        '''
        a first call from navManager
        '''
        if not cartGlobal.standAloneMode:
            try:
                cartGlobal.navManager = rpyc.connect(logIP, logPort)
                cartGlobal.navManager.root.connectionStatus("cart", True)
                cartGlobal.log("logger connection established")

            except:
                print("cart - could not establish connection to logger")
                raise SystemExit()

        if not cartGlobal.standAloneMode:
            cartGlobal.navManager.root.processStatus("cart",True)

  
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
       
        # watchdog for incoming commands from navManager
        # stop cart if we do not get regular messages
        currTime = int(round(time.time() * 1000))
        cartGlobal.log(currTime - lastMessage)
        if currTime - lastMessage > TIMEOUT:
            if cartGlobal.isCartMoving:
                arduino.sendStopCommand()
        else:
            arduino.sendHeartbeat()
            cartGlobal.log("arduino Heartbeat sent")

        lastMessage = int(round(time.time() * 1000))

    def exposed_getObstacleInfo(self):
        return obstacleInfo

    def exposed_getCartInfo(self):
        posX, posY = cartGlobal.getCartPosition()
        return cartGlobal.getOrientation(), posX, posY, cartGlobal.isCartMoving(), cartGlobal.isCartRotating()



if __name__ == '__main__':

    print ("startup cart")
    cartInit()

    if not cartGlobal.standAloneMode:

        cartGlobal.log("wait for messages from navManager ...")
        t = ThreadedServer(cartCommands, port=CART_PORT)
        t.start()

