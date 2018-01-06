import rpyc
import gui
import arduino
import time
import numpy as np

cartPort = 20002

orientation = 0
arduinoStatus = 0
movementBlocked = False
timeMovementBlocked = 0
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
        arduino.sendRotateCommand(int(angle))
        gui.controller.updateTargetRotation(orientation + int(angle))
        
    def exposed_move(self, direction, speed):
        arduino.sendMoveCommand(direction, speed)

    def exposed_stop(self):
        gui.stopCart()

    def exposed_requestCartOrientation(self):
        return str(orientation)

    def exposed_heartBeat(self):
        global lastMessage
        continue_pygame_loop()

        # watchdog for incoming commands from navManager
        # stop cart if we do not get regular messages
        currTime = int(round(time.time() * 1000))
        print (currTime-lastMessage)
        if currTime - lastMessage > TIMEOUT:
            arduino.sendStopCommand()
        else:
            arduino.sendHeartbeat()
            print("arduino Heartbeat sent")

        lastMessage = int(round(time.time() * 1000))

    def exposed_getObstacleInfo(self):
        return obstacleInfo

    def exposed_requestDistance(self, direction):
        exposed_move(self, direction, 0)

    pass


def updateDistances(Values):
    global distanceList
    sensorID = Values[0]
    numValues = Values[1]

    print("updateDistancies: ", Values)

    for i in range(numValues):
        try:
            distanceList[sensorID][i] = float(Values[2+i])
        except ValueError:
            distanceList[sensorID][i] = 0.0

    # update timestamp of last measurement
    # distanceSensors[sensorID].update({'timestamp': time.time})
    
    for key in distanceSensors[sensorID].keys():
        if key == 'timestamp':
            distanceSensors[sensorID][key] = time.time()


if __name__ == '__main__':

    import threading

    guiThread = threading.Thread(target = gui.startGui, args={})
    guiThread.start()
    print ("cart gui started")

    msgThread = threading.Thread(target = arduino.readMessages, args={})
    msgThread.start()
    #start_new_thread(arduino.readMessages,())


    from rpyc.utils.server import ThreadedServer
    t = ThreadedServer(cartCommands, port = cartPort)
    print ("start cartCommands listener")
    t.start()

