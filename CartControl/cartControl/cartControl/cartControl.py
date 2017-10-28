import rpyc
import gui
import arduino
import time
import numpy as np

orientation = 0
arduinoStatus = 0
movementBlocked = False

NUM_DISTANCE_SENSORS = 12
NUM_MEASUREMENTS_PER_SCAN = 11
distanceSensors = []
distanceSensors.append({'sensorID': 0, 'direction':'forward', 'position':'left', 'range':'short',
    'timestamp':time.time(), 'valueIndex':(0,0), 'installed': True, 'servoIndex':0, 'color':'red', 'rotation':-180})
distanceSensors.append({'sensorID': 1, 'direction':'forward', 'position':'left', 'range':'long', 
    'timestamp':time.time(), 'valueIndex':(0,1), 'installed': True, 'servoIndex':0, 'color':'blue', 'rotation':-180})
distanceSensors.append({'sensorID': 2, 'direction':'forward', 'position':'right', 'range':'short', 
    'timestamp':time.time(), 'valueIndex':(1,0), 'installed': True, 'servoIndex':1, 'color':'red', 'rotation':-180})
distanceSensors.append({'sensorID': 3, 'direction':'forward', 'position':'right', 'range':'long', 
    'timestamp':time.time(), 'valueIndex':(1,1), 'installed': True, 'servoIndex':1, 'color':'blue', 'rotation':-180})
distanceSensors.append({'sensorID': 4, 'direction':'backward', 'position':'left', 'range':'short', 
    'timestamp':time.time(), 'valueIndex':(2,0), 'installed': True, 'servoIndex':2, 'color':'red', 'rotation':0})
distanceSensors.append({'sensorID': 5, 'direction':'backward', 'position':'left', 'range':'long', 
    'timestamp':time.time(), 'valueIndex':(2,1), 'installed': True, 'servoIndex':2, 'color':'blue', 'rotation':0})
distanceSensors.append({'sensorID': 6, 'direction':'backward', 'position':'right', 'range':'short', 
    'timestamp':time.time(), 'valueIndex':(3,0), 'installed': True, 'servoIndex':3, 'color':'red', 'rotation':0})
distanceSensors.append({'sensorID': 7, 'direction':'backward', 'position':'right', 'range':'long', 
    'timestamp':time.time(), 'valueIndex':(3,1), 'installed': True, 'servoIndex':3, 'color':'blue', 'rotation':0})
distanceSensors.append({'sensorID': 8, 'direction':'left', 'position':'middle', 'range':'short', 
    'timestamp':time.time(), 'valueIndex':(3,0), 'installed': True, 'servoIndex':4, 'color':'red', 'rotation':90})
distanceSensors.append({'sensorID': 9, 'direction':'left', 'position':'middle', 'range':'long', 
    'timestamp':time.time(), 'valueIndex':(3,1), 'installed': False, 'servoIndex':4, 'color':'blue', 'rotation':90})
distanceSensors.append({'sensorID': 10, 'direction':'right', 'position':'middle', 'range':'short', 
    'timestamp':time.time(), 'valueIndex':(3,0), 'installed': True, 'servoIndex':5, 'color':'red', 'rotation':-90})
distanceSensors.append({'sensorID': 11, 'direction':'right', 'position':'middle', 'range':'long', 
    'timestamp':time.time(), 'valueIndex':(3,1), 'installed': False, 'servoIndex':5, 'color':'blue', 'rotation':-90})

distanceList = np.zeros((NUM_DISTANCE_SENSORS, NUM_MEASUREMENTS_PER_SCAN))
                        

class cartCommands(rpyc.Service):

    def exposed_echo(self, text):
        return text + " from CartControl"

    def exposed_rotateRelative(self, angle):
        arduino.sendRotateCommand(angle)
        gui.updateTargetRotation(CartControl.orientation + angle)

    def exposed_move(self, direction, speed):
        arduino.sendMoveCommand(direction, speed)

    def exposed_stop(self):
        gui.stopCart()

    def exposed_requestCartOrientation(self):
        arduino.getCartOrientation()
        #arduino.readMessages()
        #lastMessage = int(round(time.time() * 1000))
        #orientation = arduino.getOrientation()
        #print "orientation in getCartOrientation: " + str(orientation)
        #return str(orientation)

    def exposed_heartBeat(self):
        global lastMessage
        continue_pygame_loop()

        currTime = int(round(time.time() * 1000))
        print (currTime-lastMessage)
        if currTime - lastMessage > TIMEOUT:
            arduino.sendStopCommand()
        else:
            arduino.sendHeartbeat()
            print("arduino Heartbeat sent")

        lastMessage = int(round(time.time() * 1000))

    def exposed_requestDistance(self, direction):
        exposed_move(self, direction, 0)

    pass


def updateDistances(Values):
    global distanceList
    sensorID = Values[1];
    for i in range(NUM_MEASUREMENTS_PER_SCAN):
        distanceList[sensorID][i] = Values[2+i]

    # update timestamp
#    distanceSensors[sensorID].update({'timestamp': time.time})

    for key in distanceSensors[sensorID].keys():
        if key == 'timestamp':
            distanceSensors[sensorID][key] = time.time()
    #print distanceSensors[sensorID]

if __name__ == '__main__':

    import threading

    guiThread = threading.Thread(target = gui.startGui, args={})
    guiThread.start()
    print "cart gui started"

    msgThread = threading.Thread(target = arduino.readMessages, args={})
    msgThread.start()
    #start_new_thread(arduino.readMessages,())

    from rpyc.utils.server import ThreadedServer
    t = ThreadedServer(cartCommands, port = 20002)
    print "start cartCommands listener"
    t.start()

