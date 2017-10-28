import serial
import time
import cartControl
import gui

ser=None

def initSerial(comPort):
    global ser
  
    while True:
        try:
            ser = serial.Serial(port=comPort, baudrate=115200, timeout=0.1)
            print("Serial comm to arduino established")
            break

        except:
            print("exception on serial connect with " + comPort)
            time.sleep(1)


# readMessages runs in its own thread
def readMessages():

    global ser

    while ser == None:
       time.sleep(0.1)

    while ser.is_open:

        while ser.inWaiting() > 0:
            recv = ser.readline()

            if recv[0:10] == "cart ready":
                cartControl.arduinoStatus = 1

            elif recv[0:14] == "orientation x:":
                newOrientation = float(recv[14:])
                if int(cartControl.orientation) != int(newOrientation):
                    cartControl.orientation = newOrientation
                    print ("new cart orientation: " + str(cartControl.orientation))

            elif recv[0:9] == "distance,":
                distanceValues =  [int(e) if e.isdigit() else e for e in recv.split(',')]
                cartControl.updateDistances(distanceValues)
                cartControl.movementBlocked = True

            elif recv[0:9] == "obstacle:":
                distance = float(recv[9:])
                cartControl.movementBlocked = True           
                gui.controller.updateDistanceSensorObstacle(distance)

            elif recv[0:6] == "abyss:":
                distance = float(recv[6:])
                cartControl.movementBlocked = True           
                gui.controller.updateDistanceSensorAbyss(distance)

            elif recv[0:15] == "sensorDistance:":
                #print("updateDistanceMsg: " + recv)
                gui.controller.updateSensorDistance(recv[15:-2])

            else:
                try:
                    print ("<-A " + recv)
                except:
                    print ("Unexpected error:", sys.exc_info()[0])
                    pass

        pass


def sendMoveCommand(dir, speed):

    msg = '1' + str(dir) + str(speed).zfill(3) + '\n'
    print("Send move " + msg)
    ser.write(msg) 


def sendRotateCommand(relAngle):

    if relAngle > 0:   # rotate anticlock
        msg = '2' + str(relAngle).zfill(3) + '\n'
        print("Send rotate " + msg)
        ser.write(msg) 

    if relAngle < 0:
        msg = '3' + str(-relAngle).zfill(3) + '\n'
        print("Send rotate " + msg)
        ser.write(msg) 

def sendStopCommand():

    msg = '4' + '\n'
    print("Send stop " + msg)
    ser.write(msg)


def getCartOrientation():

    msg = '5' + '\n'
    #print("get Orientation " + msg)
    ser.write(msg)


def sendScanServoCommand(servoID):

    msg = '6' + str(servoID) + '\n'
    print("Send scanServo " + msg)
    ser.write(msg) 


def sendReadSensorCommand(sensorID):

    msg = '7' + str(sensorID) + '\n'
    print("Send readSensor " + msg)
    ser.write(msg) 

def sendHeartbeat():

    ser.write('9' + '\n')

