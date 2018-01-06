import time
import cartControl
import gui
import serial
import sys

ser=None

def initSerial(comPort):
    global ser
  
    while True:
        try:
            ser = serial.Serial(port=comPort, baudrate=115200, timeout=0.5)
            print("Serial comm to arduino established")
            break

        except:
            print("exception on serial connect with " + comPort)
            time.sleep(1)

#####################################
# readMessages runs in its own thread
#####################################
def readMessages():

    global ser

    while ser == None:
       time.sleep(0.1)

    while ser.is_open:

        while ser.inWaiting() > 0:
            recvB = ser.readline()
            recv = recvB.decode()
            msgID = recvB[0:3].decode()

            #print("in read message: ", msgID)

            if msgID == "!A0":     #"cart ready"
                cartControl.arduinoStatus = 1

            elif msgID == "!A1":    #"distance,":
                #!A1,<sensorID>,<ANZ_MESSUNGEN_PRO_SCAN>,[ANZ_MESSUNGEN_PRO_SCAN<value>,]
                messageItems =  [int(e) if e.isdigit() else e for e in recv.split(',')]
                cartControl.updateDistances(messageItems[1:])
                gui.controller.showDistances(messageItems[1])
                

            elif msgID == "!A2":    #"obstacle:":
                #!A2,<distance>
                distance = float(recv.split(",")[1])
                cartControl.movementBlocked = True
                cartControl.timeMovementBlocked = time.time()
                gui.controller.updateDistanceSensorObstacle(distance)

            elif msgID == "!A4":    #"abyss:":
                #!A3,<distance>
                distance = float(recv.split(",")[1])
                cartControl.movementBlocked = True                
                cartControl.timeMovementBlocked = time.time()
                gui.controller.updateDistanceSensorAbyss(distance)

            elif msgID == "!A5":    # orientation x:":
                #!A5,<orientation>
                newOrientation = float(recv.split(",")[1])
                if int(cartControl.orientation) != int(newOrientation):
                    cartControl.orientation = newOrientation
                    print ("new cart orientation: " + str(cartControl.orientation))

                    # if cart is in rotation no blocking exists
                    if cartControl.movementBlocked:
                        cartControl.movementBlocked = False

                # no new orientation, check for blocked movement and stop cart
                else:
                    if cartControl.movementBlocked:
                        if time.time() - cartControl.timeMovementBlocked > 3:
                            sendStopCommand()
                            cartControl.movementBlocked = False
                        
            else:
                try:
                    print ("<-A " + recv)
                except:
                    print ("Unexpected error on reading messages: ", sys.exc_info()[0])
                    pass

        pass


def sendMoveCommand(dir, speed):

    msg = b'1' + bytes(dir) + bytes(speed).zfill(3) + b'\n'
    print("Send move " + str(msg))
    ser.write(msg) 


def sendRotateCommand(relAngle):

    if relAngle > 0:   # rotate anticlock
        msg = bytes(str(abs(relAngle)+3000)+'\n','ascii')
        print("Send rotate " + str(msg))
        ser.write(msg) 

    if relAngle < 0:
        msg = bytes(str(abs(relAngle)+2000)+'\n','ascii')
        print("Send rotate " + str(msg))
        ser.write(msg) 

def sendStopCommand():

    msg = b'4' + b'\n'
    print("Send stop " + str(msg))
    ser.write(msg)


def getCartOrientation():

    msg = b'5' + b'\n'
    #print("get Orientation " + msg)
    ser.write(msg)


def sendReadSensorCommand(sensorID):

    msg = b'7' + bytes(sensorID) + b'\n'
    print("Send readSensor " + str(msg))
    ser.write(msg) 

def sendHeartbeat():

    msg = b'9' + b'\n'
    ser.write(msg)

