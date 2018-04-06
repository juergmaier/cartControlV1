
import sys
import time
import serial

import gui
import cartGlobal

ser = None


def initSerial(comPort):

    global ser
    
    cartGlobal.arduinoStatus = 0

    while True:
        try:
            ser = serial.Serial(comPort)
            # try to reset the arduino
            ser.setDTR(False)
            time.sleep(0.1)
            ser.setDTR(True)
            ser.baudrate = 115200
            cartGlobal.log("Serial comm to arduino established")
            return

        except:
            cartGlobal.log(f"exception on serial connect with {comPort} {sys.exc_info()[0]}")
            time.sleep(3)

#####################################
# readMessages runs in its own thread
#####################################
def readMessages():

    import cartControl

    global ser

    while ser is None:
        time.sleep(0.1)

    while ser.is_open:

        while ser.inWaiting() > 0:
            recvB = ser.readline()
            try:
                recv = recvB.decode()
            except:
                cartGlobal.log("problem with decoding cart msg")
            msgID = recvB[0:3].decode()

            #cartGlobal.log("in read message: ", msgID)

            if msgID == "!A0":     #"cart ready"
                cartGlobal.arduinoStatus = 1

            elif msgID == "!A1":    #"distance,":
                #!A1,<sensorID>,<ANZ_MESSUNGEN_PRO_SCAN>,[ANZ_MESSUNGEN_PRO_SCAN<value>,]
                messageItems = [int(e) if e.isdigit() else e for e in recv.split(',')]
                cartControl.updateDistances(messageItems[1:])
                gui.controller.showDistances(messageItems[1])
                

            elif msgID == "!A2":    #"obstacle:":
                #!A2,<distance>
                try:
                    distance = float(recv.split(",")[1])
                except:
                    cartGlobal.log(f"exception with message: {recv[:-2]}")
                cartGlobal.setMovementBlocked(True)
                gui.controller.updateDistanceSensorObstacle(distance)


            elif msgID == "!A3":    #"abyss:":
                #!A3,<distance>
                try:
                    distance = float(recv.split(",")[1])
                except:
                    cartGlobal.log(f"exception with message: {recv[:-2]}")
                cartGlobal.setMovementBlocked(True)
                gui.controller.updateDistanceSensorAbyss(distance)

            elif msgID == "!A5":    #"cart stopped":
                #!A3
                cartGlobal.setMovementBlocked(False)
                cartGlobal.setCartRotating(False)
                cartGlobal.setCartMoving(False)
                cartGlobal.log("!A5 cart stopped received from Arduino")

            elif msgID == "!A9":    # orientation x:":
                #!A9,<orientation>
                newOrientation = round(float(recv.split(",")[1]))

                # test for change of orientation
                if  cartGlobal.getOrientation() != newOrientation:
                    cartGlobal.setOrientation(newOrientation)
                    cartGlobal.log(f"new cart orientation: {newOrientation}")

                    # if cart is in rotation no blocking exists
                    if cartGlobal.getMovementBlocked()[0]:
                        cartGlobal.setMovementBlocked(False)

                # no new orientation, check for blocked movement and stop cart
                else:
                    blocked, startTime = cartGlobal.getMovementBlocked()
                    if blocked:
                        if time.time() - startTime > 3:
                            if cartGlobal.isCartMoving():
                                cartGlobal.log("cart stopped by MovementBlocked")
                                sendStopCommand()
                                cartGlobal.setMovementBlocked(False)
                        
            else:
                try:
                    cartGlobal.log("<-A " + recv[:-2])
                except:
                    cartGlobal.log(f"Unexpected error on reading messages: {sys.exc_info()[0]}")

            time.sleep(0.001)    # give other threads a chance


def sendMoveCommand(direction, speed, distanceMm):

    cartGlobal.setCartMoveDistance(distanceMm)
    duration = ((distanceMm / speed) * 2500) + 1000
    moveMsg = bytes('1' + str(direction) + str(speed).zfill(3) + str(int(duration)).zfill(4) + '\n', 'ascii')
    ser.write(moveMsg) 

    cartGlobal.setMovementBlocked(False)
    cartGlobal.setCartMoving(True)

    cartGlobal.log(f"Send move {moveMsg}, speed: {int(speed)}, distance: {distanceMm}, duration: {duration}")


def sendRotateCommand(relAngle):

    if relAngle > 0:   # rotate counterclock
        msg = bytes(str(abs(relAngle)+2000)+'\n', 'ascii')
        cartGlobal.log("Send rotate counterclock " + str(msg))
        ser.write(msg) 

    if relAngle < 0:
        msg = bytes(str(abs(relAngle)+3000)+'\n', 'ascii')
        cartGlobal.log("Send rotate clockwise" + str(msg))
        ser.write(msg) 

    cartGlobal.setMovementBlocked(False)
    cartGlobal.setCartRotating(True)
    cartGlobal.setCartMoving(True)


def sendStopCommand():

    msg = b'4' + b'\n'
    cartGlobal.log("Send stop " + str(msg))
    ser.write(msg)
    cartGlobal.setCartMoving(False)


def getCartOrientation():

    msg = b'5' + b'\n'
    #cartGlobal.log("get Orientation " + msg)
    ser.write(msg)


def sendReadSensorCommand(sensorID):

    msg = b'7' + bytes(str(sensorID),'ascii') + b'\n'
    cartGlobal.log("Send readSensor " + str(msg))
    ser.write(msg) 

def sendHeartbeat():

    msg = b'9' + b'\n'
    ser.write(msg)

