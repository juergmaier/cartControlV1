
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
            ser.writeTimeout = 0
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

        if ser.inWaiting() > 0:
            #cartGlobal.log(f"inWaiting > 0")
            recvB = ser.readline(120)
            try:
                recv = recvB.decode()
            except:
                cartGlobal.log(f"problem with decoding cart msg '{recvB}'")

            #cartGlobal.log(f"line read {recv}")
            msgID = recvB[0:3].decode()

            #cartGlobal.log("in read message: ", msgID)

            if msgID == "!A0":     #"cart ready"
                cartGlobal.arduinoStatus = 1
                cartGlobal.log("cart ready")
                sendConfigValues()

            elif msgID == "!A1":    #"distance,":
                #!A1,<sensorID>,<ANZ_MESSUNGEN_PRO_SCAN>,[ANZ_MESSUNGEN_PRO_SCAN<value>,]
                messageItems = [int(e) if e.isdigit() else e for e in recv.split(',')]
                cartControl.updateDistances(messageItems[1:])
                #gui.controller.showDistances(messageItems[1])      test, could be responsible for delays in message read?
                

            elif msgID == "!A2":    #"obstacle:":
                #!A2,<distance>
                try:
                    distanceShort = float(recv.split(",")[1])
                    distanceLong = float(recv.split(",")[2])
                except:
                    cartGlobal.log(f"exception with message: '{recv[:-2]}', no valid distance value found")
                    distanceShort = -1
                    distanceLong = -1
                cartGlobal.setMovementBlocked(True)
                if distanceShort < distanceLong:
                    distance = distanceShort
                else:
                    distance = distanceLong
                gui.controller.updateDistanceSensorObstacle(distance)


            elif msgID == "!A3":    #"abyss:":
                #!A3,<distance>
                try:
                    distanceShort = float(recv.split(",")[1])
                    distanceLong = float(recv.split(",")[2])
                except:
                    cartGlobal.log(f"exception with message: '{recv[:-2]}', no valid distance value found")
                    distanceShort = -1
                    distanceLong = -1

                if distanceShort > distanceLong:
                    distance = distanceShort
                else:
                    distance = distanceLong

                cartGlobal.setMovementBlocked(True)
                gui.controller.updateDistanceSensorAbyss(distance)


            elif msgID == "!A5":    #"cart stopped":
                #!A3
                cartGlobal.setOrientation(round(float(recv.split(",")[1])))      # delays with A9 occurred

                cartGlobal.setMovementBlocked(False)
                cartGlobal.setCartRotating(False)
                cartGlobal.setCartMoving(False)
                cartGlobal.log("!A5 cart stopped received from Arduino")

            elif msgID == "!A9":    # orientation x:":
                #!A9,<orientation>
                try:
                    newOrientation = round(float(recv.split(",")[1]))
                except:
                    cartGlobal.log("!A9 not able to retrieve orientation {recv[:-2]}")
                    newOrientation = cartGlobal.getOrientation()

                # test for change of orientation
                if  cartGlobal.getOrientation() != newOrientation:
                    cartGlobal.setOrientation(newOrientation)
                    #cartGlobal.log(f"new cart orientation: {newOrientation}")

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
            
            #cartGlobal.log("msg processed")
        time.sleep(0.001)    # give other threads a chance



########################################################################
########################################################################
def sendConfigValues():
    msg = 'a' + str(cartGlobal.MIN_RANGE_SHORT).zfill(2) + str(cartGlobal.MAX_RANGE_SHORT).zfill(2) + str(cartGlobal.MIN_RANGE_LONG).zfill(2) + str(cartGlobal.MAX_RANGE_LONG).zfill(2)
    ser.write(bytes(msg + '\n', 'ascii'))
    #cartGlobal.log(f"{msg}")


def sendMoveCommand(direction, speed, distanceMm):
    '''
    move command to cart
    the arduino does not have an odometer and movement it limited with a max duration (ms)
    when the floor cam tracking detects requested distance travelled it sends a
    stop cart to the arduino
    the arduino also monitors movement / rotation progress and may stop the cart on its own
    '''
    # conmmand 1
    distanceMmLimited = min(distanceMm, 2500)       # limit distance for single command
    cartGlobal.setCartMoveDistance(distanceMmLimited)
    cartGlobal.setRequestedCartSpeed(speed)
    duration = ((distanceMm / speed) * 2200) + 1000
    durationLimited = min(duration, 8000)      # do not move more than 8 seconds
    moveMsg = bytes('1' + str(direction) + str(speed).zfill(3) + str(int(durationLimited)).zfill(4) + '\n', 'ascii')
    ser.write(moveMsg) 

    cartGlobal.setMovementBlocked(False)
    cartGlobal.setCartMoving(True)

    cartGlobal.log(f"Send move {moveMsg}, speed: {int(speed)}, distance: {distanceMmLimited}, duration: {durationLimited}")


def sendRotateCommand(relAngle):

    ROTATION_SPEED = 200

    cartGlobal.setMovementBlocked(False)
    cartGlobal.setTargetOrientation(relAngle)
    cartGlobal.setRequestedCartSpeed(ROTATION_SPEED)        # Arduino uses fixed rotation speed

    # command 2
    if relAngle > 0:   # rotate counterclock
        msg = bytes(str(2000+abs(relAngle))+'\n', 'ascii')
        cartGlobal.log("Send rotate counterclock " + str(msg))
        ser.write(msg) 

    # command 3
    if relAngle < 0:
        msg = bytes(str(3000+abs(relAngle))+'\n', 'ascii')
        cartGlobal.log("Send rotate clockwise" + str(msg))
        ser.write(msg) 


def sendStopCommand():

    # command 4
    msg = b'4' + b'\n'
    cartGlobal.log("Send stop "+str(msg))
    ser.write(msg)
    cartGlobal.setCartMoving(False)


def sendSpeedCommand(speed):

    # command 6
    msg = bytes(str(6000+speed)+'\n', 'ascii')
    #cartGlobal.log("Send speed "+str(msg))
    ser.write(msg)
    cartGlobal.setRequestedCartSpeed(speed)


def getCartOrientation():

    msg = b'5' + b'\n'
    #cartGlobal.log("get Orientation " + msg)
    ser.write(msg)


def sendReadSensorCommand(sensorID):

    msg = b'7' + bytes(str(sensorID), 'ascii') + b'\n'
    cartGlobal.log("Send readSensor " + str(msg))
    ser.write(msg) 

def sendHeartbeat():

    msg = b'9' + b'\n'
    ser.write(msg)

