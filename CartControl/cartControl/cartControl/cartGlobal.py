
import time
import sys
import io
import numpy as np
import psutil
import cv2

##################################################################
# cartControl can run as slave of navManager or in standalone mode
##################################################################
standAloneMode = False   # False -> slave mode,  True -> standalone mode


# configuration values for cart arduino infrared distance limits
SHORT_RANGE_MIN=6
SHORT_RANGE_MAX=18
LONG_RANGE_MIN=12
LONG_RANGE_MAX=34
delayBetweenDistanceMeasurements=1     # can probably be lower when battery fully loaded

# values for arduino to calculate distance mm/s from speed value
# (values depend on battery level too, see excel sheet for calc)
SPEED_FACTOR=1.49
SPEED_OFFSET=44.6
SPEED_FACTOR_SIDEWAY=0.5
SPEED_FACTOR_DIAGONAL=0.63


# odometry is only active when cart is moving
_cartMoving = False
_cartRotating = False
_requestedCartSpeed = 0
_cartSpeedFromOdometry = 0

_movementBlocked = False
_timeMovementBlocked = None
_moveDistanceRequested = 0
_moveStartX = 0
_moveStartY = 0
_cartOrientation = 0
_targetOrientation = 0
_moveStartTime = None
_odometryRunning = False
_lastCartCommand = ""
_orientationBeforeMove = 0

# Current cart position (center of cart) relative to map center, values in mm
_cartPositionX = 0
_cartPositionY = 0

_lastDistance = 0
_lastDxPixPerSec = 0
_lastDyPixPerSec = 0
#_lastTrackedMoveDuration = 0
#_lastGoodMoveTimestamp = time.time()
_lastDistEvalTimestamp = time.time()

_lastBatteryCheckTime = time.time()
_batteryStatus = None
arduinoStatus = 0
_mVolts12V = 0

# pixel-width in mm at 13 cm distance from ground
#PIX_PER_MM = 0.7

navManager = None

taskStarted = time.time()
_f = None
#def startlog():
#    logging.basicConfig(filename="cartControl.log", level=logging.INFO, format='%(asctime)s - %(name)s - %(message)s', filemode="w")

def log(msg):

    print(f"time: {time.time()-taskStarted:.3f} "  + msg)
    
    if navManager is not None:
        navManager.root.recordLog("cart - " + msg)


def saveImg(img, frameNr):

    try:
        cv2.imwrite(f"C:/cartControl/floorImages/floor_{frameNr}.jpg", img)
    except:
        log(f"cartGlobal, saveImg exception {sys.exc_info()[0]}")


def setCartMoveDistance(distanceMm):

    global _moveDistanceRequested, _moveStartX, _moveStartY, _lastDistance, _moveStartTime

    _moveDistanceRequested = distanceMm
    _moveStartX = _cartPositionX
    _moveStartY = _cartPositionY
    _lastDistance = 0
    _moveStartTime = time.time()
    

def getMoveDistance():
    return _moveDistanceRequested


def currMoveDistance():

    dx = np.abs(_cartPositionX - _moveStartX)
    dy = np.abs(_cartPositionY - _moveStartY)

    return int(np.hypot(dx, dy))


def checkMoveDistanceReached():

    #log(f"moveDistance requested {_moveDistanceRequested}, moveDistance current {int(currMoveDistance())}, moveTime: {time.time() - _moveStartTime:.2f}")

    return currMoveDistance() >= _moveDistanceRequested


def getRemainingDistance():
    return _moveDistanceRequested - currMoveDistance()


def setCartMoving(new):

    global _cartMoving #, _lastTrackedMoveDuration

    #log(f"setCartMoving {new}")
    _cartMoving = new

    if isCartRotating():
        setCartRotating(False)

    
def isCartMoving():

    return _cartMoving


def setCartRotating(new):

    global _cartRotating

    #log(f"setCartRotating {new}")
    _cartRotating = new
    if isCartMoving():
        setCartMoving(False)


def setTargetOrientation(relAngle): #CartRotating(new):

    global _targetOrientation

    _targetOrientation = (_cartOrientation + relAngle) % 360
    log(f"setTargetOrientation -> currOrientation: {_cartOrientation}, relAngle: {relAngle}, targetOrientation: {_targetOrientation}")
    setCartRotating(True)

    
def isCartRotating():
    return _cartRotating


def setMovementBlocked(new):

    global _movementBlocked, _timeMovementBlocked

    _movementBlocked = new
    _timeMovementBlocked = time.time()


def getMovementBlocked():
    return _movementBlocked, _timeMovementBlocked


def setRequestedCartSpeed(speed):
    
    global _requestedCartSpeed

    _requestedCartSpeed = speed


def getRequestedCartSpeed():

    return _requestedCartSpeed


def setCartSpeedFromOdometry(speed):
    
    global _cartSpeedFromOdometry

    _cartSpeedFromOdometry = speed


def getCartSpeedFromOdometry():
    return _cartSpeedFromOdometry


def setOrientation(new):

    global _cartOrientation

    _cartOrientation = round(new)

    
def getOrientation():
    return _cartOrientation


def setCartPosition(newX, newY):

    global _cartPositionX, _cartPositionY

    _cartPositionX = newX
    _cartPositionY = newY


def getCartPosition():
    return _cartPositionX, _cartPositionY


def getMoveStart():
    return _moveStartX, _moveStartY    


def updateCartPositionInMm(dxPix, dyPix, duration):     # distance is pixels, cart position is mm

    global _lastDistance

    #log(f"updateCartPosition based on floor images, dx: {dx}, dy: {dy}")
    pixPerSec = np.hypot(dxPix, dyPix) / duration

    posX, posY = getCartPosition()
    posX += int(dxPix * PIX_PER_MM)
    posY += int(dyPix * PIX_PER_MM)
    setCartPosition(posX, posY)

    startX, startY = getMoveStart()
    dxMoved = np.abs(posX - startX)
    dyMoved = np.abs(posY - startY)
    currDistance = np.hypot(dxMoved, dyMoved)

    log(f"cartMovement[mm] total distance: {currDistance:.0f}, total time: {time.time() - _moveStartTime:.2f}, curr speed Pix/s: {pixPerSec:.2f}")
 #       _lastDistance = currDistance



def setOdometryRunning(newStatus):

    global _odometryRunning

    _odometryRunning = newStatus


def isOdometryRunning():
    return _odometryRunning


def getRemainingRotation():
    d = abs(_cartOrientation - _targetOrientation) % 360
    return 360 - d if d > 180 else d


def setDistEvalTimestamp():

    global _lastDistEvalTimestamp

    _lastDistEvalTimestamp = time.time()


def getLastBatteryCheckTime():
    return _lastBatteryCheckTime

def setLastBatteryCheckTime(newTime):

    global _lastBatteryCheckTime

    _lastBatteryCheckTime = newTime


def getBatteryStatus():
    return _batteryStatus


def updateBatteryStatus():

    global _batteryStatus

    _batteryStatus = psutil.sensors_battery()
    setLastBatteryCheckTime(time.time())

    # inform navManager about low battery
    if _batteryStatus.percent < 25:
        msg = f"low battery: {_batteryStatus.percent:.0f} percent"
        if standAloneMode or navManager is None:
            log(msg)
        else:
            navManager.root.lowBattery("cart - " + msg)


def getSensorName(sensorID):
    return ["", "VL nah","VL fern","VR nah","VR fern","left","right","HL nah","HL fern","HR nah","HL fern"][sensorID+1]


def getVoltage12V():
    return _mVolts12V


def setVoltage12V(value):

    global _mVolts12V

    _mVolts12V = value
