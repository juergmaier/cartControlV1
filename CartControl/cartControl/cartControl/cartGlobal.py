
import time
import sys
import numpy as np
import cv2


##################################################################
# cartControl can run as slave of navManager or in standalone mode
##################################################################
standAloneMode = False   # False -> slave mode,  True -> standalone mode

# configuration values for cart arduino infrared distance limits
MIN_RANGE_SHORT=9
MAX_RANGE_SHORT=16
MIN_RANGE_LONG=16
MAX_RANGE_LONG=30

# odometry is only active when cart is moving
_cartMoving = False
_cartRotating = False
_requestedCartSpeed = 0
_cartSpeedFromOdometry = 0

_movementBlocked = False
_timeMovementBlocked = None
_moveDistance = 0
_moveStartX = 0
_moveStartY = 0
_cartOrientation = 0
_targetOrientation = 0
_moveStartTime = None
_odometryRunning = False

# Current cart position (center of cart) relative to map center, values in mm
_cartPositionX = 0
_cartPositionY = 0

_lastDistance = 0
_lastDxPixPerSec = 0
_lastDyPixPerSec = 0
#_lastTrackedMoveDuration = 0
#_lastGoodMoveTimestamp = time.time()
_lastDistEvalTimestamp = time.time()

arduinoStatus = 0

# pixel-width in mm at 13 cm distance from ground
PIX_PER_MM = 0.47

navManager = None

taskStarted = time.time()
#def startlog():
#    logging.basicConfig(filename="cartControl.log", level=logging.INFO, format='%(asctime)s - %(name)s - %(message)s', filemode="w")

def log(msg):
    if standAloneMode or navManager is None:
        print(f"time: {time.time()-taskStarted:.3f} "  + msg)
    else:
        navManager.root.recordLog("cart - " + msg)


def saveImg(img, frameNr):

    try:
        cv2.imwrite(f"C:/cartControl/floorImages/floor_{frameNr}.jpg", img)
    except:
        log(f"cartGlobal, saveImg exception {sys.exc_info()[0]}")


def setCartMoveDistance(distanceMm):

    global _moveDistance, _moveStartX, _moveStartY, _lastDistance, _moveStartTime

    _moveDistance = distanceMm
    _moveStartX = _cartPositionX
    _moveStartY = _cartPositionY
    _lastDistance = 0
    _moveStartTime = time.time()
    

def getMoveDistance():
    return _moveDistance


def currMoveDistance():

    dx = np.abs(_cartPositionX - _moveStartX)
    dy = np.abs(_cartPositionY - _moveStartY)

    return int(np.hypot(dx, dy))


def checkMoveDistanceReached():

    #log(f"moveDistance requested {_moveDistance}, moveDistance current {int(currMoveDistance())}, moveTime: {time.time() - _moveStartTime:.2f}")

    return currMoveDistance() >= _moveDistance


def getRemainingDistance():
    return _moveDistance - currMoveDistance()


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


def updateCartPositionInMm(dx, dy):     # here we use mm not pixels!

    global _lastDistance

    #log(f"updateCartPosition based on floor images, dx: {dx}, dy: {dy}")

    posX, posY = getCartPosition()
    posX += int(dx * PIX_PER_MM)
    posY += int(dy * PIX_PER_MM)
    setCartPosition(posX, posY)

    startX, startY = getMoveStart()
    dxMoved = np.abs(posX - startX)
    dyMoved = np.abs(posY - startY)
    currDistance = np.hypot(dxMoved, dyMoved)

#    if currDistance > _lastDistance+20:
    log(f"cartMovement[mm] total distance: {currDistance:.0f}, total time: {time.time() - _moveStartTime:.2f}")
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
