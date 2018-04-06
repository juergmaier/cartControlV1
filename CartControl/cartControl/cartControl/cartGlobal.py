
import time
import sys
import numpy as np
import cv2

# odometry is only active when cart is moving
_cartMoving = False
_cartRotating = False
_movementBlocked = False
_timeMovementBlocked = None
_moveDistance = 0
_moveStartX = 0
_moveStartY = 0
_orientation = 0
_moveStartTime = None

# Values in mm
currPosX = 0
currPosY = 0

lastDistance = 0
arduinoStatus = 0

# pixel-width in mm at 13 cm distance from ground
pix2mmX = 0.7
# pixel-height at 13 cm distance from ground
pix2mmY = 0.7

frameNr = 0

navManager = None

simulateLogger = True

#def startlog():
#    logging.basicConfig(filename="cartControl.log", level=logging.INFO, format='%(asctime)s - %(name)s - %(message)s', filemode="w")

def log(msg):
    if not simulateLogger:
        navManager.root.recordLog("cart - " + msg)
    print("cart - " + msg)

def saveImg(img):

    global frameNr

    try:
        cv2.imwrite(f"C:/cartControl/floorImages/floor_{frameNr}.jpg", img)
    except:
        log(f"cartGlobal, saveImg exception {sys.exc_info()[0]}")
    frameNr += 1


def setCartMoveDistance(distanceMm):

    global _moveDistance, _moveStartX, _moveStartY, lastDistance, _moveStartTime

    _moveDistance = distanceMm
    _moveStartX = currPosX
    _moveStartY = currPosY
    lastDistance = 0
    _moveStartTime = time.time()
    

def checkMoveDistanceReached():

    dx = np.abs(currPosX - _moveStartX)
    dy = np.abs(currPosY - _moveStartY)

    dist = np.hypot(dx, dy)
    #log(f"moveDistance requested {_moveDistance}, moveDistance current {int(dist)}, moveTime: {time.time() - _moveStartTime:.2f}")

    return dist >= _moveDistance


def setCartMoving(new):

    global _cartMoving

    log(f"setCartMoving {new}")
    _cartMoving = new
    if isCartRotating():
        setCartRotating(False)

    
def isCartMoving():

    return _cartMoving


def setCartRotating(new):

    global _cartRotating

    log(f"setCartRotating {new}")
    _cartRotating = new
    if isCartMoving():
        setCartMoving(False)

    
def isCartRotating():

    return _cartRotating


def setMovementBlocked(new):

    global _movementBlocked, _timeMovementBlocked

    _movementBlocked = new
    _timeMovementBlocked = time.time()


def getMovementBlocked():
    return _movementBlocked, _timeMovementBlocked



def setOrientation(new):

    global _orientation

    _orientation = round(new)

    
def getOrientation():

    return _orientation


def updateCartPosition(dx, dy):

    global currPosX, currPosY, lastDistance

    currPosX += int(dx)
    currPosY += int(dy)

    dxMoved = np.abs(currPosX - _moveStartX)
    dyMoved = np.abs(currPosY - _moveStartY)
    currDistance = np.hypot(dxMoved, dyMoved)

    if currDistance > lastDistance+20:
        log(f"dx: {dx:5.0f} dy: {dy:5.0f}  currPosX {currPosX:5.0f}  currPosY {currPosY:5.0f}, moveTime: {time.time() - _moveStartTime:.2f}")
        lastDistance = currDistance



