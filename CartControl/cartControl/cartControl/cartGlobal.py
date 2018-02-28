
import numpy as np
import logging
import time

# odometry is only active when cart is moving
_cartMoving = False
_cartRotating = False
_movementBlocked = False
_timeMovementBlocked = None
_moveDistance = 0
_moveStartX = 0
_moveStartY = 0
_orientation = 0

currPosXPix = 0
currPosYPix = 0
currPosX = 0
currPosY = 0
lastDistance = 0
arduinoStatus = 0

# pixel-width in mm at 13 cm distance from ground
pixX = 85 / 320
# pixel-height at 13 cm distance from ground
pixY = 65 / 240


def startlog():
    logging.basicConfig(filename="cartControl.log", level=logging.INFO, format='%(asctime)s - %(name)s - %(message)s', filemode="w")

def log(msg):
    print(msg)
    logging.info(msg)


def setCartMoveDistance(dist):

    global _moveDistance, _moveStartX, _moveStartY, lastDistance

    _moveDistance = dist
    _moveStartX = currPosX
    _moveStartY = currPosY
    lastDistance = 0
    

def checkMoveDistanceReached():

    dx = np.abs(currPosX - _moveStartX)
    dy = np.abs(currPosY - _moveStartY)

    return np.sqrt(np.power(dx, 2) + np.power(dy,2)) >= _moveDistance


def setCartMoving(new):

    global _cartMoving

    log(f"setCartMoving {new}")
    _cartMoving = new

    
def getCartMoving():

    return _cartMoving


def setCartRotating(new):

    global _cartRotating

    log(f"setCartRotating {new}")
    _cartRotating = new

    
def getCartRotating():

    return _cartRotating


def setMovementBlocked(new):

    global _movementBlocked, _timeMovementBlocked

    _movementBlocked = new
    _timeMovementBlocked = time.time()


def getMovementBlocked():
    return _movementBlocked, _timeMovementBlocked



def setOrientation(new):

    global _orientation

    _orientation = new

    
def getOrientation():

    return _orientation


def updateCartPosition(dx, dy):

    global currPosX, currPosY, currPosXPix, currPosYPix, lastDistance

    currPosXPix += dx
    currPosYPix += dy
    currPosX += dx * pixX
    currPosY += dy * pixY

    dx = np.abs(currPosX - _moveStartX)
    dy = np.abs(currPosY - _moveStartY)
    currDistance = np.sqrt(np.power(dx, 2) + np.power(dy, 2))
    if currDistance > lastDistance+10:
        log(f"dx {dx:5.0f} dy {dy:5.0f}  currPosXmm {currPosX:5.0f}  currPosYmm {currPosY:5.0f}")
        lastDistance = currDistance



