
'''
cam odometry - part of the cart control app

when cart is moving use ps3 cam pics to track movement
using ORB
'''

import time, glob, os
import numpy as np
import cv2

import cartGlobal, arduino

MIN_CANDIDATES = 8
MIN_VECTOR_LENGTH = 2

camConnected = False
cap = None
orb = None
bf = None
matchFailures = 0

_lastDxPixPerSec = 0
_lastDyPixPerSec = 0
_lastCheckDecelerate = time.time()

def checkDecelerate():

    global _lastCheckDecelerate

    _lastCheckDecelerate = time.time()

    if cartGlobal.isCartMoving():

        cartSpeed = cartGlobal.getRequestedCartSpeed()
        remainingDist = cartGlobal.getRemainingDistance()
        #cartGlobal.log(f"check decelerate move, remainingDist: {int(remainingDist)}, currSpeed: {cartSpeed}")
        if cartSpeed * remainingDist / 80 < cartSpeed:
            cartSpeed -= 30
            cartGlobal.log(f"decelerate, remainingDist: {remainingDist:.0f}, new speed: {cartSpeed:.0f}")
            arduino.sendSpeedCommand(cartSpeed)    


    if cartGlobal.isCartRotating():

        #cartGlobal.log(f"check decelerate rotation, restRot: {restRot}, currSpeed: {cartSpeed}")
        if cartSpeed * cartGlobal.getRemainingRotation() / 5 < cartGlobal.getRequestedCartSpeed():
            cartSpeed -= 20
            arduino.sendSpeedCommand(cartSpeed)    


def reject_outliers(data, m = 2.):
    d = np.abs(data - np.median(data))
    mdev = np.median(d)
    s = d/mdev if mdev else 0.
    return data[s<m]


def removeMinMax(nparr):
    minMax = [np.argmin(nparr), np.argmax(nparr)]   # indices of min/max
    return np.delete(nparr,minMax)


def similarAngleDistance(src, dst):
    '''
    from the matching comparison points of the previous and current images eval std of angle and length
    and return true if first 10 vectors are similar
    '''
    numCandidate = 0
    angle = np.zeros(MIN_CANDIDATES)
    length = np.zeros(MIN_CANDIDATES)

    for i in range(len(src)):

        diffY = src[i][0][1] - dst[i][0][1]
        diffX = src[i][0][0] - dst[i][0][0]
        angle[numCandidate] = int(np.degrees(np.arctan2(diffY, diffX)))
        length[numCandidate] = int(np.hypot(diffY, diffX))
        numCandidate += 1
        if numCandidate >= MIN_CANDIDATES:      # check only the first x items
            break

    # remove min and max value
    angleReduced = removeMinMax(angle)
    lengthReduced = removeMinMax(length)

    # compare angles only for vectors with some distance
    if np.average(lengthReduced) < MIN_VECTOR_LENGTH:
        angleReduced[:] = 0
        #cartGlobal.log("very small movement, ignore angles")

    #cartGlobal.log(f"angle:  {angle}, avg: {sum(angle)/numCandidate:.0f}, std: {np.std(angle):.2f}")
    #cartGlobal.log(f"angleReduced:  {angleReduced}, avg: {sum(angleReduced)/(numCandidate-2):.0f}, std: {np.std(angleReduced):.2f}")
    #cartGlobal.log(f"length: {length}, avg: {sum(length)/numCandidate:.0f}, std: {np.std(length):.2f}")

    good = numCandidate >= MIN_CANDIDATES and np.std(angleReduced) < 5 and np.std(lengthReduced) < 10
    #if not good:
    #    print(f"insufficient results in frame comparison, std angle: {np.std(angle):.2f}, std length: {np.std(length):.2f}")

    return good
        

def estimateUntrackedMoveInPix(untrackedMoveTime):

    calculatedDxPix = _lastDxPixPerSec * untrackedMoveTime
    calculatedDyPix = _lastDyPixPerSec * untrackedMoveTime
    #log(f"calculated dx,dy, _lastDxPixPerSec: {_lastDxPixPerSec:.3f}, _lastDyPixPerSec: {_lastDyPixPerSec:.3f}, _lastTrackedMoveDuration: {_lastTrackedMoveDuration:.3f}")
    distancePix = np.hypot(calculatedDxPix, calculatedDyPix)
    speed = distancePix / untrackedMoveTime
    cartGlobal.log(f"untrackedMove duration: {untrackedMoveTime:.4f}, distancePix: {distancePix:.2f}, speed [pix/s]: {speed:.2f}")
    return calculatedDxPix, calculatedDyPix


def doOdometry():

    global matchFailures, _lastDxPixPerSec, _lastDyPixPerSec

    saveFloorImages = True

    goodFrames = 0
    badFrames = 0
    frameNr = 0
    failures = np.zeros(10)

    cartGlobal.log("cart starts moving")

    # check for working cam and take a start image
    while True:

        # take a start image
        img1Timestamp = time.time()
        img2Timestamp = time.time()

        for _ in range(2):      # first frame is outdated
            ret, img2 = cap.read()

        if ret:
            if saveFloorImages:
                cartGlobal.saveImg(img2, frameNr)

            # use bw image for comparison
            img2bw = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)

            # find relevant points in image
            kps2, des2 = orb.detectAndCompute(img2bw, None)

            _lastDxPixPerSec = 0
            _lastDyPixPerSec = 0

            break

        else:
            cartGlobal.log("cam problems, no start frame")


    frameSkipped = False

    # while cart is moving
    # take a ground pic and compare it with the previous one
    # find distance travelled in mm and update current position
    ########################################################
    while cartGlobal.isCartMoving():

        # check for problems with last frame comparison, compensate with last good move
        if frameSkipped:

            badFrames +=1

            # based on last good move and time since last eval update cart position
            duration = img2Timestamp - img1Timestamp
            dxPix, dyPix = estimateUntrackedMoveInPix(duration)

            # cartGlobal.log(f"  frame comparison failed with frame {frameNr}!, using last good dx/dy {dxPix:.3f},{dyPix:.3f}")
            cartGlobal.updateCartPositionInMm(dxPix, dyPix, duration)

        else:
            goodFrames += 1

        # check for slowing down
        checkDecelerate()

        # check for requested distance travelled
        if cartGlobal.checkMoveDistanceReached():

            arduino.sendStopCommand("distance reached")
            cartGlobal.log(f"move distance reached: target distance: {cartGlobal._moveDistanceRequested}, travelled distance: {cartGlobal.currMoveDistance()}, move time: {time.time() - cartGlobal._moveStartTime:.2f}")
            posX, posY = cartGlobal.getCartPosition()
            cartGlobal.log(f"current position {posX}/{posY}")

                 
        frameSkipped = True     # assume we have a problem detecting movement
        frameNr += 1
        cartGlobal.log("")
        cartGlobal.log(f"cart is moving, check progress, current frame: {frameNr}")
        #cartGlobal.setDistEvalTimestamp()

        # use last taken image as previous image
        img1bw = img2bw[:]
        kps1 = kps2[:]
        des1 = des2[:]
        img1Timestamp = img2Timestamp

        # get new image
        img2Timestamp = time.time()
        for _ in range(2):
            ret, img2 = cap.read()

        if ret:
            if saveFloorImages:
                cartGlobal.saveImg(img2, frameNr)
            img2bw = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)
            kps2, des2 = orb.detectAndCompute(img2bw, None)

        else:
            #cartGlobal.log("cam problems, frame skipped")
            failures[0] += 1
            continue        # retry

        try:
            matches = bf.knnMatch(des1, des2, k=2)

        except:
            #cartGlobal.log("knnMatch failure")
            failures[1] += 1
            continue

        if not matches:
            #cartGlobal.log(f"No matches from bf.knnMach!")
            failures[2] += 1
            continue

        # get good matches (g_matches) as per Lowe's ratio 
        g_match = []

        if np.shape(matches)[1] != 2:
            #cartGlobal.log(f"no valid matches pairs found, frame: {frameNr}")
            failures[3] += 1
            continue

        for m, n in matches:
            if m.distance < 0.8 * n.distance:
                g_match.append(m)

        if len(g_match) < MIN_CANDIDATES:
            #cartGlobal.log(f"not enough matches pairs found, frame: {frameNr}")
            failures[4] += 1
            continue

        else:

            # get source and target points       
            src_pts = np.float32([ kps1[m.queryIdx].pt for m in g_match ]).reshape(-1,1,2)
            dst_pts = np.float32([ kps2[m.trainIdx].pt for m in g_match ]).reshape(-1,1,2)

            #cartGlobal.log(f"src/dst points {len(src_pts)}/{len(dst_pts)}")

            #_, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
            _, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
            matchesMask = mask.ravel().tolist()


            # try to find x/y distance moved
            src = [src_pts for (src_pts, matchesMask) in zip(src_pts, matchesMask) if matchesMask > 0]
            dst = [dst_pts for (dst_pts, matchesMask) in zip(dst_pts, matchesMask) if matchesMask > 0]

            #if len(src) < MIN_CANDIDATES + 5:
            if len(src) < MIN_CANDIDATES:
                #cartGlobal.log(f"not enough matches after findHomography, frame: {frameNr}")
                failures[5] += 1
                continue

            # use only similar angle/distance values
            if not similarAngleDistance(src, dst):
                #cartGlobal.log(f"not enough similar angles/distancies found, frame: {frameNr}")
                failures[6] += 1
                continue

            #try:
            xSum, ySum = 0, 0
            numCompare = min(len(src), MIN_CANDIDATES)

            for i in range(numCompare):
                xSum += src[i][0][0]-dst[i][0][0]
                ySum += src[i][0][1]-dst[i][0][1]

            if numCompare:
                dxPix = xSum / numCompare
                dyPix = ySum / numCompare

                duration = img2Timestamp - img1Timestamp
                distance = np.hypot(dxPix, dyPix)

                _lastDxPixPerSec = dxPix / (duration)
                _lastDyPixPerSec = dyPix / (duration)

                #cartGlobal.log(f"dxPix {dxPix:.4f}, dyPix {dyPix:.4f}, duration: {img2Timestamp-img1Timestamp:.4f}, speed: {distance/duration:.4f}")

                frameSkipped = False
                cartGlobal.updateCartPositionInMm(dxPix, dyPix, duration)

            else:
                #cartGlobal.log(f"no src points")
                failures[7] += 1

        

        #cartGlobal.log(f"time used for frame comparison {time.time()-start:.4f}")

    # move done
    cartGlobal.log(f"frames: {goodFrames+badFrames}, bad frames: {badFrames}")
    if failures[0] > 0:
        cartGlobal.log(f"cam problems, frame skipped: {failures[0]:.0f}")
    if failures[1] > 0:
        cartGlobal.log(f"knnMatch failure: {failures[1]:.0f}")
    if failures[2] > 0:
        cartGlobal.log(f"No matches from bf.knnMach: {failures[2]:.0f}")
    if failures[3] > 0:
        cartGlobal.log(f"no valid matches pairs found, {failures[3]:.0f}")
    if failures[4] > 0:
        cartGlobal.log(f"not enough matches pairs found: {failures[4]:.0f}")
    if failures[5] > 0:
        cartGlobal.log(f"not enough matches after findHomography: {failures[5]:.0f}")
    if failures[6] > 0:
        cartGlobal.log(f"not enough similar angles/distancies found: {failures[6]:.0f}")
    if failures[7] > 0:
        cartGlobal.log(f"no src points: {failures[7]:.0f}")



def trackCartMovements():

    global cap, camConnected, orb, bf

    cartGlobal.log("trackCartMovements started")

    if not camConnected:

        cap = cv2.VideoCapture(1)
        cap.set(cv2.CAP_PROP_FPS, 120)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        cartGlobal.log(f"cam params fps: {cap.get(cv2.CAP_PROP_FPS)}, w: {cap.get(cv2.CAP_PROP_FRAME_WIDTH)}, h: {cap.get(cv2.CAP_PROP_FRAME_HEIGHT)}")

        time.sleep(0.5)         # allow cam to adjust to light

        # remove tracking jpg's from previous run
        for imgFile in glob.glob("C:/cartControl/floorImages/*.jpg"):
            os.remove(imgFile)

        for _ in range(2):      # first frame is black
            ret, img = cap.read()

        if ret:
            cartGlobal.log("cam connected")
            cv2.imshow("initial frame", img)
            cv2.waitKey(500)
            cartGlobal.saveImg(img, 0)
            cv2.destroyAllWindows()

            camConnected = True

            # Initiate ORB detector (fastThreshold has highest impact on number of candidate points seen)
            orb = cv2.ORB_create(nfeatures=250, edgeThreshold=8, patchSize=18, fastThreshold=6,  scoreType=cv2.ORB_FAST_SCORE)  # 2.5.2018
    
            # create BFMatcher object
            bf = cv2.BFMatcher()
    
        else:
            cartGlobal.log("no cam found")
            raise SystemExit()

    while True:

        if cartGlobal.isCartMoving():

            doOdometry()

            cartGlobal.log(f"odometry, cart stopped, moveDistance: {cartGlobal.currMoveDistance():.0f}")
