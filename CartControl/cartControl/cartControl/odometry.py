
'''
cam odometry - part of the cart control app

when cart is moving use ps3 cam pics to track movement
using ORB
'''

import numpy as np
import cv2
import time, glob, os

import cartGlobal, arduino

camConnected = False
cap = None
orb = None
bf = None
matchFailures = 0

def doOdometry():

    global matchFailures

    minMatch = 8

    # first frame for tracking
    ###########################
    if not cartGlobal.getCartMoving():

        time.sleep(0.1)

    else:

        cartGlobal.log("cart starts moving")

        for i in range(2):      # read more than 1 frame looks to work better?
            ret, img1 = cap.read()

        if ret:
            img1bw = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
            kps1, des1 = orb.detectAndCompute(img1bw, None)


        # while cart is moving
        #######################
        while cartGlobal.getCartMoving():

            start = time.time()

            #cartGlobal.log("cart is moving")

            # get 2 images
            for i in range(2):
                ret, img2 = cap.read()

            if not ret:
                cartGlobal.log("cam problems")
                continue

            img2bw = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)
            kps2, des2 = orb.detectAndCompute(img2bw, None)

            try:
                matches = bf.knnMatch(des1, des2, k=2)

            except:
                cartGlobal.log("knnMatch failure")
                continue

            if len(matches) == 0:
                cartGlobal.log(f"No matches from bf.knnMach!")
                continue

            # get good matches (g_matches) as per Lowe's ratio 
            g_match = []

            if np.shape(matches)[1] != 2:
                cartGlobal.log("no valid matches pairs")
                continue

            for m, n in matches:
                if m.distance < 0.7 * n.distance:
                    g_match.append(m)

            if len(g_match) < minMatch:

                matchFailures += 1
                cartGlobal.log(f"  Not enough matches have been found! - {len(g_match)}, {minMatch}, matchFailure {matchFailures}")
                cv2.imwrite(f"NoMatch1_{matchFailures}.jpg", img1bw)
                cv2.imwrite(f"NoMatch2_{matchFailures}.jpg", img2bw)

            else:

                # get source and target points       
                src_pts = np.float32([ kps1[m.queryIdx].pt for m in g_match ]).reshape(-1,1,2)
                dst_pts = np.float32([ kps2[m.trainIdx].pt for m in g_match ]).reshape(-1,1,2)

                M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
                matchesMask = mask.ravel().tolist()


                # try to find x/y distance moved
                src = [src_pts for (src_pts, matchesMask) in zip(src_pts, matchesMask) if matchesMask > 0]
                dst = [dst_pts for (dst_pts, matchesMask) in zip(dst_pts, matchesMask) if matchesMask > 0]

                #try:
                xSum, ySum = 0, 0
                numCompare = min(len(src), 10)
                for i in range(numCompare):
                    xSum += src[i][0][0]-dst[i][0][0]
                    ySum += src[i][0][1]-dst[i][0][1]
                dx = xSum / numCompare
                dy = ySum / numCompare
                cartGlobal.updateCartPosition(dx, dy)
                #cartGlobal.log(f"x/y in cm {time.time()-start}")
                #cartGlobal.log(f"dx {dx:.0f} dy {dy:.0f}  currPosXPix {currPosXPix:.0f}  currPosYPix {currPosYPix:.0f}  currPosXmm {currPosX:.0f}  currPosYmm {currPosY:.0f}")
                #except:
                #    cartGlobal.log("x/y offset can not be calculated")
                if not cartGlobal.getCartRotating:
                    if cartGlobal.checkMoveDistanceReached():
                        arduino.sendStopCommand()
                        cartGlobal.log(f"move distance reached: distance {cartGlobal._moveDistance}, start x/y: {cartGlobal._moveStartX}/{cartGlobal._moveStartY}, curr x/y: {cartGlobal.currPosX}/{cartGlobal.currPosY}")

            # use new image as last one
            img1bw = img2bw
            kps1 = kps2
            des1 = des2

            #cartGlobal.log(f"time used for frame comparison {time.time()-start}")

        cartGlobal.log("odometry, cartMoving == False")



def trackCartMovements():

    global cap, camConnected, orb, bf

    cartGlobal.log("trackCartMovements started")

    if not camConnected:

        cap = cv2.VideoCapture(1)
        cap.set(cv2.CAP_PROP_FPS, 80)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320); 
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240); 
        cartGlobal.log(f"cam params fps: {cap.get(cv2.CAP_PROP_FPS)}, w: {cap.get(cv2.CAP_PROP_FRAME_WIDTH)}, h: {cap.get(cv2.CAP_PROP_FRAME_HEIGHT)}")

        time.sleep(2)

        for i in range(5):      # first frame is black?
            ret, img = cap.read()

        if ret:
            cartGlobal.log("cam connected")
            #cv2.imshow("front cam", img)
            #cv2.waitKey()

            camConnected = True

            # Initiate ORB detector
            orb = cv2.ORB_create(nfeatures=250, edgeThreshold = 4, patchSize=20, fastThreshold = 7,  scoreType=cv2.ORB_FAST_SCORE)
    
            # create BFMatcher object
            bf = cv2.BFMatcher()
    
            # remove tracking failure-jpg's
            for imgFile in glob.glob("*.jpg"):
                os.remove(imgFile)

        else:
            cartGlobal.log("no cam found")
            raise SystemExit()

    while True:

        if cartGlobal.getCartMoving():

            doOdometry()
