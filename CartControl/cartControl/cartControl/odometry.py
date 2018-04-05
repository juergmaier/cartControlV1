
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
    if not cartGlobal.isCartMoving():

        time.sleep(0.1)

    else:

        cartGlobal.log("cart starts moving")

        for _ in range(2):      # first frame is black
            ret, img1 = cap.read()

        if ret:
            cartGlobal.saveImg(img1)
            img1bw = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
            kps1, des1 = orb.detectAndCompute(img1bw, None)

        else:
            cartGlobal.log("cam problems, no start frame")


        # while cart is moving
        # take a ground pic and compare it with the previous one
        # find distance travelled in mm and update current position
        ########################################################
        while cartGlobal.isCartMoving():

            start = time.time()

            #cartGlobal.log("cart is moving")

            # get 2 images
            for _ in range(2):
                ret, img2 = cap.read()

            if ret:
                cartGlobal.saveImg(img2)
                img2bw = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)
                kps2, des2 = orb.detectAndCompute(img2bw, None)

            else:
                cartGlobal.log("cam problems, frame skipped")
                continue        # retry

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

                cartGlobal.log(f"  Not enough matches have been found! - {len(g_match)}, {minMatch}, frame {cartGlobal.frameNr}")

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
                dxPix = xSum / numCompare
                dyPix = ySum / numCompare
                cartGlobal.updateCartPosition(dxPix * cartGlobal.pix2mmX, dyPix * cartGlobal.pix2mmY)

                if not cartGlobal.isCartRotating():

                    # check for requested distance travelled
                    if cartGlobal.checkMoveDistanceReached():

                        arduino.sendStopCommand()
                        cartGlobal.log(f"move distance reached: distance {cartGlobal._moveDistance}, start x/y: {cartGlobal._moveStartX}/{cartGlobal._moveStartY}, curr x/y: {cartGlobal.currPosXmm}/{cartGlobal.currPosYmm}")

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
        cap.set(cv2.CAP_PROP_FPS, 100)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320); 
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240); 
        cartGlobal.log(f"cam params fps: {cap.get(cv2.CAP_PROP_FPS)}, w: {cap.get(cv2.CAP_PROP_FRAME_WIDTH)}, h: {cap.get(cv2.CAP_PROP_FRAME_HEIGHT)}")

        time.sleep(0.5)         # allow cam to adjust to light

        for i in range(2):      # first frame is black
            ret, img = cap.read()

        if ret:
            cartGlobal.log("cam connected")
            cartGlobal.saveImg(img)

            camConnected = True

            # Initiate ORB detector (fastThreshold has highest impact on number of candidate points seen)
            orb = cv2.ORB_create(nfeatures=150, edgeThreshold=8, patchSize=15, fastThreshold=6,  scoreType=cv2.ORB_FAST_SCORE)
    
            # create BFMatcher object
            bf = cv2.BFMatcher()
    
            # remove tracking failure-jpg's
            for imgFile in glob.glob("C:/cartControl/floorImages/*.jpg"):
                os.remove(imgFile)

        else:
            cartGlobal.log("no cam found")
            raise SystemExit()

    while True:

        if cartGlobal.isCartMoving():

            doOdometry()
