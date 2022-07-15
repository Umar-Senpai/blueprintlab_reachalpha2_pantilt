# import the necessary packages
from imutils import contours
from skimage import measure
import numpy as np
import argparse
import imutils
import cv2
from PIL import Image, ImageDraw, ImageFont

def track_light(image_array, x_center, y_center, prevImage = None):
    frame = image_array
    
    # if prevImage is not None:
    #     frame = cv2.absdiff(image_array, prevImage)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # blurred = cv2.GaussianBlur(gray, (11, 11), 0)
    blurred = gray
    height, width, channels = frame.shape


    # threshold the image to reveal light regions in the
    # blurred image
    thresh = cv2.threshold(gray, 240, 255, cv2.THRESH_BINARY)[1]

    # perform a series of erosions and dilations to remove
    # any small blobs of noise from the thresholded image
    thresh = cv2.erode(thresh, None, iterations=2)
    thresh = cv2.dilate(thresh, None, iterations=4)
    # perform a connected component analysis on the thresholded
    # image, then initialize a mask to store only the "large"
    # components
    labels = measure.label(thresh, background=0)
    # hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # ### RED
    # # lower_range = np.array([10,50,50])
    # # upper_range = np.array([10,255,255])
    # ### BLUE
    # lower_range = np.array([110,50,50])
    # upper_range = np.array([130,255,255])

    # mask = cv2.inRange(hsv, lower_range, upper_range)
    # frame = cv2.bitwise_and(frame,frame, mask= mask)
    # numPixels = cv2.countNonZero(mask)
    mask = np.zeros(thresh.shape, dtype="uint8")

    # loop over the unique components
    for label in np.unique(labels):
        # if this is the background label, ignore it
        if label == 0:
            continue

        # otherwise, construct the label mask and count the
        # number of pixels
        labelMask = np.zeros(thresh.shape, dtype="uint8")
        labelMask[labels == label] = 255
        numPixels = cv2.countNonZero(labelMask)
        # print("NUM PIXELS", numPixels)

        # if the number of pixels in the component is sufficiently
        # large, then add it to our mask of "large blobs"
        if numPixels < 300:
            mask = cv2.add(mask, labelMask)

    # find the contours in the mask, then sort them from left to
    # right
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                            cv2.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0]
    # print('cnts', cnts)
    flag = False
    if len(cnts) > 0:
        cnts = contours.sort_contours(cnts)[0]

    # print(len(cnts))
    cXArray = []
    cYArray = []
    if len(cnts) > 1:
        flag = True
    # loop over the contours
    for c in cnts:
        (x, y, w, h) = cv2.boundingRect(c)
        ((cX, cY), radius) = cv2.minEnclosingCircle(c)
        # if radius < 3.5 or radius > 8.0:
        #     continue
        # print('radius', radius)
        cXArray.append(cX)
        cYArray.append(cY)
        cv2.circle(frame, (int(cX), int(cY)), int(radius),
                   (0, 255, 0), 2)
        # for i in range(len(cnts)):
        #     cv2.putText(frame, "Light-{}".format(i + 1), (x, y - 15),
        #                 cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)
    # for (i, c) in enumerate(cnts):
    #     # draw the bright spot on the image
    #     (x, y, w, h) = cv2.boundingRect(c)
    #     ((cX, cY), radius) = cv2.minEnclosingCircle(c)
    #     cv2.circle(frame, (int(cX), int(cY)), int(radius),
    #                (0, 255, 0), 2)
    #     cv2.putText(frame, "Light-{}".format(i + 1), (x, y - 15),
    #                 cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)

    # show the output image
    cv2.circle(frame, (int(width/2), int(height/2)), int(5), (0, 0, 255), 3)
    # print(width/2, height/2)
    cv2.putText(frame, "CENTER", (int(width/2), int(height/2) - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 2)
    if flag:
        print(cXArray)
        
        meanX = np.mean(cXArray)
        meanY = np.mean(cYArray)
        x_center = np.mean([x for x in cXArray if abs(x - meanX) < 200])
        y_center = np.mean([x for x in cYArray if abs(x - meanY) < 200])

    ### FOR TESTING ###
    # x_center = 400
    # y_center = 400
    ### FOR TESTING ###

    cv2.circle(frame, (int(x_center), int(y_center)), int(5), (0, 0, 255), 3)
    cv2.putText(frame, "MODEM CENTER", (int(x_center), int(y_center) - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 2)
    # print(x_center, y_center)

    # show the output image
    return frame, x_center, y_center