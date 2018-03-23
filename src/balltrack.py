#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Int16MultiArray
from sensor_msgs.msg import Image
import image_geometry



def imagecb(data):
    # Convert Image message to CV image with blue-green-red color order (bgr8)
    try:
        bridge = CvBridge()
        img_CV = bridge.imgmsg_to_cv2(data, "bgr8")
        img_hsv = cv2.cvtColor(img_CV, cv2.COLOR_BGR2HSV)
    except CvBridgeError, e:
        print("==[CAMERA MANAGER]==", e)
    # cv2.imshow("Cam", img_CV)
    # cv2.waitKey(3)
    return img_hsv

def ball_track(image):
    toServos = rospy.Publisher('center_error', Int16MultiArray, queue_size = 10)
    cv2.namedWindow("Object Tracker", cv2.WINDOW_NORMAL)
    # cv2.createTrackbar('H low','Object Tracker',0,255, nothing)
    # cv2.createTrackbar('S low','Object Tracker',0,255, nothing)
    # cv2.createTrackbar('V low','Object Tracker',0,255, nothing)
    # cv2.createTrackbar('H high','Object Tracker',0,255, nothing)
    # cv2.createTrackbar('S high','Object Tracker',0,255, nothing)
    # cv2.createTrackbar('V high','Object Tracker',0,255, nothing)
    #
    # hLow = cv2.getTrackbarPos('H low', 'Object Tracker')
    # sLow = cv2.getTrackbarPos('S low', 'Object Tracker')
    # vLow = cv2.getTrackbarPos('V low', 'Object Tracker')
    # hHigh = cv2.getTrackbarPos('H high', 'Object Tracker')
    # sHigh = cv2.getTrackbarPos('S high', 'Object Tracker')
    # vHigh = cv2.getTrackbarPos('V high', 'Object Tracker')

    mv_image = imagecb(image)
    greenLower = (0, 0, 162)
    greenUpper = (0, 0, 255)
    mask = cv2.inRange(mv_image, greenLower, greenUpper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    cam_model = image_geometry.PinholeCameraModel()

    circcont = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    if (len(circcont) > 0):
        c = max(circcont, key=cv2.contourArea)
        ((x,y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        try:
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        except ZeroDivisionError:
            pass

        cv2.circle(mask, (int(x), int(y)), int(radius), (255,0,0), 2)
        cent_err = (x-375, y-240)
        err_msg = Int16MultiArray()
        err_msg.data = cent_err
        toServos.publish(err_msg)
        print cent_err
        # print radius
        # print x, y
        # print vect




    cv2.imshow('Object Tracker', mask)
    cv2.waitKey(1)

    #
    # # mask = cv2.inRange(hsv, greenLower, greenUpper)
    # mask = cv2.erode(mask, None, iterations=2)
    # mask = cv2.dilate(mask, None, iterations=2)







def nothing():
    pass





def viewcam():
    rospy.init_node('cam_mask', anonymous=True)
    rospy.Subscriber('/mv_25001306/image_raw', Image, ball_track)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down...")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    viewcam()
