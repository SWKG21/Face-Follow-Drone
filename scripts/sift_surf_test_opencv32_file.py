#!/usr/bin/env python

import numpy as np
import cv2
import rospy
import rospkg

rospy.init_node('sift_surf_test_opencv32_file', anonymous=True)

print("OpenCV Version: {}".format(cv2.__version__))

# get an instance of RosPack with the default search paths
rospack = rospkg.RosPack()
path = rospack.get_path('tp_ros_ensta_u2is')
pathFile = path + '/img/' + 'box_in_scene.png'
print ("we process this file : %s" % pathFile)

while(True):

    imgIn = cv2.imread(pathFile)

    # Our operations on the imgIn come here
    gray = cv2.cvtColor(imgIn, cv2.COLOR_BGR2GRAY)
    #gray = imgIn
    cv2.imshow('image',imgIn)

    sift = cv2.xfeatures2d.SIFT_create()
    kp = sift.detect(gray,None)
    
    imgOutSift = imgIn
    cv2.drawKeypoints(imgIn,kp, imgOutSift, flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

    # Display the resulting imgIn
    cv2.imshow('sift detection',imgOutSift)

    surf = cv2.xfeatures2d.SURF_create()
    kp = surf.detect(gray,None)
    
    imgOutSurf = imgIn
    cv2.drawKeypoints(imgIn,kp, imgOutSurf, flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

    # Display the result
    cv2.imshow('surf detection',imgOutSurf)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cv2.destroyAllWindows()
