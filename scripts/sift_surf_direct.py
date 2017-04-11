#!/usr/bin/env python
from __future__ import print_function

import numpy as np
import cv2
import sys
from distutils.version import LooseVersion

def main(args):

  print("OpenCV Version: {}".format(cv2.__version__))
  print("Python Version: {}".format(sys.version))

  cap = cv2.VideoCapture(0)

  while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()

    # Our operations on the frame come here
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # ****** SIFT
    if LooseVersion(cv2.__version__).version[0] == 2:
      # OpenCV 2.4.12 code
      sift = cv2.FeatureDetector_create("SIFT")
    else:
      # OpenCV 3.1 code
      sift = cv2.xfeatures2d.SIFT_create()

    kp = sift.detect(gray,None)
    
    imgOutSift = frame
    cv2.drawKeypoints(frame,kp, imgOutSift, flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

    # Display the resulting frame
    cv2.imshow('sift detection',imgOutSift)

    # ****** SURF 
    if LooseVersion(cv2.__version__).version[0] == 2:
      # OpenCV 2.4.12 code
      surf = cv2.FeatureDetector_create("SURF")
    else:
      # OpenCV 3.1 code
      surf = cv2.xfeatures2d.SURF_create()

    kp = surf.detect(gray,None)
    
    imgOutSurf = frame
    cv2.drawKeypoints(frame,kp, imgOutSurf, flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

    # Display the resulting frame
    cv2.imshow('surf detection',imgOutSurf)

    if cv2.waitKey(1) & 0xFF == ord('q'):
      break

  # When everything done, release the capture
  cap.release()
  cv2.destroyAllWindows()

if __name__ == '__main__':
  main(sys.argv)
