#!/usr/bin/env python
from __future__ import print_function

#import roslib
#roslib.load_manifest('my_package')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from distutils.version import LooseVersion

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_out",Image, queue_size=1)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("image_topic_in",Image,self.callback)

  # Callback when a new image arrived
  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    # Our operations on the frame come here
    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)


    if LooseVersion(cv2.__version__).version[0] == 2:
      # OpenCV2 code
      sift = cv2.FeatureDetector_create("SIFT")
    else:
      # OpenCV3 code
      sift = cv2.xfeatures2d.SIFT_create()

    kp = sift.detect(gray,None)
    
    imgSift = cv_image
    cv2.drawKeypoints(cv_image, kp, imgSift, flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

    # Display
    cv2.imshow('sift detection',imgSift)
    cv2.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)

def main(args):
  print("OpenCV Version: {}".format(cv2.__version__))
  print("Python Version: {}".format(sys.version))
  ic = image_converter()
  rospy.init_node('image_converter_sift', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
