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

class face_det:

  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_out",Image, queue_size=1)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("image_topic_in",Image,self.callback)
    self.faceCascade = cv2.CascadeClassifier("haarcascade_frontalface_default.xml")


  # Callback when a new image arrived
  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    # Our operations on the frame come here
    image = cv2.imread(cv_image)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    # Detect faces
    faces = self.faceCascade.detectMultiScale(
      gray,
      scaleFactor=1.1,
      minNeighbors=5,
      minSize=(30, 30),
      flags = cv2.cv.CV_HAAR_SCALE_IMAGE
    )
    # Draw a rectangle around the faces
    for (x, y, w, h) in faces:
      cv2.rectangle(image, (x, y), (x+100, y+100), (0, 255, 0), 2)
    
    cv2.imshow("Faces found", image)
    cv.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)

def main(args):
  print("OpenCV Version: {}".format(cv2.__version__))
  print("Python Version: {}".format(sys.version))
  ic = face_det()
  rospy.init_node('face_det', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
