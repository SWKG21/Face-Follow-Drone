#!/usr/bin/env python
from __future__ import print_function

#import roslib
#roslib.load_manifest('my_package')
import sys
import rospy
import cv2
import time
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import threading

class face_det:

  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_out",Image, queue_size=1)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("image_topic_in",Image,self.callback)
    #self.lock = threading.RLock()
    # Create the haar cascade
    cascPath = "/home/turtlebot/catkin_ws/src/tp_ros_ensta_u2is/scripts/haarcascade_frontalface_default.xml"
    self.faceCascade = cv2.CascadeClassifier(cascPath)


  # Callback when a new image arrived
  def callback(self,data):
    #self.lock.acquire()
    # Read the image
    
    try:
      image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    # Detect faces in the image
    self.faces = self.faceCascade.detectMultiScale(
      gray,
      scaleFactor=1.1,
      minNeighbors=5,
      minSize=(120, 110)
      #flags = cv2.CV_HAAR_SCALE_IMAGE
    )
    

    #print("Found {0} faces!".format(len(faces)))

    # Draw a rectangle around the faces
    for (x, y, w, h) in self.faces:
      cv2.rectangle(image, (x, y), (x+w, y+h), (0, 255, 0), 2)

    cv2.imshow("Faces found", image)
    cv2.waitKey(1)
    '''try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)'''
    #self.lock.release()

def main(args):
  print("OpenCV Version: {}".format(cv2.__version__))
  print("Python Version: {}".format(sys.version))
  ic = face_det()
  rospy.init_node('fd', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

