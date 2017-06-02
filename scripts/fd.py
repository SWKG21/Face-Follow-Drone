#!/usr/bin/env python
from __future__ import print_function

#import roslib
#roslib.load_manifest('my_package')
import sys
import rospy
import cv2
import time
from std_msgs.msg import String, Empty
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError

class face_det:

  def __init__(self):
    # Create the haar cascade
    cascPath = "/home/turtlebot/catkin_ws/src/tp_ros_ensta_u2is/scripts/haarcascade_frontalface_default.xml"
    self.faceCascade = cv2.CascadeClassifier(cascPath)
    self.image_pub = rospy.Publisher("image_topic_out",Image, queue_size=1)
    self.pubLand = rospy.Publisher('/ardrone/land', Empty, queue_size=1)
    self.pubTakeoff = rospy.Publisher('/ardrone/takeoff', Empty, queue_size=1)
    self.pubCommand = rospy.Publisher('/cmd_vel', Twist, queue_size = 3)
    time.sleep(4)
    self.pubTakeoff.publish(Empty())
    time.sleep(4)
    msg = Twist()
    msg.linear.z = 0.3
    self.pubCommand.publish(msg)
    time.sleep(3)
    msg = Twist()
    msg.linear.z = 0.0
    self.pubCommand.publish(msg)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("image_topic_in",Image,self.callback)
    
    # Bigger face
    self.h = -1
    self.x = -1
    self.y = -1
    self.w = -1
    self.lastsign = 0
    self.start = True


  # Callback when a new image arrived
  def callback(self,data):
    msg = Twist()
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
      minNeighbors=7,
      minSize=(55, 55),
      #flags = cv2.CV_HAAR_SCALE_IMAGE
    )
    #print("Found {0} faces!".format(len(faces)))
    self.h = -1
    self.x = -1
    self.y = -1
    self.w = -1
    # Draw a rectangle around the faces
    for (x, y, w, h) in self.faces:
      if h > self.h:
        self.x = x
        self.y = y
        self.w = w
        self.h = h
    if(self.x >= 0):
      print(x, file = sys.stdout)
      if self.w < 80:
        msg.linear.x = 0.025
      elif self.w > 115:
        msg.linear.x = -0.03
      else:
        msg.linear.x = 0
      if 0 < self.x < 60:
        #msg.linear.y = self.vel
        msg.angular.z = 0.25
        self.lastsign = 1
      elif self.x > 200:
        msg.angular.z = -0.25
        self.lastsign = -1
        #msg.linear.y = -self.vel
      else:
        msg.linear.y = 0
      if 0 < self.y < 60:
        msg.linear.z = 0.10
      elif self.y > 120:
        msg.linear.z = -0.10
      else:
        msg.linear.z = 0
      self.pubCommand.publish(msg)
      cv2.rectangle(image, (self.x, self.y), (self.x+self.w, self.y+self.h), (0, 255, 0), 2)
    else:
      msg.linear.x = 0
      msg.linear.y = 0
      msg.linear.z = 0
      msg.angular.x = 0
      msg.angular.y = 0
      msg.angular.z = -self.lastsign*0.05
      self.pubCommand.publish(msg)
    cv2.imshow("Faces found", image)
    cv2.waitKey(1)

def main(args):
  print("OpenCV Version: {}".format(cv2.__version__))
  print("Python Version: {}".format(sys.version))
  rospy.init_node('fd', anonymous=True)
  ic = face_det()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
  main(sys.argv)


