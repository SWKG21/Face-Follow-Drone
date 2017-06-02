#!/usr/bin/env python
import sys
import rospy
import cv2
import time
from std_msgs.msg import String, Empty
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist 
from cv_bridge import CvBridge, CvBridgeError

# Import the messages we're interested in sending and receiving
from geometry_msgs.msg import Twist  	 # for sending commands to the drone
from std_msgs.msg import Empty       	 # for land/takeoff/emergency
from ardrone_autonomy.msg import Navdata # for receiving navdata feedback

if __name__ == '__main__':
  '''rospy.init_node('takeoffland', anonymous=True)
  pubLand = rospy.Publisher('/ardrone/land', Empty, queue_size=1)
  pubTakeoff = rospy.Publisher('/ardrone/takeoff', Empty, queue_size=1)
  time.sleep(4)
  pubTakeoff.publish(Empty())
  time.sleep(10)
  pubLand.publish(Empty())
  '''
  rospy.init_node('takeoffland', anonymous=True)
