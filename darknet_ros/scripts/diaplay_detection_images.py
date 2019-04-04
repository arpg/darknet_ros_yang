#!/usr/bin/env python
from __future__ import print_function

import roslib
# roslib.load_manifest('my_package')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


def callback(data):
	


def listener():
    image_pub = rospy.Subscriber('/darknet_ros/detection_image', Image, callback)
    bridge = CvBridge()
    
    rospy.init_node('listener', anonymous=True)

    rospy.spin()

def main(args):
  print (args)

  try:
    listener()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
