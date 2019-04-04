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

import glob # as it does pattern matching and expansion

def talker(image_path):
    image_pub = rospy.Publisher('/camera/color/image_raw', Image, queue_size=10)
    bridge = CvBridge()
    
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    image_list = glob.glob(image_path + "*.png")
    image_list.sort()

    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        for img in image_list:
            rospy.loginfo(img)
            cv_image = cv2.imread(img)
            # resized_image = cv2.resize(cv_image, (800, 600))
            image_pub.publish(bridge.cv2_to_imgmsg(cv_image, "bgr8"))
            rate.sleep()

def main(args):
  print (args)
  if len(args) < 2:
    print ("Require a absolute path to images (must be .png)!")
    return 
  try:
    talker(args[1])
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
