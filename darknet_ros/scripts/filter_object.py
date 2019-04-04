#!/usr/bin/env python
from __future__ import print_function

import roslib
# roslib.load_manifest('my_package')
import sys
import rospy
from sensor_msgs.msg import CameraInfo

from darknet_ros_msgs.msg import BoundingBox, BoundingBoxes, Artifact

import yaml

class FilterObject:
    def __init__(self, camera_params_file, bounding_boxes_topic, depth_image_topic, object_pos_topic):
        """
        Basic logic: 
        """
        
        print ('camera_params_file:', camera_params_file)
        print ('bounding_boxes_topic:', bounding_boxes_topic)
        print ('depth_image_topic:', depth_image_topic)
        print ('object_pos_topic:', object_pos_topic)
        
        self.detected_object = Artifact()

        rospy.Subscriber(bounding_boxes_topic, BoundingBoxes, callback=self.callback_position)
        #rospy.Subscriber(depth_image_topic, Depth, callback=self.callback_depth)
        self.pub_objects = rospy.Publisher(object_pos_topic, Artifact, queue_size=10)

        
    def callback_object(self, data):
        """
        Filter the in-coming object (data) if it matches (is close to) an existing object, 
        and update the exsiting object;
        Otherwise (if no match), then add to the maintained object list.
        """


if __name__ == '__main__':
    if len(sys.argv) < 4:
        print ('Usage: \n\t locate_objects.py image_topic depth_topic camera_info_topic object_pos_topic')
    try:
        rospy.init_node('locate_objects')
        fo = FilterObject(sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4])
        rospy.spin()
        
    except KeyboardInterrupt:
        print("Shutting down")
