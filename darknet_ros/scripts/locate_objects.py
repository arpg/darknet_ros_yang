#!/usr/bin/env python
from __future__ import print_function

import roslib
# roslib.load_manifest('my_package')

import rospy
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image as msg_Image
from cv_bridge import CvBridge, CvBridgeError
import sys
import os

from numpy import median

from darknet_ros_msgs.msg import BoundingBox, BoundingBoxes, Artifact

import yaml

class LocateObject:
    def __init__(self, camera_info_topic, bounding_boxes_topic, depth_image_topic, object_pos_topic):
        """
        Basic logic: 
        """
        
        print ('camera_info_topic:', camera_info_topic)
        print ('bounding_boxes_topic:', bounding_boxes_topic)
        print ('depth_image_topic:', depth_image_topic)
        print ('object_pos_topic:', object_pos_topic)
        
        self.camera_info = None
        self.depth = None
        self.detected_object = Artifact()
        self.bridge = CvBridge()
        
        rospy.Subscriber(camera_info_topic, CameraInfo, callback=self.callback_camera)
        
        rospy.Subscriber(bounding_boxes_topic, BoundingBoxes, callback=self.callback_position)
        
        rospy.Subscriber(depth_image_topic, msg_Image, callback=self.callback_depth)
        
        self.pub_objects = rospy.Publisher(object_pos_topic, Artifact, queue_size=10)

        
    def callback_position(self, data):
        if self.depth is None or self.camera_info is None:
            return
            
        cv_image = self.bridge.imgmsg_to_cv2(self.depth, self.depth.encoding)
        
        fx = self.camera_info.K[0]
        fy = self.camera_info.K[4]
        cx = self.camera_info.K[2]
        cy = self.camera_info.K[5]
        
        # iterate through the bounding boxes
        for box in data.bounding_boxes:
            # get the center of the box
            object_type = box.Class
            if object_type == 'person':
                object_type = 'survivor'
            prob = box.probability
            u = int( (box.xmin + box.xmax)/2 )
            v = int( (box.ymin + box.ymax)/2 )
        
            # find the depth of the box
            # depth = median of a grid of depth values around the center
            # depth = self.depth[x, y]
            z_list = []
            for i in range( max(0, u-20), min(u+20, self.depth.width) ):
                for j in range( max(0, v-20), min(v+20, self.depth.height) ):
                    z_list.append(cv_image[j, i])
                    
            #print ('z_list:', z_list)
            
            z = median(z_list)/1e3 # mm --> meter
            
            # calculate the (x, y, z) of the box
            x = (u-cx)*z/fx
            y = (v-cy)*z/fy
            
            print (object_type, prob, u, v)
            
            print ('Object position', x, y, z)
            
            # sometimes there is invalid estimate to be (0, 0, 0)
            if (x*x + y*y + z*z) < 1e4:
                continue
                
            # publish an object message with object type, probability, and (x, y, z)
            self.detected_object.obj_class = object_type
            self.detected_object.obj_prob = prob
            self.detected_object.position.x = x
            self.detected_object.position.y = y
            self.detected_object.position.z = z
            self.pub_objects.publish(self.detected_object)
            
            
    def callback_depth(self, data):
        print ('recieved depth message')
        self.depth = data

    def callback_camera(self, data):
        print ('recieved camera message')
        self.camera_info = data

if __name__ == '__main__':
    if len(sys.argv) < 4:
        print ('Usage: \n\t locate_objects.py image_topic depth_topic camera_info_topic object_pos_topic')
    try:
        rospy.init_node('locate_objects')
        lo = LocateObject(sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4])
        rospy.spin()
        
    except KeyboardInterrupt:
        print("Shutting down")
