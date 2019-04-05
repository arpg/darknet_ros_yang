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

import time


class LocateArtifact:
    def __init__(self, camera_info_topic, depth_image_topic, color_image_topic, pose_topic, pub_color_topic, bounding_boxes_topic, artifact_topic):
        """
        Basic logic: 
        """
        
        self.camera_info = None
        self.depth = None
        self.depth_keep = None
        self.pose_keep = None
        self.artifact = Artifact()
        self.bridge = CvBridge()
        self.can_publish_color = True
        
        rospy.Subscriber(camera_info_topic, CameraInfo, callback=self.callback_camera)
        
        rospy.Subscriber(depth_image_topic, msg_Image, callback=self.callback_depth)
        
        rospy.Subscriber(color_image_topic, msg_Image, callback=self.callback_color)
        
        #rospy.Subscriber(pose_topic, odometry_msgs, callback=self.callback_pose)
        
        rospy.Subscriber(bounding_boxes_topic, BoundingBoxes, callback=self.callback_boundingbox)
        
        self.pub_color_image = rospy.Publisher(pub_color_topic, msg_Image, queue_size=1)
        
        self.pub_artifact = rospy.Publisher(artifact_topic, Artifact, queue_size=1)
        
        # to ensure the color message can be resent after a time threshold
        self.start = time.time()
        self.time_threshold = 1 # 5 seconds
            
            
    def callback_depth(self, data):
        print ('recieved depth message')
        self.depth = data

    def callback_camera(self, data):
        print ('recieved camera message')
        self.camera_info = data
        
        
    #def callback_pose(self, data):
        #print ('recieved pose message')
        #self.pose = data
        
    def callback_color(self, data):
        print ('recieved color message')
        # record the depth, pose, color data with the 'same' timestamp
        if self.depth is None:
            print ('depth is None')
            return
        
        #print (self.can_publish_color)
            
        if not self.can_publish_color:
            end = time.time()
            if (end - self.start) > self.time_threshold:
                self.can_publish_color = True
                
        if self.can_publish_color:
            self.pub_color_image.publish(data)
            self.can_publish_color = False
            self.depth_keep = self.depth
            #self.pose_keep = self.pose
            
            self.start = time.time()
            print ('published color message')
        
        
    def callback_boundingbox(self, data):
        if self.depth_keep is None or self.camera_info is None:
            return
            
        cv_image = self.bridge.imgmsg_to_cv2(self.depth_keep, self.depth_keep.encoding)
        
        fx = self.camera_info.K[0]
        fy = self.camera_info.K[4]
        cx = self.camera_info.K[2]
        cy = self.camera_info.K[5]
        
        # iterate through the bounding boxes
        for box in data.bounding_boxes:
            # get the center of the box
            object_type = box.Class
            if object_type == 'none':
                continue
                
            prob = box.probability
            u = int( (box.xmin + box.xmax)/2 )
            v = int( (box.ymin + box.ymax)/2 )
        
            # find the depth of the box
            # depth = median of a grid of depth values around the center
            # depth = self.depth[x, y]
            z_list = []
            for i in range( max(0, u-20), min(u+20, self.depth_keep.width) ):
                for j in range( max(0, v-20), min(v+20, self.depth_keep.height) ):
                    z_list.append(cv_image[j, i])
            
            z = median(z_list)/1e3 # mm --> meter
            
            # calculate the (x, y, z) of the box
            x = (u-cx)*z/fx
            y = (v-cy)*z/fy
            
            print (object_type, prob, u, v)
            
            print ('Object position', x, y, z)
            
            # sometimes there is invalid estimate to be (0, 0, 0)
            if (x*x + y*y + z*z) < 1e-4:
                continue

            # publish an Artifact message with object type, probability, and (x, y, z)
            self.artifact.obj_class = object_type
            self.artifact.obj_prob = prob
            self.artifact.position.x = x
            self.artifact.position.y = y
            self.artifact.position.z = z
            self.pub_artifact.publish(self.artifact)
            
        self.can_publish_color = True
        
        
        
if __name__ == '__main__':
    if len(sys.argv) < 8:
        print ('Usage: \n\t locate_artifacts.py image_topic depth_topic camera_info_topic object_pos_topic')
    try:
        rospy.init_node('locate_artifacts')
        print ('arguments:', sys.argv)
        
        la = LocateArtifact(sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4], sys.argv[5], sys.argv[6], sys.argv[7])
        
        rospy.spin()
        
    except KeyboardInterrupt:
        print("Shutting down")
