# darknet_ros for MARBLE

![ROS Graph for 'locate artifacts'](https://github.com/yangautumn/darknet_ros/blob/master/darknet_ros/doc/locate-artifacts_rosgraph.png)

The ROS graph kind of explains itself. To make it work correctly, you need to make sure the nodes subscribe the right topics for `camera_info`, `color image` and `depth image`. Note that the `depth image` is not the raw one, but the aligned-to-color-image depth image (on the RealSense camera end, you need to run `roslaunch realsense2_camera rs_aligned_depth.launch`.)

Run the following launch files:
 - `artifact_yolo_v3-tiny.launch`
 - `locate_artifacts.launch`
 
To see the `Artifact` message, run `rostopic echo /artifact/artifact`.

### todo 
In the `locate_artifacts.py`, I considered the vehicle `pose` message, but I didn't really try to receive the messages and perform transformation on the artifact's position. You need to make some changes to do that.
