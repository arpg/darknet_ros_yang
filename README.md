# darknet_ros for MARBLE

orginal repo: https://github.com/leggedrobotics/darknet_ros

![ROS Graph for 'locate artifacts'](https://github.com/yangautumn/darknet_ros/blob/master/darknet_ros/doc/locate-artifacts_rosgraph.png)

The ROS graph kind of explains itself. To make it work correctly, you need to make sure the nodes subscribe the right topics for `camera_info`, `color image` and `depth image`. Note that the `depth image` is not the raw one, but the aligned-to-color-image depth image (on the RealSense camera end, you need to run `roslaunch realsense2_camera rs_aligned_depth.launch`.)

Run the following launch files:
 - `artifact_yolo_v3-tiny.launch`
 - `locate_artifacts.launch`

NOTE: Must run `yolo-v3` first to ensure that the first color message can be passed to `yolo/darknet_ros` to start the loop.
### todo
A better logic should be used for robustness.

[Now I set a timer. After `time_threshold` (1 second), if the flag for publishing color image is still `false`, then set it to `true` to continue the loop. The threshold should be set related to the frequency of `yolo`, for example fps = 15, the threshold should be larger than 1/15 second.]

The settings/config file for darknet_ros is located in `darknet_ros/config/artifact_ros.yaml` or `ros.yaml`.

To see the `Artifact` message, run `rostopic echo /artifact/artifact`.

### todo 
In the `locate_artifacts.py`, I considered the vehicle `pose` message, but I didn't really try to receive the messages and perform transformation on the artifact's position. You need to make some changes to do that.
