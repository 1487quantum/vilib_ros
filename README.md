# vilib_ros
<a href="LICENSE" ><img src="https://img.shields.io/github/license/1487quantum/vilib_ros?style=for-the-badge"/></a>

ROS wrapper for CUDA Visual Library by RPG.

## Dependencies
The following packages are required for the vilib wrapper:
- vision_opencv: `$ sudo apt install ros-melodic-vision-opencv`
- image pipeline: `$ sudo apt install ros-melodic-image-pipeline`

## Topic
### Subscription
#### `/img_in` (sensor_msgs::Image) 
Input image from camera.

### Publication
#### `/fast_pts` (vilib_ros::keypt)
FAST feature keypoints.
#### `/img_out`(sensor_msgs::Image)
Image overlayed with detected keypoints.

## vilib_ros::keypt 
A custom msg type is used to publish the various feature keypoints in the `/fast_pts` topic. The structure is as follows:
```
time stamp: The timestamp when the topic is published.
geometry_msgs/Point[] points: The FAST features detected, stored as an array of geometry_msgs/Point.
int32 size: The number of points detcted in the current frame.
```
