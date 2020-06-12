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

## Dynamic Reconfigure
This wrapper supports dynamic reconfiguration of the various parameters! Launch `rqt_reconfigure` and adjust the parameters accordingly.

```
$ rosrun rqt_reconfigure rqt_reconfigure
```
### Parameters
#### Frame preprocessing
- PYRAMID_LEVELS: Pyramid level
#### FAST detector parameters
- FAST_EPSILON: Threshold for feature detection
- FAST_MIN_ARC_LENGTH: Threshold for feature detection
- FAST_SCORE: FAST_SCORE Setter (Sum of absolute differences/Maximum threshold value)
#### Non-Maximum Suppression (NMS) parameters
- HORIZONTAL_BORDER: Horizontal image detection padding
- VERTICAL_BORDER: Vertical image detection padding
- CELL_SIZE_WIDTH: NMS cell width
- CELL_SIZE_HEIGHT: NMS cell height

## Known Issue(s)
- Ouput image does not render the correct color. (Appears black only)


# License
Licensed under the [MIT License](./LICENSE).
