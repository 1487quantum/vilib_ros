# vilib_ros
<a href="LICENSE" ><img src="https://img.shields.io/github/license/1487quantum/vilib_ros?style=for-the-badge"/></a>

ROS wrapper for CUDA Visual Library by RPG.

## Dependencies
The following packages are required for the vilib_ros wrapper.
### CUDA
Ensure that the CUDA toolkit has been installed: [https://developer.nvidia.com/cuda-toolkit](https://developer.nvidia.com/cuda-toolkit)
### vilib
The library could be found [here](https://github.com/uzh-rpg/vilib), follow the installation steps to install the library.
### ROS Packages
- vision_opencv: `$ sudo apt install ros-melodic-vision-opencv`
- image pipeline: `$ sudo apt install ros-melodic-image-pipeline`

## Nodes
There are 2 nodes for this package, which are:
- vilib_ros_node: FAST corner detection via the vilib FAST library
- vilib_ros_tracker: Lucas - Kanade Feature Tracking via the vilib FAST library

### FAST Corner Detection
> vilib_ros_node
#### `/img_in` (sensor_msgs::Image)  [Subscription]
Input image from camera.
#### `/fast_pts` (vilib_ros::keypt) [Publication]
FAST feature keypoints detected from the image.
#### `/img_out`(sensor_msgs::Image) [Publication]
Image overlayed with detected keypoints.

### Feature Tracking (Lucas - Kanade Feature Tracking)
> vilib_ros_tracker
#### `/img_in` (sensor_msgs::Image)  [Subscription]
Input image from camera.
#### `/feature_pts` (vilib_ros::keypt) [Publication]
Track points detected from the image.
#### `/img_out`(sensor_msgs::Image) [Publication]
Image overlayed with detected tracking points.

## Custom message
### vilib_ros::keypt 
A custom msg type is used to publish the various feature keypoints in the `/fast_pts` topic. The structure is as follows:
```
time stamp: The timestamp when the topic is published.
geometry_msgs/Point[] points: The FAST features detected, stored as an array of geometry_msgs/Point.
int32 size: The number of points detcted in the current frame.
```
> **Note:** The z parameter of the geometry_msgs/Point in `vilib_ros_node` is not used, whereas the z parameter of the geometry_msgs/Point in `vilib_ros_tracker` is used as the __tracking point ID__.

## Dynamic Reconfigure
This wrapper supports dynamic reconfiguration of the various parameters for the nodes! Launch `rqt_reconfigure` and adjust the parameters accordingly.

```
$ rosrun rqt_reconfigure rqt_reconfigure
```
### FAST Corner Detection
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

### Feature Tracking
#### Frame preprocessing
- FRAME_IMAGE_PYRAMID_LEVELS: Image pre-processing pyramid level
#### Feature Detection parameters
- FEATURE_DETECTOR_CELL_SIZE_WIDTH: NMS cell width
- FEATURE_DETECTOR_CELL_SIZE_HEIGHT{32}: NMS cell height
- FEATURE_DETECTOR_MIN_LEVEL: Image pre-processing pyramid level (Minimum)
- FEATURE_DETECTOR_MAX_LEVEL: Image pre-processing pyramid level (Maximum)
- FEATURE_DETECTOR_HORIZONTAL_BORDER: Horizontal image detection padding
- FEATURE_DETECTOR_VERTICAL_BORDER: Vertical image detection padding

#### FAST Parameters
- FEATURE_DETECTOR_FAST_EPISLON: Threshold for feature detection
- FEATURE_DETECTOR_FAST_ARC_LENGTH: FAST arc length
- FEATURE_DETECTOR_FAST_SCORE: FAST_SCORE Setter (Sum of absolute differences/Maximum threshold value)

## Known Issue(s)
- Ouput image does not render the correct color. (Appears black only)


# License
Licensed under the [MIT License](./LICENSE).
