# image_predistort
This is a node that applies the distortion from a CameraInfo message. Note that while something like `image_proc` _undistorts_ an image, this node performs the inverse and _distorts_ an image based on a CameraInfo message.

This is intended to predistort an image being sent to a calibrated projector with a lens with a lot of distortion, so that when it projects the image the lens undistorts the image.

This node has not been tested, but it at least runs and does something in ROS Noetic.

## Example usage
```
rosrun image_predistort image_predistort image:=/proj_view/image image_predistort:=/proj_view/image_predistort
```
