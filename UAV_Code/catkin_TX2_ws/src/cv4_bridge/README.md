# cv4_bridge
Modefied [cv_bridge](https://github.com/ros-perception/vision_opencv/tree/melodic) to compile with Opencv4
## Introduction
ROS has pre-build debian package for cv_bridge. But it's incompatible to use OpenCV4 with some version of `cv_bridge`, which is build with OpenCV3.  
In some cases, like using Jestson Xavier NX with Jetpack < 5.0, we can only use `ros-melodic`, while the official pre-install opencv (with cuda) is v4.1 (or higher). So I modefied the `cv_bridge` package to make it compatible with OpenCV4. Note that it's only work for `ros-melodic`, and I'll add supported version for `ros-noetic` if necessary in the future.  
## How to use
Add this repository to the same work space with other packages that you need to run. Then rebuild the whole workspace and your package will find the newer `cv_bridge` first.
## Reference:
   - https://github.com/BrutusTT/vision_opencv/commit/b0281a5c844ea0b0d9e0104674474adf50810f49
   - https://github.com/ros-perception/vision_opencv/issues/272
