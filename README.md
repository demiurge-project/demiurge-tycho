# demiurge-tycho
This package contains the codebase for Tycho, the new tracking system in the IRIDA Robotics Arena. It fuses the input from multiple gigabit ethernet (GigE) cameras (at the moment, this is hard-coded to 3) using an unscented Kalman filter (UKF). The robots are detected using ArUco markers and the corresponding OpenCV library.

The following sections contain the instructions to download, build, configure and test the Tycho software. They correspond to the Quick Start guide from the [technical report](https://iridia.ulb.ac.be/IridiaTrSeries/link/IridiaTr2022-009.pdf), to which we refer for further usage and implementation details. We assume that the user has already installed an array of three gigabit ethernet (GigE) cameras connected to a server through a switch. We use Prosilica GC1600C cameras. Accordingly, the instructions in the forked `avt_vimba_camera` repository should be followed to install the corresponding Vimba SDK and ROS driver.

## Installation
The software has been developed and tested under Ubuntu Linux using the ROS Noetic distribution. To ensure that all dependencies are met, we recommend to install the `desktop_full` ROS metapackage. Additionally, install the `robot_localization` package by running
```
sudo apt install ros-robot-localization
```
in order to use state estimation through sensor fusion.

To download and build the software, navigate to your catkin workspace and run
```
cd src
git clone --recurse-submodules https://github.com/demiurge-project/demiurge-tycho.git
cd ..
catkin_make
```

## Configuration
Before attempting to run Tycho for the first time, three components must be configured: the cameras, the `transformer` node and the `cropper` node.

### Cameras
Each of the `tycho_launchers/launch/mono_camera_<i>.launch` files, where `<i>={0,1,2}`, need to be individually configured. For each camera, first set the `ip` and `guid` parameters to the corresponding values, which can be found by running Vimba Viewer, included in the Vimba SDK. Then, set each of the following parameters as instructed:

#### camera_info_url
The `camera_info_url` parameter points to a YAML file containing all calibration parameters for camera `<i>`, which is stored in the `tycho_launchers/calibrations` directory as `camera_<i>_calibration.yaml`. To generate each YAML file, we use the `camera_calibration` [package](http://wiki.ros.org/camera_calibration). This step should only be required the first time the tracking system is used.

We follow the steps explained in the package documentation for [calibration of monocular cameras](https://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration). First, a calibration board with a checkered pattern is required. As per the instructions, we also use an 8x6 checkerboard with 108mm squares. The board is made of a 3mm MDF sheet reinforced with pine wood beams to prevent it from bending while manipulating it.

The calibration node is then run as
```
rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.108 image:=/camera_<i>/image_raw camera:=/camera_<i>
```
Move the checkerboard around the camera frame until the CALIBRATE button in the calibration window is highlighted. Press CALIBRATE and wait until the process is finished. Hit SAVE to generate a TAR.GZ file containing all the images that were used to obtain the calibration parameters, as well as the corresponding YAML file. Finally, press COMMIT to permanently upload the calibration to the camera. In the YAML file, set the `camera_name` parameter to `camera_<i>`.

#### exposure
The exposure parameter should be adjusted whenever the lighting conditions in the arena change. To ensure consistent performance of the tracking system, set the ambient light intensity to a fixed value. Using Vimba Viewer, open the camera feed and navigate to the Brightness tab. Adjust the exposure until the contrast between black and white is clear and there are no visible reflections; then, set the parameter in the YAML file to the value obtained. Make sure the gain parameter is set to zero throughout this process.

#### whitebalance
Similarly, the white balance should also be adjusted according to the lighting conditions. Using Vimba Viewer, navigate to the white balance configuration menu in the Color tab and hit "Once" to obtain an adequate value.

### Transformer node
The `transformer` node transforms the pose of each robot detected from the camera frame to the arena frame. For each camera, the required calibration parameters are stored in YAML files in the `tycho_transformer/config` directory. To generate the YAML files for each of the three cameras, place the transformer calibration board (see `tycho_transformer/assets` directory) in the centre of the arena. Rotate the board until the `x` and `y` axes are aligned with those of the arena and run
```
roslaunch tycho_launchers transformer_calibration
```
The YAML files will be stored automatically in the `tycho_transformer/config` directory. This step should be repeated whenever the position or the orientation of any of the cameras change.
 
### Cropper node
The cropper node allows the user to define a polygonal region of interest within the field of view of each camera, such that every point outside this region is ignored during tracking. The vertices of the polygons are stored in YAML files in the `tycho_cropper/config` directory. This is an optional feature; therefore, this step is not required if the region of interest is the entire field of the view of the cameras.

To generate the YAML files for each of the three cameras, run
```
roslaunch tycho_launchers cropper_calibration [ratio:=0.8]	
```
Three windows will appear -- one for each camera. If the windows appear either too large or too small, use the `ratio` parameter to adjust the window size.

On each window, left-click on the vertices of the region of interest in either clockwise or anticlockwise order. After the last vertex has been defined, right-click to save the YAML file and close the window. Once all three windows are closed, the calibrator will shut down automatically.

## Basic usage
To launch Tycho, simply run
```
roslaunch tycho_launchers multi_camera.launch [crop:=false]
```
Setting the `crop` parameter to `true` will activate the cropper node.

## Visualisation
The user can visualise the output of Tycho using RViz. Navigate to the `tycho_launchers/config` directory and run
```
rosrun rviz rviz -d rviz_cameras.rviz
```
The three camera feeds and the poses of the robots in the arena will be displayed.
