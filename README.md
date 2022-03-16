# demiurge-tycho
(_Currently under development_)

This package contains the codebase for the new tracking system in the IRIDA arena. It fuses the input from multiple cameras (at the moment, this is hard-coded to 3) using an unscented Kalman filter (UKF). The robots are detected using ArUco markers and the corresponding OpenCV library.

## Download and build
This assumes you have your catkin workspace in your `$HOME` directory.
```
$ cd ~/catkin_ws/src
$ git clone --recurse-submodules https://github.com/demiurge-project/demiurge-tycho.git
$ cd ..
$ catkin_make
```

## Visualisation in RViz
```
$ roslaunch tycho_launchers multi_camera.launch
$ cd ~/catkin_ws/src/demiurge-tycho/tycho_launchers/launch
$ rosrun rviz rviz -d rviz_cameras.rviz
```
