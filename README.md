# Snapdragon: Camera driver

This page shows you how to set up your snapdragon flight to use the cameras and optical flow.
The described package can be used with ROS by building with catkin or, alternatively, with pure cmake, where only the executables that do not depend on ROS will be built.

## Building with pure CMake
For the pure CMake install variant, clone the required repositories in a directory, e.g. `~/src`:
```sh
cd ~/src
git clone https://github.com/ethz-ait/klt_feature_tracker.git
git clone https://github.com/ChristophTobler/snap_cam.git
```

Compile with:
```sh
cd snap_cam
mkdir -p build
cd build
cmake ..
make
```

Run the optical flow application with (note that you need to be root for this):
```sh
./optical_flow
```

## Building with ROS
### Prerequisites
To run the ROS nodes on the snapdragon flight, ROS indigo has to be installed. Follow [this](http://wiki.ros.org/indigo/Installation/UbuntuARM) link to install it on your snapdragon flight. (preferably using the linaro user: $ su linaro)

If you're having permission issues while installing ros try
```sh
sudo chown -R linaro:linaro /home/linaro
```

#### Install the following packages:
<br />  mavlink
```sh
sudo apt-get install ros-indigo-mavlink
```
<br />  tf
```sh
sudo apt-get install ros-indigo-tf
```
orocos
```sh
sudo apt-get install ros-indigo-orocos-toolchain
```
angles
```sh
sudo apt-get install ros-indigo-angles
```
tf2/tf2_ros
```sh
sudo apt-get install ros-indigo-tf2
sudo apt-get install ros-indigo-tf2-ros
```

#### create a catkin workspace
Next you have to create a catkin workspace (e.g. in /home/linaro)
```sh
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
cd ..
catkin_make
```

Then clone the following four catkin packages and build again
```sh
cd src
git clone https://github.com/ros-perception/vision_opencv
git clone https://github.com/ros-perception/image_common
git clone https://github.com/ethz-ait/klt_feature_tracker.git
git clone https://github.com/ChristophTobler/snap_cam.git
```

### Others
eigen3
```sh
sudo apt-get install libeigen3-dev
```
sip
```sh
sudo apt-get install sip-dev
```
yaml-cpp
```sh
sudo apt-get install libyaml-cpp-dev
```

To install OpenCV, flash the package (where to put?) to the snapdragon and install it using

<div class="host-code"></div>

```sh
adb push /path/to/file /home/linaro/

```
```sh
dpkg -i opencv3_20160222-1_armhf.deb
```

## Image publisher node
Once your catkin workspace is built and sourced you can start the image publisher using
```sh
roslaunch snap_cam snap_cam_node.launch
```
You can set the parameters (camera, resolution and fps) in the launch file (/pathToYourCatkinWs/src/snap_cam/launch/optical_flow_node.launch)

You can now subscribe to the images in your own ROS node.
