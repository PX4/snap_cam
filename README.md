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
roslaunch snap_cam <CAM>.launch
```
where `<CAM>` is either `optflow` or `highres` to stream the optical flow or high resolution cameras, respectively.
You can set the parameters (camera, resolution and fps) in the launch files (`pathToYourCatkinWs/src/snap_cam/launch/<cam>.launch`)

You can now subscribe to the images in your own ROS node.

## Camera calibration
For optical flow computations, the optical flow camera needs to be calibrated.
For this you must build this package with catkin as described above and launch the optical flow image publisher:
```sh
roslaunch snap_cam optflow.launch
```

Clone and build this package in a catkin workspace on your computer.
On your computer launch the calibration app:
```sh
export ROS_MASTER_URI=http://<snapdragon IP>:11311
roslaunch snap_cam cameraCalibrator.launch
```

Set the appropriate checkerboard parameters in the app.
Start recording by clicking on the button and record your checkerboard from sufficiently varying angles.
Once done, click stop recording.
The camera calibration will be written to `pathToYourCatkinWs/src/snap_cam/calib/cameraParameters.yaml`.
Push this file to your snapdragon.
```sh
adb push /pathToYourCatkinWs/src/snap_cam/calib/cameraParameters.yaml pathToSnapCam/calib/cameraParameters.yaml
```

## Running the optical flow
Assuming that you want to run the optical flow code without ROS, i.e. by building it according to the plain CMake instructions above, and with an appropriate calibration file, run the following in your build directory:
```sh
./optical_flow [-c /path/to/cameraParameters.yaml -r cam_resolution -n num_features -f fps]
```
All arguments are optional.
* `-r` specifies the camera resolution. The default is `VGA`. Valid resolutions are `VGA` and `QVGA`.
* `-f` specifies the camera frame-rate. The default is 15. Valid values are 15, 24, 30, 60, 90, 120.
* `-n` specifies the number of features with which to compute the optical flow. The default is 10.
* `-c` specifies the calibration file. The default is `../calib/cameraParameters.yaml`.
