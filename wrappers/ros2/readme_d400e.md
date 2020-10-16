# ROS2 Wrapper (Beta) for FRAMOS D400e camera series

This readme file provides instructions on how to use the D400e camera series with ROS2.

## Supported platforms

Ubuntu 18.04 x86_64

## Prerequisites

FRAMOS CameraSuite version 4.2.1.0 or higher

Intel® RealSense™ SDK with support for D400e camera series version 2.33.10 or higher

## Notes

The ROS2 Wrapper for FRAMOS D400e camera series is based on and is very similar to 
ROS Wrapper for FRAMOS D400e camera series.

## Installation

### Prerequisites

Make sure that `main`, `universe`, `restricted` and `multiverse` repositories are added

```
sudo add-apt-repository main
sudo add-apt-repository universe
sudo add-apt-repository restricted
sudo add-apt-repository multiverse
sudo apt update
```

### Install the ROS2 distribution

Install ROS2 Eloquent Desktop package following instructions available at:
https://index.ros.org/doc/ros2/Installation/Eloquent/Linux-Install-Debians/

### Install additional required packages
```
sudo apt install ros-eloquent-diagnostic-updater
sudo apt install libyaml-cpp-dev
sudo apt install python3-colcon-common-extensions
```

### Build Intel® RealSense™ ROS2 wrapper from sources
This instructions are based on tutorial "Using colcon to build packages" available at: 
https://index.ros.org/doc/ros2/Tutorials/Colcon-Tutorial/.
Prepare colcon workspace and copy the content of this archive into workspace.

```
mkdir -p ~/colcon_ws/src/realsense-ros2
cd ~/colcon_ws/src/realsense-ros2
cp -r /usr/src/librealsense2/wrappers/ros2/. .
echo "source /opt/ros/eloquent/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Build ROS2 wrapper package

```
cd ~/colcon_ws
colcon build
echo "source ~/colcon_ws/install/local_setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Usage Instructions for single camera

Both single camera and multiple camera case are started in the same way. The difference is only in the `multicam_config_file.yaml` config file.
Enter the serial numbers or IP addresses of cameras in the `serial_no` or `ip_address` fields in the file `multicam_config_file.yaml` to use specific cameras. Serial number has precedence over IP address. If both fields are empty, the first detected camera is used.
Start the camera node in ROS2:

```
ros2 launch realsense2_camera framos_multiple_devices_launch.py
```

Launch `rviz` in another terminal

```
ros2 run rviz2 rviz2
```

In the `rviz` GUI 
- change the `Fixed Frame` from `map` to `camera_depth_frame`
- click `Add`, select `By topic` and choose `camera/color/image_raw/Image`

## Usage instruction for two or more cameras

Modify `multicam_config_file.yaml` config file located in `~/colcon_ws/src/realsense-ros2/realsense2_camera/config` in a way as shown in `example_multicam_config_file_for_2_cameras.yaml`. Create a new `cameraN` entry for each additional camera in the file `multicam_config_file.yaml`. Make sure that the prefixes of properties match the camera name. Use the existing `camera` and `camera2` entries as a reference.


Start the camera nodes in ROS2

```
ros2 launch realsense2_camera framos_multiple_devices_launch.py
```

Launch `rviz` in another terminal

```
ros2 run rviz2 rviz2
```

## Known issues and limitations

In this beta release, the ROS2 wrapper package needs to be rebuilt each time the `multicam_config_file.yaml` config file is modified.
```
cd ~/colcon_ws
colcon build
```

Dynamic reconfiguration of camera parameters is not functional in the beta release.

Multiple camera use case is not fully supported in the beta release. For example, only streams from first camera will be properly displayed in `rviz` application.

This wrapper currently does not work with ROS2 Foxy Fitzroy on Ubuntu 20.04.