# Python Wrapper for FRAMOS D400e camera series

This readme file provides instructions on how to use the D400e camera series with Python.

## Supported platforms

Linux x86_64 - Ubuntu 16 LTS, Ubuntu 18 LTS

Linux ARM64 - Jetson TX2, Jetson Xavier, L4T 32.3.1

Windows x86_64

## Prerequisites

FRAMOS CameraSuite version 4.1.2.0 or higher

Intel® RealSense™ SDK with support for D400e cameras version 2.29.1 or higher

## Notes

The `pyrealsense2` wrapper that is part of this package supports the D400e camera series.

The `pyrealsense2` wrapper available from `pip` does not support the D400e camera series.

The `pybackend2` wrapper that is part of this package does not support the D400e camera series.

## Linux

Install required packages:
```
sudo apt install gcc g++ cmake libglfw3-dev libgtk-3-dev git libssl-dev libusb-1.0-0-dev pkg-config python3 python3-dev python3-pip
```

Additional packages are required on Ubuntu 18:
```
sudo apt install libgl1-mesa-dev libglu1-mesa-dev
```

Install Python packages required by some of the Python examples on x86_64:
```
pip3 install --user opencv-python
```

Install Python packages required by some of the Python examples on the Jetson platforms:

```
sudo apt install python3-opencv
```

Copy the sources to a folder with user permissions, for example home:

```
cp -r /usr/src/librealsense2 ~
```

Create a build folder:
```
cd librealsense2
mkdir build
cd build
```

Generate Makefiles with cmake:
```
cmake ../ -DBUILD_PYTHON_BINDINGS:bool=true -DPYTHON_EXECUTABLE=/usr/bin/python3 -DCMAKE_BUILD_TYPE=Release
```

Build with make:
```
make -j$(nproc)
```

Copy the pyrealsense library next to the example and run it:
```
cd ~/librealsense2-framos/wrappers/python/examples
cp ~/librealsense2-framos/build/wrappers/python/pyrealsense2* .
python3 align-depth2color.py
```

## Windows

Build the Python the `pyrealsense` wrapper from source.

Following tools are required for building the Intel® RealSense™ library:

- Visual Studio 2015
- CMake
- Python

Sources for Intel® RealSense™ library, tools and examples are located in

```
C:\Program Files\FRAMOS\librealsense2\src
```

This folder is owned by the administrator. Before proceeding with the build, 
copy it to a location owned by the user.

Launch the CMake GUI tool.
Set ```Where is the source code``` to the location of the copied ```src``` folder.
Set ```Where to build the binaries``` to the location of the copied ```src``` folder with the ```/build``` suffix.
Select the option ```BUILD_PYTHON_BINDINGS```.
Press the ```Configure``` button to update CMake values. 
Make sure to select the ```Visual Studio 14 2015``` generator and ```x64``` optional platform when prompted.
Press the ```Generate``` button to generate the Visual Studio Solution.
Press the ```Open Project``` button to open the generated Visual Studio Solution.
Inside the solution, set the build type to ```Release``` and build the wrapper by right clicking on the ```pyrealsense2``` projects and selecting ```Build```.

The wrapper must be copied next to the Python script along with required dependencies.

Copy the Python wrapper (name may include additional information)

- `pyrealsense2.pyd`

Copy the RealSense library with support for D400e camera series from `Program Files\FRAMOS\librealsense2\bin`

- `realsense2.dll`

Copy the contents of `Program Files\FRAMOS\CameraSuite\bin`

- `CameraSuite.dll`

Copy the contents of `Program Files\FRAMOS\CameraSuite\GenICam_v3_0\bin\Win64_x64`

- `GCBase_MD_VC120_v3_0.dll`
- `GenApi_MD_VC120_v3_0.dll`
- `Log_MD_VC120_v3_0.dll`
- `MathParser_MD_VC120_v3_0.dll`
- `NodeMapData_MD_VC120_v3_0.dll`
- `XmlParser_MD_VC120_v3_0.dll`

Install Python3 OpenCV package, required by some examples

```
pip3 install --user opencv-python
```

