# Getting Started with FRAMOS RealSense™ SDK2.0 for LabVIEW®

> **Note**:
>This document refers to RealSense™ SDK2.0 for LabVIEW® readme file found in *"librealsense/wrappers/labview"*.

## Introduction:
The latest generation of RealSense Depth Cameras can be controlled via the RealSense SDK2.0, also referred to as libRealSense 2.x.x, or LibRS for short. This includes stereo depth cameras D415 and D435 as well as their corresponding modules, such as D410 and D430. FRAMOS-librealsense2 extends the standard librealsense2 API with features applicable to the D400e cameras.


## Getting Started:
Source files are located in FRAMOS-libreasesnse2 install folder inside of *wrappers* subfolder. It is necessary to copy realsense2.dll file from *bin* to folder with source files. There are few simple demos to help you get started with more to follow.

For D400e devices, FRAMOS-librealsense2 library currently supports following demos, compatible for both D415e and D435e:   
* Monochrome + Depth
* Left-Right
* PointCloud
* Third Color Camera

Further details can be found in *readme* or inside demos.