# D400e librealsense2 API extensions

This readme file provides an overview of extensions to the librealsense2 API implemented to support D400e cameras.

## Heartbeat time

Heartbeat mechanism is used to detect disconnect between the host and a D400e camera. Host sends the heartbeat command to the camera in regular intervals and the camera sends a response. If the camera does not respond in a certain interval, the host considers the camera disconnected. If the camera does not receive a heartbeat command in the same interval, it considers the host disconnected. Length of the interval in which the host sends the heartbeat command is called heartbeat time. Length of the interval after which the host considers the camera disconnected and vice versa is called heartbeat timeout.

Heartbeat time in seconds can be acquired and set using the extended librealsense2 API. Heartbeat timeout is implemented as 4x heartbeat time. All D400e cameras connected to a single application have the same heartbeat time. Setting heartbeat time affects all connected D400e cameras.

C++

```cpp
#include <librealsense2-framos/rs.hpp>
```
```cpp
float heartbeat_time_s = rs2::d400e::get_heartbeat_time();
rs2::d400e::set_heartbeat_time(3.f);
```

C

```c
#include <librealsense2-framos/rs.h>
```
```c
rs2_error* e = 0;
float heartbeat_time_s = rs2_d400e_get_heartbeat_time(e);
rs2_d400e_set_heartbeat_time(3.f, e);
```

Python

```python
heartbeat_time_s = rs.d400e.get_heartbeat_time()
rs.d400e.set_heartbeat_time(3)
```

C#

```c#
namespace Intel.Realsense
```

```c#
double heartbeatTimeS = D400e.GetHearbeatTime();
D400e.SetHeartbeatTime(3);
```

## Buffer count

Images acquired from D400e cameras are stored in circular buffer inside the driver. The library is using images from this buffer and notifies the driver when a buffer is no longer needed. Driver drops images that arrive from camera if this buffer is full because the software is not using the images fast enough.

Buffer count can be acquired and set using the extended librealsense2 API. All D400e cameras connected to a single application have the same number of buffers. The buffer count must be set before instantiating `context` or `pipeline` objects. Setting buffer count after these objects are instantiated will have no effect.

C++

```cpp
#include <librealsense2-framos/rs.hpp>
```
```cpp
int buffer_count = rs2::d400e::get_buffer_count();
rs2::d400e::set_buffer_count(10);
```

C

```c
#include <librealsense2-framos/rs.h>
```
```c
rs2_error* e = 0;
int buffer_count = rs2_d400e_get_buffer_count(e);
rs2_d400e_set_buffer_count(10, e);
```

Python

```python
buffer_count = rs.d400e.get_buffer_count()
rs.d400e.set_buffer_count(10)
```

C#

```c#
namespace Intel.Realsense
```

```c#
double bufferCount = D400e.GetBufferCount();
D400e.SetBufferCount(10);
```

## Camera information

Available camera information in the librealsense2 API is listed in the `rs2_camera_info`  enumeration available in the `librealsense2-framos-framos/h/rs_sensor.h` header file. This enumeration was extended to provide information specific to D400e cameras.

The `RS2_CAMERA_INFO_DEVICE_VERSION` enumerator represents FRAMOS firmware version on a D400e camera. This value is different from the `RS2_CAMERA_INFO_FIRMWARE_VERSION` enumerator which represents the Intel D4 firmware version.

The `RS2_CAMERA_INFO_IP_ADDRESS` enumerator represents the IP address of a D400e camera.

The `RS2_CAMERA_INFO_SUBNET_MASK` enumerator represents the subnet mask of a D400e camera.

Same API calls are used to obtain information from both normal and extended enumerators.

C++

```cpp
rs2::device device; //obtain rs2::device from rs2::context or rs2::pipeline_profile
const char* ip_address = device.get_info(RS2_CAMERA_INFO_IP_ADDRESS);
```

C

```c
rs2_device* dev; //obtain rs2_device* using rs2_create_device()
rs2_error* e = 0;
const char* ip_address;
ip_address = rs2_get_device_info(device, RS2_CAMERA_INFO_IP_ADDRESS, e);
```

Python

```python
#obtain device from rs.context() or rs.pipeline()
ip_address = device.get_info(rs.camera_info.ip_address)
```

C#

```c#
namespace Intel.RealSense
```

```c#
Device device; //obtain Device from Context or PipelineProfile
String ipAddress = device.Info[CameraInfo.IpAddress];
```

## Sensor Options

Available sensor options in the librealsense2 API are listed in the `rs2_option` enumeration available in the `librealsense2-framos-framos/h/rs_option.h` header file. This enumeration was extended to provide options specific to D400e cameras.

The `RS2_OPTION_INTER_PACKET_DELAY` enumerator represents the delay in microseconds between stream packets that the camera sends to the host. The library automatically detects optimal value for this option on initialization.

The `RS2_OPTION_PACKET_SIZE` enumerator represents the size of stream packets in bytes that the camera uses to stream images. The library automatically detects optimal value for this option on initialization. This option cannot be set while the sensor is streaming.

The `RS2_OPTION_USER_OUTPUT_LEVEL` enumerator represents output level of M8 pin3 (opto-isolated OUT). 

The `RS2_OPTION_EXT_TRIGGER_SOURCE` enumerator represents external trigger mode. Value 1 represents hardware trigger and value 2 software trigger. See `Framos_D435e_External_Event_Camera_Synchronization_AppNote` for details.

The `RS2_OPTION_SOFTWARE_TRIGGER` enumerator executes software trigger when set to 1.  See `Framos_D435e_External_Event_Camera_Synchronization_AppNote` for details.

The `RS2_OPTION_SOFTWARE_TRIGGER_ALL_SENSORS` enumerator selects which sensors receive the software trigger signal. When set to 1, both stereo and color sensor receive software trigger signal. When set to 0, only the stereo sensor receives the software trigger signal. See `Framos_D435e_External_Event_Camera_Synchronization_AppNote` for details.

Same API calls are used to set both normal and extended options.

C++

```cpp
rs2::sensor sensor; //obtain rs2::sensor from rs2::device or rs2::context
sensor.set_option(RS2_OPTION_INTER_PACKET_DELAY, 65.f);
```

C

```c
rs2_sensor* sensor; //obtain rs2_sensor* using rs2_create_sensor()
rs2_error* e = 0;
rs2_set_option(sensor, RS2_OPTION_INTER_PACKET_DELAY, 65.f, e);
```

Python

```python
#obtain sensor from device or rs.context()
sensor.set_option(rs.option.inter_packet_delay, 65)
```

C#

```c#
namespace Intel.RealSense
```

```c#
Sensor sensor; //obtain Sensor from Device
sensor.Options[Option.InterPacketDelay].Value = 65;
```

