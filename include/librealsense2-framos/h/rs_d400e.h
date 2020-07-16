// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020 FRAMOS GmbH.


/** \file rs_d400e.h
* \brief Exposes functionality specific to D400e devices for C compilers
*/


#ifndef LIBREALSENSE_RS2_D400E_H
#define LIBREALSENSE_RS2_D400E_H

#ifdef __cplusplus
extern "C" {
#endif

#include "rs_types.h"

/**
* Sets heartbeat time in seconds for D400e series devices.
* Heartbeat time is used for device disconnect detection.
* \param       time      New heartbeat time in seconds for all D400e devices. Heartbeat timeout is set to 4x heartbeat time
* \param[out]  error     If non-null, receives any error that occurs during this call, otherwise, errors are ignored
*/
void rs2_d400e_set_heartbeat_time(double time, rs2_error** error);

/**
* Acquires heartbeat time in seconds for D400e series devices.
* Heartbeat time is used for device disconnect detection.
* \param[out]  error     If non-null, receives any error that occurs during this call, otherwise, errors are ignored
* @return                Current heartbeat time in seconds for all D400e devices. Heartbeat timeout is 4x heartbeat time
*/
double rs2_d400e_get_heartbeat_time(rs2_error** error);

/**
* Sets number of buffers for D400e series devices.
* \param       buffer_count Number of buffers to set for all D400e devices
* \param[out]  error     If non-null, receives any error that occurs during this call, otherwise, errors are ignored
*/
void rs2_d400e_set_buffer_count(int buffer_count, rs2_error** error);

/**
* Acquires number for buffers for D400e series devices.
* \param[out]  error     If non-null, receives any error that occurs during this call, otherwise, errors are ignored
* @return                Current number for buffers for all D400e devices
*/
int rs2_d400e_get_buffer_count(rs2_error** error);

#ifdef __cplusplus
}
#endif
#endif
