// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#ifndef LIBREALSENSE_RS2_D400E_HPP
#define LIBREALSENSE_RS2_D400E_HPP

#include "rs_types.hpp"

namespace rs2
{
    namespace d400e
    {
        /**
        * set heartbeat time for d400e devices
        * \param[in] time     heartbeat time in seconds
        */
        inline void set_heartbeat_time(double time)
        {
            rs2_error* e = nullptr;
            rs2_d400e_set_heartbeat_time(time, &e);
            rs2::error::handle(e);
        }

        /**
        * retrieve heartbeat time for d400e devices
        * \return             heartbeat time in seconds
        */
        inline double get_heartbeat_time()
        {
            rs2_error* e = nullptr;
            auto time = rs2_d400e_get_heartbeat_time(&e);
            rs2::error::handle(e);

            return time;
        }

        /**
        * set buffer count for d400e devices
        * \param[in] buffer_count     number of buffers
        */
        inline void set_buffer_count(int buffer_count)
        {
            rs2_error* e = nullptr;
            rs2_d400e_set_buffer_count(buffer_count, &e);
            rs2::error::handle(e);
        }

        /**
        * retrieve buffer count for d400e devices
        * \return             buffer count in seconds
        */
        inline int get_buffer_count()
        {
            rs2_error* e = nullptr;
            auto buffer_count = rs2_d400e_get_buffer_count(&e);
            rs2::error::handle(e);

            return buffer_count;
        }

        /**
        * enable or disable diagnostics feature for d400e devices
        * \param[in] dev_serial     Serial number of a device on which diagnostic feature have to be toggled
        * \param[in] toggle         1 - to enable device diagnostics feature, 0 - to disable device diagnostics feature
        * \return                   0 on success, 1 on failure
        */
        inline int toggle_device_diagnostics(const char* dev_serial, int toggle)
        {
            rs2_error* e = nullptr;
            int status = rs2_d400e_toggle_device_diagnostics(dev_serial, toggle, &e);
            rs2::error::handle(e);

            return status;
        }
    }
}
#endif // LIBREALSENSE_RS2_D400E_HPP
