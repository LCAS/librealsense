// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020 FRAMOS GmbH.

#pragma once

#include <memory>
#include "api.h"

namespace librealsense
{
    namespace d400e
    {
        using seconds = double;

        class heartbeat_time
        {
        public:
            static heartbeat_time& get_instance();
            void set(seconds time);
            seconds get();
            heartbeat_time(heartbeat_time const&) = delete;
            void operator=(heartbeat_time const&) = delete;
        private:
            heartbeat_time();
        };

        class device_diagnostics
        {
        public:
            static device_diagnostics& get_instance();
            int set(const char* dev_serial, int toggle);
            device_diagnostics(device_diagnostics const&) = delete;
            void operator=(device_diagnostics const&) = delete;
            
            enum dev_diag_status
            {
                dev_diag_status_OK = 0,
                dev_diag_status_NOK = 1
            };

            enum dev_diag_toggle
            {
                dev_diag_toggle_ON = 1,
                dev_diag_toggle_OFF = 0
            };

        private:
            device_diagnostics();
        };
    }
}