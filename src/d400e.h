// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020 FRAMOS GmbH.

#pragma once

#include <memory>

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
    }
}