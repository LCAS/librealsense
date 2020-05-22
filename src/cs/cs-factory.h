// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 FRAMOS GmbH.

#ifndef LIBREALSENSE2_CS_FACTORY_H
#define LIBREALSENSE2_CS_FACTORY_H

#include <cs/cs-sensor.h>
#include <cs/cs-device.h>
#include "context.h"

namespace librealsense {

#define CS_PACKET_RESEND_GROUP_MAX_SIZE 1

    class cs_info : public device_info {
    public:
        std::shared_ptr <device_interface> create(std::shared_ptr <context> ctx,
                                                  bool register_device_notifications) const override;

        cs_info(std::shared_ptr <context> ctx,
                platform::cs_device_info hwm)
                : device_info(ctx),
                  _hwm(std::move(hwm)) {}

        static std::vector <std::shared_ptr<device_info>> pick_cs_devices(
                std::shared_ptr <context> ctx,
                std::vector <platform::cs_device_info> &cs);

        platform::backend_device_group get_device_data() const override {
            return platform::backend_device_group({_hwm});
        }

        static std::vector <platform::cs_device_info> query_cs_devices();

        static cs_camera_model get_camera_model(std::string pid);

        static bool is_timestamp_supported(std::string pid);

    private:
        platform::cs_device_info _hwm;
    };

    class d400e_camera : public cs_depth,
                         public cs_color,
                         public cs_advanced_mode_base
    {
    public:
        d400e_camera(std::shared_ptr<context> ctx,
                     const platform::cs_device_info &hwm_device,
                     const platform::backend_device_group& group,
                     bool register_device_notifications);

        std::shared_ptr<matcher> create_matcher(const frame_holder& frame) const override;

        std::vector<tagged_profile> get_profiles_tags() const override;

        cs_sensor& get_cs_sensor(size_t subdevice) { return dynamic_cast<cs_sensor&>(get_sensor(subdevice)); }

        void hardware_reset() override
        {
            if (get_cs_sensor(_color_device_idx).is_streaming()) {
                get_cs_sensor(_color_device_idx).stop();
                get_cs_sensor(_color_device_idx).close();
            }

            if (get_cs_sensor(_depth_device_idx).is_streaming()) {
                get_cs_sensor(_depth_device_idx).stop();
                get_cs_sensor(_depth_device_idx).close();
            }

            _cs_device->reset();
        }
    private:
        std::string get_equivalent_pid(std::string id) const;
    };
}

#endif //LIBREALSENSE2_CS_FACTORY_H