// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 FRAMOS GmbH.

#include "cs/cs-factory.h"

namespace librealsense {

    std::shared_ptr <device_interface> cs_info::create(std::shared_ptr <context> ctx,
                                                       bool register_device_notifications) const 
    {
        switch (get_camera_model(_hwm.id)) {
            case CS_D435E:            
            case CS_D415E:
                return std::make_shared<d400e_camera>(ctx, this->get_device_data(),
                                                      register_device_notifications);
            default:
                throw std::runtime_error(to_string() << "Unsupported CS model! 0x"
                                                     << std::hex << std::setw(4) << std::setfill('0') << _hwm.id);
        }
    }

    cs_camera_model cs_info::get_camera_model(std::string pid) 
    {
        if (pid.compare(CS_CAMERA_MODEL_D435e) == 0) return CS_D435E;
        else if (pid.compare(CS_CAMERA_MODEL_D415e) == 0) return CS_D415E;
        else return CS_UNDEFINED;
    }

    bool cs_info::is_timestamp_supported(std::string pid) 
    {
        switch (get_camera_model(pid)) {
            case CS_D435E:
            case CS_D415E:
                return true;
            default:
                return false;
        }
    }

    std::vector <std::shared_ptr<device_info>> cs_info::pick_cs_devices(
            std::shared_ptr <context> ctx,
            std::vector <platform::cs_device_info> &cs) 
    {
        std::vector <std::shared_ptr<device_info>> results;

        for (auto &group : cs) {
            auto info = std::make_shared<cs_info>(ctx, group);
            results.push_back(info);
        }

        return results;
    }

    std::vector<platform::cs_device_info> cs_info::query_cs_devices() 
    {
        auto& cs_watcher = cs_device_watcher::get_cs_device_watcher();
        cs_watcher.find_cs_devices(0.15);
        return cs_watcher.get_cs_devices();
    }

    cs_device_watcher::cs_device_watcher() 
    {

    }

    platform::cs_device_info cs_device_watcher::get_cs_device_info(smcs::IDevice device)
    {
        auto info = platform::cs_device_info();
        info.serial = device->GetSerialNumber();
        info.id = device->GetModelName();
        info.info = device->GetManufacturerSpecificInfo();
        return info;
    }

    void cs_device_watcher::find_cs_devices(double max_wait_time_s) 
    {
        auto smcs_api = smcs::GetCameraAPI();
        smcs_api->FindAllDevices(max_wait_time_s);
        auto devices = smcs_api->GetAllDevices();
        for (const auto& device : devices)
            if (device->IsOnNetwork() && device->IsSameSubnet())
                connected_cs_devices_.insert(get_cs_device_info(device));
    }

    std::vector<platform::cs_device_info> cs_device_watcher::get_cs_devices() 
    {
        return std::vector<platform::cs_device_info>(connected_cs_devices_.begin(), connected_cs_devices_.end());
    }

    void cs_device_watcher::OnDisconnect(smcs::IDevice device)
    {
        connected_cs_devices_.erase(get_cs_device_info(device));
    }

    cs_device_watcher& cs_device_watcher::get_cs_device_watcher() 
    {
        static cs_device_watcher instance;
        return instance;
    }

    d400e_camera::d400e_camera(std::shared_ptr<context> ctx,
                               const platform::backend_device_group& group,
                               bool register_device_notifications)
            : device(ctx, group, register_device_notifications),
              cs_device_interface(ctx, group),
              cs_depth(ctx, group),
              cs_color(ctx, group),
              cs_advanced_mode_base()
    {
        environment::get_instance().get_extrinsics_graph().register_extrinsics(*_color_stream, *_depth_stream, _color_extrinsic);

        register_info(RS2_CAMERA_INFO_NAME, "FRAMOS " + _cs_device_info.id);
        register_info(RS2_CAMERA_INFO_SERIAL_NUMBER, _cs_device_info.serial);
        register_info(RS2_CAMERA_INFO_PRODUCT_ID, get_equivalent_pid(_cs_device_info.id));
        register_info(RS2_CAMERA_INFO_FIRMWARE_VERSION, cs_depth::_fw_version);
        register_info(RS2_CAMERA_INFO_DEVICE_VERSION, _cs_device->get_device_version());
        register_info(RS2_CAMERA_INFO_IP_ADDRESS, _cs_device->get_ip_address());
        register_info(RS2_CAMERA_INFO_SUBNET_MASK, _cs_device->get_subnet_mask());
        //added because ROS wrapper 2.2.9 requires this property
        register_info(RS2_CAMERA_INFO_PHYSICAL_PORT, "N/A");
        register_info(RS2_CAMERA_INFO_ADVANCED_MODE, ((is_camera_in_advanced_mode()) ? "YES" : "NO"));

        cs_advanced_mode_init(cs_depth::_hw_monitor, &get_depth_sensor());
    }

    std::shared_ptr<matcher> d400e_camera::create_matcher(const frame_holder& frame) const
    {
        std::vector<stream_interface*> streams = {_depth_stream.get(), _left_ir_stream.get() , _right_ir_stream.get(), _color_stream.get()};
        return matcher_factory::create(RS2_MATCHER_DEFAULT, streams);
    }

    std::vector<tagged_profile> d400e_camera::get_profiles_tags() const
    {
        std::vector<tagged_profile> markers;
        markers.push_back({ RS2_STREAM_DEPTH, -1, (uint32_t)-1, (uint32_t)-1, RS2_FORMAT_ANY, (uint32_t)-1, profile_tag::PROFILE_TAG_SUPERSET | profile_tag::PROFILE_TAG_DEFAULT });
        markers.push_back({ RS2_STREAM_COLOR, -1, (uint32_t)-1, (uint32_t)-1, RS2_FORMAT_ANY, (uint32_t)-1, profile_tag::PROFILE_TAG_SUPERSET | profile_tag::PROFILE_TAG_DEFAULT });
        return markers;
    }

    std::string d400e_camera::get_equivalent_pid(std::string id) const
    {
        if (id.compare(CS_CAMERA_MODEL_D435e) == 0) 
            return "0B07"; //D435 PID
        else if (id.compare(CS_CAMERA_MODEL_D415e) == 0) 
            return "0AD3"; //D415 PID
        else 
            return "N/A";
    }
}