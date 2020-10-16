#include <realsense2_camera/realsense_node.h>
#include <realsense2_camera/base_realsense_node.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <regex>

using namespace realsense2_camera;

RealSenseNode::RealSenseNode():
    Node("RealSenseCameraNode", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)),
    _is_alive(true),
    _static_tf_broadcaster(this),
    _logger(this->get_logger())
{
    RCLCPP_INFO(_logger, "RealSense ROS v%s", REALSENSE_ROS_VERSION_STR);
    RCLCPP_INFO(_logger, "Running with LibRealSense v%s", RS2_API_VERSION_STR);
    //TODO
    //Provjeriti i dodati hvatanje signala ima na ros2 githubu
    //signal(SIGINT, signalHandler);
    auto severity = rs2_log_severity::RS2_LOG_SEVERITY_ERROR;
    tryGetLogSeverity(severity);
    if (rs2_log_severity::RS2_LOG_SEVERITY_DEBUG == severity) {
      console_bridge::setLogLevel(console_bridge::CONSOLE_BRIDGE_LOG_DEBUG);
    }

    rs2::log_to_console(severity);

    this->declare_parameter<std::string>("multicam_config_file", std::string("/config/multicam_config_file.yaml"));
}

RealSenseNode::~RealSenseNode() 
{
   _is_alive = false;
    if (_query_thread.joinable())
    {
        _query_thread.join();
    }

    for (uint32_t i = 0; i < _device_list.size(); i++)
    {
        if (_device_list[i]->device)
        {
            _device_list[i]->realSenseNode.reset();
        }
    }
}

std::string RealSenseNode::parse_usb_port(std::string line)
{
    std::string port_id;
    std::regex self_regex("(?:[^ ]+/usb[0-9]+[0-9./-]*/){0,1}([0-9.-]+)(:){0,1}[^ ]*", std::regex_constants::ECMAScript);
    std::smatch base_match;
    bool found = std::regex_match(line, base_match, self_regex);
    if (found)
    {
        port_id = base_match[1].str();
        if (base_match[2].str().size() == 0)    //This is libuvc string. Remove counter is exists.
        {
            std::regex end_regex = std::regex(".+(-[0-9]+$)", std::regex_constants::ECMAScript);
            bool found_end = std::regex_match(port_id, base_match, end_regex);
            if (found_end)
            {
                port_id = port_id.substr(0, port_id.size() - base_match[1].str().size());
            }
        }
    }
    return port_id;
}

void RealSenseNode::loadParameters()
{
    std::string multicam_config_file;
    std::string path = ament_index_cpp::get_package_share_directory("realsense2_camera");

    this->get_parameter("multicam_config_file", multicam_config_file);
    //TODO provjeriti dal file postoji, ako ne da izbaci gresku!
    YAML::Node config = YAML::LoadFile(path+multicam_config_file);

    this->parseYAML(config, &_device_list);
}

void RealSenseNode::parseYAML(YAML::Node &node, std::vector<camera_device*> *device_list)
{

    for(YAML::iterator it=node.begin(); it!=node.end(); ++it)
    {
        YAML::Node child = it->second;
           
        if (it->first.as<std::string>().compare("ros__parameters") == 0)
        {
            for(YAML::iterator it_y=child.begin(); it_y!=child.end(); ++it_y)
            {
                YAML::Node second_child = it_y->second;

                auto cd = new camera_device();

                cd->serial = second_child["serial_no"].as<std::string>();
                cd->ip_address = second_child["ip_address"].as<std::string>();
                cd->usb_port_id = second_child["usb_port_id"].as<std::string>();
                cd->device_type = second_child["device_type"].as<std::string>();
                cd->rosbag_filename = second_child["rosbag_filename"].as<std::string>();
                cd->camera_name = it_y->first.as<std::string>();
                cd->is_found = false;
                cd->is_started = false;
                cd->initial_reset = second_child["initial_reset"].as<bool>();
                cd->node = this->create_sub_node(cd->camera_name);

                device_list->push_back(cd);
            }
        }
        else parseYAML(child, device_list);
    }
}

void RealSenseNode::publishStaticTransforms()
{
       std::vector<geometry_msgs::msg::TransformStamped> static_tf_msgs;

       for (uint32_t i = 0; i < _device_list.size(); i++)
       {
               if (_device_list[i]->is_started)
               {
                       auto transforms = _device_list[i]->realSenseNode->getStaticTransforms();
                       static_tf_msgs.insert(std::end(static_tf_msgs), std::begin(transforms), std::end(transforms));
               }
       }

       _static_tf_broadcaster.sendTransform(static_tf_msgs);
}

void RealSenseNode::getDeviceFromBag()
{
    for (uint32_t i = 0; i < _device_list.size(); i++)
    {
        if (!_device_list[i]->rosbag_filename.empty() && !_device_list[i]->is_found)
        {
            RCLCPP_INFO(_logger, "publish topics from rosbag file: %s", _device_list[i]->rosbag_filename.c_str());
            auto pipe = std::make_shared<rs2::pipeline>();
            rs2::config cfg;
            cfg.enable_device_from_file(_device_list[i]->rosbag_filename.c_str(), false);
            cfg.enable_all_streams();
            pipe->start(cfg); //File will be opened in read mode at this point
            _device_list[i]->device = pipe->get_active_profile().get_device();
            _device_list[i]->serial = _device_list[i]->device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
            _device_list[i]->is_found = true;
        }
    }
}

void RealSenseNode::getDevice(rs2::device_list list)
{
    if (0 == list.size())
    {
        RCLCPP_WARN(_logger, "No RealSense devices were found!");
    }
    else
    {
        RCLCPP_INFO(_logger, " ");

        for (auto&& dev : list)
        {   
            std::string ip;

            bool found_ip = false;
            bool already_on_list = false;

            auto sn = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
            found_ip = dev.supports(RS2_CAMERA_INFO_IP_ADDRESS);
            if (found_ip) ip = dev.get_info(RS2_CAMERA_INFO_IP_ADDRESS);

            for (uint32_t i = 0; i < _device_list.size(); i++)
            {
                if ((_device_list[i]->serial.compare(sn) == 0 || (_device_list[i]->ip_address.compare(ip) == 0 && found_ip)) && _device_list[i]->is_found) 
                {
                    already_on_list = true;
                    break;
                }
            }

            if (!already_on_list)   
            {

                RCLCPP_INFO(_logger, "Device with serial number %s was found.", sn);

                if (found_ip) RCLCPP_INFO(_logger, "Device with ip address %s was found.", ip.c_str());
                std::string pn = dev.get_info(RS2_CAMERA_INFO_PHYSICAL_PORT);
                std::string name = dev.get_info(RS2_CAMERA_INFO_NAME);
                RCLCPP_INFO(_logger, "Device with physical ID %s was found.", pn.c_str());
                std::vector<std::string> results;
                RCLCPP_INFO(_logger, "Device with name %s was found.", name.c_str());
                std::string port_id = parse_usb_port(pn);   

                for (uint32_t i = 0; i < _device_list.size(); i++)
                {
                    if (!_device_list[i]->is_found)
                    {
                        if (port_id.empty() && !found_ip)
                        {
                            std::stringstream msg;
                            msg << "Error extracting usb port from device with physical ID: " << pn << std::endl << "Please report on github issue at https://github.com/IntelRealSense/realsense-ros";
                            if (_device_list[i]->usb_port_id.empty())
                            {
                                RCLCPP_WARN(_logger, msg.str());
                            }
                            else
                            {
                                RCLCPP_ERROR(_logger, msg.str());
                                RCLCPP_ERROR(_logger, "Please use serial number instead of usb port.");
                            }
                        }
                        else if (!port_id.empty() && !found_ip)
                        {
                            RCLCPP_INFO(_logger, "Device with port number %s was found.", port_id.c_str());                    
                        }

                        bool found_device_type(true);
                        if (!_device_list[i]->device_type.empty())
                        {
                            std::smatch match_results;
                            std::regex device_type_regex(_device_list[i]->device_type.c_str(), std::regex::icase);
                            found_device_type = std::regex_search(name, match_results, device_type_regex);
                        }

                        if ((_device_list[i]->ip_address.empty() || (_device_list[i]->ip_address.compare(ip) == 0 && found_ip)) && (_device_list[i]->serial.empty() || _device_list[i]->serial.compare(sn) == 0) && (_device_list[i]->usb_port_id.empty() || port_id == _device_list[i]->usb_port_id) && found_device_type)
                        {
                            _device_list[i]->device = dev;
                            _device_list[i]->serial = sn;
                            _device_list[i]->ip_address = ip;
                            _device_list[i]->is_found = true;

                            bool remove_tm2_handle(_device_list[i]->device && RS_T265_PID != std::stoi(_device_list[i]->device.get_info(RS2_CAMERA_INFO_PRODUCT_ID), 0, 16));
                            if (remove_tm2_handle)
                            {
                                _ctx.unload_tracking_module();
                            }

                            if (_device_list[i]->initial_reset)
                            {
                                _device_list[i]->initial_reset = false;
                                try
                                {
                                    RCLCPP_INFO(_logger, "Resetting device...");
                                    _device_list[i]->device.hardware_reset();
                                    _device_list[i]->device = rs2::device();
                                    
                                }
                                catch(const std::exception& ex)
                                {
                                    std::stringstream msg;
                                    msg << "An exception has been thrown: " << ex.what();
                                    RCLCPP_WARN(_logger, msg.str());
                                }
                            }

                            break;
                        }
                    }
                }
            }
        }

        for (uint32_t i = 0; i < _device_list.size(); i++)
        {
            if (!_device_list[i]->is_found) 
            {
                // T265 could be caught by another node.
                std::string msg ("The requested device with ");
                bool add_and(false);
                if (!_device_list[i]->serial.empty())
                {
                    msg += "serial number " + _device_list[i]->serial;
                    add_and = true;
                }
                if (!_device_list[i]->ip_address.empty())
                {
                    msg += "ip address " + _device_list[i]->ip_address;
                    add_and = true;
                }
                if (!_device_list[i]->usb_port_id.empty())
                {
                    if (add_and)
                    {
                        msg += " and ";
                    }
                    msg += "usb port id " + _device_list[i]->usb_port_id;
                    add_and = true;
                }
                if (!_device_list[i]->device_type.empty())
                {
                    if (add_and)
                    {
                        msg += " and ";
                    }
                    msg += "device name containing " + _device_list[i]->device_type;
                }
                msg += " is NOT found. Will Try again.";
                RCLCPP_WARN(_logger, msg);
            }
        }
    }       
}

void RealSenseNode::change_device_callback(rs2::event_information& info)
{
    for (uint32_t i = 0; i < _device_list.size(); i++ )
    {
        if (info.was_removed(_device_list[i]->device))
        {
            RCLCPP_WARN(_logger, "%s The device has been disconnected!", _device_list[i]->camera_name.c_str());
            _device_list[i]->realSenseNode.reset(nullptr);
            _device_list[i]->device = rs2::device();
            _device_list[i]->is_started = false;
            _device_list[i]->is_found = false;
        }
    }

    rs2::device_list new_devices = info.get_new_devices();
    if (new_devices.size() > 0)
    {
        RCLCPP_INFO(_logger,"Checking new devices...");
        getDevice(new_devices);

        for (uint32_t i = 0; i < _device_list.size(); i++)
        {
            if (_device_list[i]->is_found && !_device_list[i]->is_started)
            {
                _device_list[i]->is_started = true;
                StartDevice(_device_list[i]);
            }
        }
    }
}

void RealSenseNode::onInit()
{
    try
    {
#ifdef BPDEBUG
        std::cout << "Attach to Process: " << getpid() << std::endl;
        std::cout << "Press <ENTER> key to continue." << std::endl;
        std::cin.get();
#endif
        loadParameters();
        /////////////////////////////////////////////////////
    
        std::function<void(rs2::event_information&)> change_device_callback_function = [this](rs2::event_information& info){change_device_callback(info);};
        _ctx.set_devices_changed_callback(change_device_callback_function);

        _query_thread = std::thread([=]()
                    {
                        std::chrono::milliseconds timespan(6000);
                        volatile bool devices_found = false;
                        
                        while (_is_alive && !devices_found)
                        {
                            bool devices_flag = true;
                            getDevice(_ctx.query_devices());
                            getDeviceFromBag();

                            for (uint32_t i = 0; i < _device_list.size(); i++)
                            {
                                devices_flag &= _device_list[i]->is_found;

                                if (_device_list[i]->is_found && !_device_list[i]->is_started)
                                {
                                    _device_list[i]->is_started = true;
                                    StartDevice(_device_list[i]);
                                    publishStaticTransforms();
                                }
                            }

                            devices_found = devices_flag;

                            if (!devices_found) std::this_thread::sleep_for(timespan);
                        }
                    });
    }
    catch(const std::exception& ex)
    {
        std::stringstream msg;
        msg << "An exception has been thrown: " << ex.what();
        RCLCPP_ERROR(_logger, msg.str());
        exit(1);
    }
    catch(...)
    {
        RCLCPP_ERROR(_logger, "Unknown exception has occured!");
        exit(1);
    }
}

void RealSenseNode::StartDevice(camera_device *device)
{
    if (device->realSenseNode) device->realSenseNode.reset();

    std::string pid_str(device->device.get_info(RS2_CAMERA_INFO_PRODUCT_ID));
    uint16_t pid = std::stoi(pid_str, 0, 16);
    switch(pid)
    {
    case SR300_PID:
    case SR300v2_PID:
    case RS400_PID:
    case RS405_PID:
    case RS410_PID:
    case RS460_PID:
    case RS415_PID:
    case RS420_PID:
    case RS420_MM_PID:
    case RS430_PID:
    case RS430_MM_PID:
    case RS430_MM_RGB_PID:
    case RS435_RGB_PID:
    case RS435i_RGB_PID:
    case RS_USB2_PID:
    case RS_L515_PID:
        device->realSenseNode = std::unique_ptr<BaseRealSenseNode>(new BaseRealSenseNode(this, device->node, device->device, device->serial));
        break;
    case RS_T265_PID:
        //device->realSenseNode = std::unique_ptr<T265RealsenseNode>(new T265RealsenseNode(this, device->device, device->serial));
        break;
    default:
        RCLCPP_FATAL(_logger, "Unsupported device! Product ID: 0x%d", pid_str);
        rclcpp::shutdown();
        exit(1);
    }
    assert(device->realSenseNode);
    device->realSenseNode->publishTopics();
}

void RealSenseNode::tryGetLogSeverity(rs2_log_severity& severity) const
{
    static const char* severity_var_name = "LRS_LOG_LEVEL";
    auto content = getenv(severity_var_name);

    if (content)
    {
        std::string content_str(content);
        std::transform(content_str.begin(), content_str.end(), content_str.begin(), ::toupper);

        for (uint32_t i = 0; i < RS2_LOG_SEVERITY_COUNT; i++)
        {
            auto current = std::string(rs2_log_severity_to_string((rs2_log_severity)i));
            std::transform(current.begin(), current.end(), current.begin(), ::toupper);
            if (content_str == current)
            {
                severity = (rs2_log_severity)i;
                break;
            }
        }
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto realsense_node = std::make_shared<RealSenseNode>();

    realsense_node->onInit();

    rclcpp::spin(realsense_node);
    rclcpp::shutdown();

    return 0;
}