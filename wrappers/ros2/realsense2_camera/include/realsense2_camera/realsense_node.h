#pragma once

#include <rcl/time.h>
#include <rclcpp/clock.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.h>
#include <builtin_interfaces/msg/time.hpp>

#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <librealsense2/hpp/rs_processing.hpp>
#include <librealsense2/rs_advanced_mode.hpp>

#include <cv_bridge/cv_bridge.h>
#include <console_bridge/console.h>

#include <realsense2_camera/constants.h>
#include <realsense2_camera/msg/extrinsics.hpp>
#include <realsense2_camera/msg/imu_info.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

#include <eigen3/Eigen/Geometry>
#include "yaml-cpp/yaml.h"
#include <csignal>
#include <fstream>
#include <thread>

namespace realsense2_camera
{
    const stream_index_pair COLOR{RS2_STREAM_COLOR, 0};
    const stream_index_pair DEPTH{RS2_STREAM_DEPTH, 0};
    const stream_index_pair INFRA1{RS2_STREAM_INFRARED, 1};
    const stream_index_pair INFRA2{RS2_STREAM_INFRARED, 2};
    const stream_index_pair FISHEYE{RS2_STREAM_FISHEYE, 0};
    const stream_index_pair FISHEYE1{RS2_STREAM_FISHEYE, 1};
    const stream_index_pair FISHEYE2{RS2_STREAM_FISHEYE, 2};
    const stream_index_pair GYRO{RS2_STREAM_GYRO, 0};
    const stream_index_pair ACCEL{RS2_STREAM_ACCEL, 0};
    const stream_index_pair POSE{RS2_STREAM_POSE, 0};

    const std::vector<stream_index_pair> IMAGE_STREAMS = {DEPTH, INFRA1, INFRA2,
                                                          COLOR,
                                                          FISHEYE,
                                                          FISHEYE1, FISHEYE2};

    const std::vector<stream_index_pair> HID_STREAMS = {GYRO, ACCEL, POSE};

    class InterfaceRealSenseNode
    {
    public:
        virtual void publishTopics() = 0;
        virtual void registerDynamicReconfigCb(rclcpp::Node::SharedPtr nh) = 0;
        virtual std::vector<geometry_msgs::msg::TransformStamped> getStaticTransforms() = 0;
        virtual ~InterfaceRealSenseNode() = default;
    };

    class RealSenseNode: public rclcpp::Node
    {
        public:
            RealSenseNode();
            ~RealSenseNode();
            void onInit();
        private:
            typedef struct camera_device
            {
                std::string serial;
                std::string usb_port_id;
                std::string ip_address;
                std::string device_type;
                std::string camera_name;
                std::string rosbag_filename;
                rs2::device device;
                bool is_found = false;
                bool is_started = false;
                bool initial_reset = false;
                rclcpp::Node::SharedPtr node;
                std::unique_ptr<InterfaceRealSenseNode> realSenseNode;
            } camera_device;

            void closeDevice();
            void StartDevice(camera_device *device);
            void loadParameters();
            void change_device_callback(rs2::event_information& info);
            void getDevice(rs2::device_list list);
            void getDeviceFromBag();
            void publishStaticTransforms();
            void tryGetLogSeverity(rs2_log_severity& severity) const;
            void parseYAML(YAML::Node &node, std::vector<camera_device*> *device_list);
            static std::string parse_usb_port(std::string line);

            rs2::context _ctx;
            std::thread _query_thread;
            bool _is_alive;

            std::vector<camera_device*> _device_list;
            tf2_ros::StaticTransformBroadcaster _static_tf_broadcaster;

            rclcpp::Logger _logger;
        }; 
}//end namespace