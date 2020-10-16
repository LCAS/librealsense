#include <realsense2_camera/realsense_node.h>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2/utils.h>
#include <condition_variable>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/update_functions.hpp>

#include <queue>
#include <mutex>
#include <atomic>
#include <thread>

#pragma once

namespace realsense2_camera
{
    struct FrequencyDiagnostics
    {
      FrequencyDiagnostics(rclcpp::Node::SharedPtr node, double expected_frequency, std::string name, std::string hardware_id) :
        expected_frequency_(expected_frequency),
        frequency_status_(diagnostic_updater::FrequencyStatusParam(&expected_frequency_, &expected_frequency_)),
        diagnostic_updater_(node),
        _logger(node->get_logger())
      {
        RCLCPP_INFO(_logger, "Expected frequency for %s = %.5f", name.c_str(), expected_frequency_);
        diagnostic_updater_.setHardwareID(hardware_id);
        diagnostic_updater_.add(frequency_status_);

        diagnostic_updater_.force_update();
      }

      void update()
      {
        frequency_status_.tick();
        //diagnostic_updater_.force_update();
      }

      double expected_frequency_;
      rclcpp::Logger _logger;
      diagnostic_updater::FrequencyStatus frequency_status_;
      diagnostic_updater::Updater diagnostic_updater_;
    };
    typedef std::pair<image_transport::Publisher, std::shared_ptr<FrequencyDiagnostics>> ImagePublisherWithFrequencyDiagnostics;

    class TemperatureDiagnostics
    {
        public:
            TemperatureDiagnostics(rclcpp::Node::SharedPtr node, std::string name, std::string serial_no);
            void diagnostics(diagnostic_updater::DiagnosticStatusWrapper& status);

            void update(double crnt_temperaure)
            {
                _crnt_temp = crnt_temperaure;
                _updater.force_update();
            };

        private:
            double _crnt_temp;
            diagnostic_updater::Updater _updater;

    };

    class NamedFilter
    {
        public:
            std::string _name;
            std::shared_ptr<rs2::filter> _filter;

        public:
            NamedFilter(std::string name, std::shared_ptr<rs2::filter> filter):
            _name(name), _filter(filter)
            {}
    };

    class PipelineSyncer : public rs2::asynchronous_syncer
    {
    public: 
        void operator()(rs2::frame f) const
        {
            invoke(std::move(f));
        }
    };

    class SyncedImuPublisher
    {
        public:
            SyncedImuPublisher() {_is_enabled=false;};
            SyncedImuPublisher(rclcpp::Node::SharedPtr node, std::string topic_name, std::size_t waiting_list_size=1000);
            ~SyncedImuPublisher();
            void Pause();   // Pause sending messages. All messages from now on are saved in queue.
            void Resume();  // Send all pending messages and allow sending future messages.
            void Publish(sensor_msgs::msg::Imu msg);     //either send or hold message.
            uint32_t getNumSubscribers() { return _publisher->get_subscription_count();};
            void Enable(bool is_enabled) {_is_enabled=is_enabled;};
        
        private:
            void PublishPendingMessages();

        private:
            std::mutex                                                          _mutex;
            rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr                 _publisher;
            rclcpp::Node::SharedPtr                                             _node;
            bool                                                                _pause_mode;
            std::queue<sensor_msgs::msg::Imu>                                   _pending_messages;
            std::size_t                                                         _waiting_list_size;
            bool                                                                _is_enabled;
            std::string                                                         _topic_name;
    };

    class BaseRealSenseNode : public InterfaceRealSenseNode
    {
    public:
        BaseRealSenseNode(rclcpp::Node* nh,
                          rclcpp::Node::SharedPtr pnh,
                          rs2::device dev,
                          const std::string& serial_no);

        void toggleSensors(bool enabled);
        virtual std::vector<geometry_msgs::msg::TransformStamped> getStaticTransforms() override;
        virtual void publishTopics() override;
        virtual void registerDynamicReconfigCb(rclcpp::Node::SharedPtr nh) override;
        virtual ~BaseRealSenseNode();

        enum imu_sync_method{NONE, COPY, LINEAR_INTERPOLATION};

    protected:
        class float3
        {
            public:
                float x, y, z;

            public:
                float3& operator*=(const float& factor)
                {
                    x*=factor;
                    y*=factor;
                    z*=factor;
                    return (*this);
                }
                float3& operator+=(const float3& other)
                {
                    x+=other.x;
                    y+=other.y;
                    z+=other.z;
                    return (*this);
                }
        };

        bool _is_running;
        std::string _base_frame_id;
        std::string _odom_frame_id;
        std::map<stream_index_pair, std::string> _frame_id;
        std::map<stream_index_pair, std::string> _optical_frame_id;
        std::map<stream_index_pair, std::string> _depth_aligned_frame_id;
        rclcpp::Node::SharedPtr _node;
        rclcpp::Node* _nh;
        bool _align_depth;
        std::vector<rs2_option> _monitor_options;

        virtual void calcAndPublishStaticTransform(const stream_index_pair& stream, const rs2::stream_profile& base_profile);
        rs2::stream_profile getAProfile(const stream_index_pair& stream);
        tf2::Quaternion rotationMatrixToQuaternion(const float rotation[9]) const;
        void publish_static_tf(const rclcpp::Time& t,
                               const float3& trans,
                               const tf2::Quaternion& q,
                               const std::string& from,
                               const std::string& to);
    private:
        class CimuData
        {
            public:
                CimuData() : m_time(-1) {};
                CimuData(const stream_index_pair type, Eigen::Vector3d data, uint64_t time): //time in nanoseconds
                    m_type(type),
                    m_data(data),
                    m_time(time){};
                bool is_set() {return m_time > 0;};
            public:
                stream_index_pair m_type;
                Eigen::Vector3d m_data;
                uint64_t          m_time;
        };

        static std::string getNamespaceStr(rclcpp::Node::SharedPtr nh);
        void getParameters();
        void setupDevice();
        void setupErrorCallback();
        void setupPublishers();
        void enable_devices();
        void setupFilters();
        void setupStreams();
        void setBaseTime(double frame_time, bool warn_no_metadata);
        cv::Mat& fix_depth_scale(const cv::Mat& from_image, cv::Mat& to_image);
        void clip_depth(rs2::depth_frame depth_frame, float clipping_dist);
        void updateStreamCalibData(const rs2::video_stream_profile& video_profile);
        void SetBaseStream();
        void publishStaticTransforms();
        void publishDynamicTransforms();
        void publishIntrinsics();
        void runFirstFrameInitialization(rs2_stream stream_type);
        void publishPointCloud(rs2::points f, const rclcpp::Time& t, const rs2::frameset& frameset);
        msg::Extrinsics rsExtrinsicsToMsg(const rs2_extrinsics& extrinsics, const std::string& frame_id) const;
        
        msg::IMUInfo getImuInfo(const stream_index_pair& stream_index);
        void publishFrame(rs2::frame f, const rclcpp::Time& t,
                          const stream_index_pair& stream,
                          std::map<stream_index_pair, cv::Mat>& images,
                          const std::map<stream_index_pair, rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr>& info_publishers,
                          //TODO use diagnostic in future
                          //const std::map<stream_index_pair, ImagePublisherWithFrequencyDiagnostics>& image_publishers,
                          const std::map<stream_index_pair, image_transport::Publisher>& image_publishers,
                          std::map<stream_index_pair, int>& seq,
                          std::map<stream_index_pair, sensor_msgs::msg::CameraInfo>& camera_info,
                          const std::map<stream_index_pair, std::string>& optical_frame_id,
                          const std::map<rs2_stream, std::string>& encoding,
                          bool copy_data_from_frame = true);
        bool getEnabledProfile(const stream_index_pair& stream_index, rs2::stream_profile& profile);

        void publishAlignedDepthToOthers(rs2::frameset frames, const rclcpp::Time& t);
        sensor_msgs::msg::Imu CreateUnitedMessage(const CimuData accel_data, const CimuData gyro_data);

        void FillImuData_Copy(const CimuData imu_data, std::deque<sensor_msgs::msg::Imu>& imu_msgs);
        void ImuMessage_AddDefaultValues(sensor_msgs::msg::Imu& imu_msg);
        void FillImuData_LinearInterpolation(const CimuData imu_data, std::deque<sensor_msgs::msg::Imu>& imu_msgs);
        void imu_callback(rs2::frame frame);
        void imu_callback_sync(rs2::frame frame, imu_sync_method sync_method=imu_sync_method::COPY);
        void pose_callback(rs2::frame frame);
        void multiple_message_callback(rs2::frame frame, imu_sync_method sync_method);
        void frame_callback(rs2::frame frame);
        void registerDynamicOption(rclcpp::Node::SharedPtr nh, rs2::options sensor, std::string& module_name);
        //void readAndSetDynamicParam(ros::NodeHandle& nh1, std::shared_ptr<ddynamic_reconfigure::DDynamicReconfigure> ddynrec, const std::string option_name, const int min_val, const int max_val, rs2::sensor sensor, int* option_value);
        void registerAutoExposureROIOptions(rclcpp::Node::SharedPtr nh);
        void set_auto_exposure_roi(const std::string option_name, rs2::sensor sensor, int new_value);
        void set_sensor_auto_exposure_roi(rs2::sensor sensor);
        rs2_stream rs2_string_to_stream(std::string str);
        void startMonitoring();
        void publish_temperature();

        rs2::device _dev;
        std::map<stream_index_pair, rs2::sensor> _sensors;
        std::map<std::string, std::function<void(rs2::frame)>> _sensors_callback;
        rclcpp::AsyncParametersClient::SharedPtr _parameters_client;
        rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr _param_sub;

        std::string _json_file_path;
        std::string _serial_no, _device_name;
        float _depth_scale_meters;
        float _clipping_distance;
        bool _allow_no_texture_points;

        double _linear_accel_cov;
        double _angular_velocity_cov;
        bool  _hold_back_imu_for_frames;

        std::map<stream_index_pair, rs2_intrinsics> _stream_intrinsics;
        std::map<stream_index_pair, int> _width;
        std::map<stream_index_pair, int> _height;
        std::map<stream_index_pair, int> _fps;
        std::map<rs2_stream, int>        _format;
        std::map<stream_index_pair, bool> _enable;
        std::map<rs2_stream, std::string> _stream_name;
        bool _publish_tf;
        double _tf_publish_rate;
        tf2_ros::StaticTransformBroadcaster _static_tf_broadcaster;
        tf2_ros::TransformBroadcaster _dynamic_tf_broadcaster;
        std::vector<geometry_msgs::msg::TransformStamped> _static_tf_msgs;
        std::shared_ptr<std::thread> _tf_t;

        //TODO use diagnostic updater
        //std::map<stream_index_pair, ImagePublisherWithFrequencyDiagnostics> _image_publishers;
        std::map<stream_index_pair, image_transport::Publisher> _image_publishers;
        std::map<stream_index_pair, rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr> _imu_publishers;
        std::shared_ptr<SyncedImuPublisher> _synced_imu_publisher;
        std::map<rs2_stream, int> _image_format;
        std::map<stream_index_pair, rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr> _info_publisher;
        std::map<stream_index_pair, rclcpp::Publisher<msg::IMUInfo>::SharedPtr> _imu_info_publisher;
        std::map<stream_index_pair, cv::Mat> _image;
        std::map<rs2_stream, std::string> _encoding;

        std::map<stream_index_pair, int> _seq;
        std::map<rs2_stream, int> _unit_step_size;
        std::map<stream_index_pair, sensor_msgs::msg::CameraInfo> _camera_info;
        std::atomic_bool _is_initialized_time_base;
        double _camera_time_base;
        std::map<stream_index_pair, std::vector<rs2::stream_profile>> _enabled_profiles;

        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _pointcloud_publisher;
        rclcpp::Time _ros_time_base;
        bool _sync_frames;
        bool _pointcloud;
        bool _publish_odom_tf;
        imu_sync_method _imu_sync_method;
        std::string _filters_str;
        stream_index_pair _pointcloud_texture;
        PipelineSyncer _syncer;
        std::vector<NamedFilter> _filters;
        std::vector<rs2::sensor> _dev_sensors;
        std::map<rs2_stream, std::shared_ptr<rs2::align>> _align;

        std::map<stream_index_pair, cv::Mat> _depth_aligned_image;
        std::map<stream_index_pair, cv::Mat> _depth_scaled_image;
        std::map<rs2_stream, std::string> _depth_aligned_encoding;
        std::map<stream_index_pair, sensor_msgs::msg::CameraInfo> _depth_aligned_camera_info;
        std::map<stream_index_pair, int> _depth_aligned_seq;
        std::map<stream_index_pair, rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr> _depth_aligned_info_publisher;
        //TODO use diagnostic in future
        //std::map<stream_index_pair, ImagePublisherWithFrequencyDiagnostics> _depth_aligned_image_publishers;
        std::map<stream_index_pair, image_transport::Publisher> _depth_aligned_image_publishers;
        std::map<stream_index_pair, rclcpp::Publisher<msg::Extrinsics>::SharedPtr> _depth_to_other_extrinsics_publishers;
        std::map<stream_index_pair, rs2_extrinsics> _depth_to_other_extrinsics;
        std::map<std::string, rs2::region_of_interest> _auto_exposure_roi;
        std::map<rs2_stream, bool> _is_first_frame;
        std::map<rs2_stream, std::vector<std::function<void()> > > _video_functions_stack;

        typedef std::pair<rs2_option, std::shared_ptr<TemperatureDiagnostics>> OptionTemperatureDiag;
        std::vector< OptionTemperatureDiag > _temperature_nodes;
        std::shared_ptr<std::thread> _monitoring_t;
        mutable std::condition_variable _cv;

        stream_index_pair _base_stream;
        const std::string _namespace;

        sensor_msgs::msg::PointCloud2 _msg_pointcloud;
        std::vector< unsigned int > _valid_pc_indices;
        rclcpp::Logger _logger;
        rclcpp::Clock _ros_clock;
    };
}