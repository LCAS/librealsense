// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 FRAMOS GmbH.

#include "cs/cs-sensor.h"
#include "cs/cs-options.h"
#include "global_timestamp_reader.h"
#include "stream.h"
#include "device.h"
#include <regex>

#include <array>
#include <set>
#include <unordered_set>
#include <iomanip>

namespace librealsense {

    cs_firmware_version::cs_firmware_version(smcs::IDevice &device)
        : _major(0), _minor(0), _patch(0), _build(0)
    {
        auto device_version = device->GetDeviceVersion();
        std::smatch match;
        if (std::regex_search(device_version, match, std::regex("FW:([0-9])\\.([0-9])\\.([0-9])\\.([0-9])")))
        {
            try
            {
                _major = std::stoi(match[1]);
                _minor = std::stoi(match[2]);
                _patch = std::stoi(match[3]);
                _build = std::stoi(match[4]);
            }
            catch (...)
            {

            }
        }

    }

    cs_sensor::cs_sensor(std::string name,
                         std::shared_ptr<platform::cs_device> cs_device,
                         std::unique_ptr<frame_timestamp_reader> timestamp_reader,
                         device* dev,
                         cs_stream stream)
            : sensor_base(name, dev, (recommended_proccesing_blocks_interface*)this),
              _device(std::move(cs_device)),
              _timestamp_reader(std::move(timestamp_reader)),
              _cs_stream(stream),
              _user_count(0),
              _external_trigger_mode(false)
    {
        register_metadata(RS2_FRAME_METADATA_BACKEND_TIMESTAMP,     make_additional_data_parser(&frame_additional_data::backend_timestamp));
    }

    cs_sensor::~cs_sensor()
    {
        try
        {
            if (_is_streaming)
                cs_sensor::stop();

            if (_is_opened)
                cs_sensor::close();
        }
        catch(...)
        {
            LOG_ERROR("An error has occurred while stop_streaming()!");
        }
    }

    void cs_sensor::open(const stream_profiles& requests)
    {
        std::lock_guard<std::mutex> lock(_configure_lock);
        if (_is_streaming)
            throw wrong_api_call_sequence_exception("open(...) failed. CS device is streaming!");
        else if (_is_opened)
            throw wrong_api_call_sequence_exception("open(...) failed. CS device is already opened!");

        auto on = std::unique_ptr<power>(new power(std::dynamic_pointer_cast<cs_sensor>(shared_from_this())));

        if (!smcs::GetCameraAPI()->IsUsingKernelDriver())
            throw wrong_api_call_sequence_exception("GigE filter driver not loaded! Please make sure that the driver is installed and available.");

        _source.init(_metadata_parsers);
        _source.set_sensor(_source_owner->shared_from_this());

        std::vector<platform::stream_profile> commited;

        _cs_selected_streams.clear();

        for (auto&& req_profile : requests)
        {
            auto&& req_profile_base = std::dynamic_pointer_cast<stream_profile_base>(req_profile);
            auto selected_stream = get_stream(req_profile->get_stream_type(), req_profile->get_stream_index());

            try
            {
                auto infrared_stream = selected_stream == CS_STREAM_IR_LEFT || selected_stream == CS_STREAM_IR_RIGHT;
                if (infrared_stream && !_device->is_infrared_supported())
                    throw wrong_api_call_sequence_exception("Device does not support infrared streams!");

                unsigned long long last_frame_number = 0;
                rs2_time_t last_timestamp = 0;
                _device->probe_and_commit(req_profile_base->get_backend_profile(),
                                          [this, req_profile_base, req_profile, last_frame_number, last_timestamp](platform::stream_profile p, platform::frame_object f, std::function<void()> continuation) mutable
                {
                    const auto&& system_time = environment::get_instance().get_time_service()->get_time();
                    const auto&& fr = generate_frame_from_data(f, _timestamp_reader.get(), last_timestamp, last_frame_number, req_profile_base);
                    const auto&& requires_processing = true; // TODO - Ariel add option
                    const auto&& timestamp_domain = _timestamp_reader->get_frame_timestamp_domain(fr);
                    const auto&& bpp = get_image_bpp(req_profile_base->get_format());
                    auto&& frame_counter = fr->additional_data.frame_number;
                    auto&& timestamp = fr->additional_data.timestamp;

                    if (!this->is_streaming())
                    {
                        LOG_WARNING("Frame received with streaming inactive,"
                                            << librealsense::get_string(req_profile_base->get_stream_type())
                                            << req_profile_base->get_stream_index()
                                            << ", Arrived," << std::fixed << f.backend_time << " " << system_time);
                        return;
                    }

                    frame_continuation release_and_enqueue(continuation, f.pixels);

                    LOG_DEBUG("Serial number:" << this->get_info(RS2_CAMERA_INFO_SERIAL_NUMBER)
                                               << ",FrameAccepted," << librealsense::get_string(req_profile_base->get_stream_type())
                                               << ",Counter," << std::dec << fr->additional_data.frame_number
                                               << ",Index," << req_profile_base->get_stream_index()
                                               << ",BackEndTS," << std::fixed << f.backend_time
                                               << ",SystemTime," << std::fixed << system_time
                                               << " ,diff_ts[Sys-BE]," << system_time - f.backend_time
                                               << ",TS," << std::fixed << timestamp << ",TS_Domain," << rs2_timestamp_domain_to_string(timestamp_domain)
                                               << ",last_frame_number," << last_frame_number << ",last_timestamp," << last_timestamp);

                    last_frame_number = frame_counter;
                    last_timestamp = timestamp;

                    const auto&& vsp = As<video_stream_profile, stream_profile_interface>(req_profile);
                    int width = vsp ? vsp->get_width() : 0;
                    int height = vsp ? vsp->get_height() : 0;
                    frame_holder fh = _source.alloc_frame(stream_to_frame_types(req_profile_base->get_stream_type()), width * height * bpp / 8, fr->additional_data, requires_processing);
                    if (fh.frame)
                    {
                        memcpy((void*)fh->get_frame_data(), fr->data.data(), sizeof(byte)*fr->data.size());
                        auto&& video = (video_frame*)fh.frame;
                        video->assign(width, height, width * bpp / 8, bpp);
                        video->set_timestamp_domain(timestamp_domain);
                        fh->set_stream(req_profile_base);
                    }
                    else
                    {
                        LOG_INFO("Dropped frame. alloc_frame(...) returned nullptr");
                        return;
                    }

                    if (!requires_processing)
                    {
                        fh->attach_continuation(std::move(release_and_enqueue));
                    }

                    if (fh->get_stream().get())
                    {
                        _source.invoke_callback(std::move(fh));
                    }

                }, selected_stream);
            }
            catch (...)
            {
                _device->close(_cs_selected_streams);
                throw;
            }
            _cs_selected_streams.push_back(selected_stream);
            commited.push_back(req_profile_base->get_backend_profile());
        }

        _internal_config = commited;

        if (_on_open)
            _on_open(_internal_config);

        _power = move(on);
        _is_opened = true;

        try {
            _device->stream_on([&](const notification& n)
                               {
                                   _notifications_processor->raise_notification(n);
                               }, _cs_selected_streams, _internal_config);
        }
        catch (...)
        {
            std::stringstream error_msg;
            error_msg << "\tFormats: \n";
            for (auto&& profile : _internal_config)
            {
                rs2_format fmt = fourcc_to_rs2_format(profile.format);
                error_msg << "\t " << std::string(rs2_format_to_string(fmt)) << std::endl;
                try {
                    _device->close(_cs_selected_streams);
                }
                catch (...) {}
            }
            error_msg << std::endl;
            reset_streaming();
            _power.reset();
            _is_opened = false;

            throw std::runtime_error(error_msg.str());
        }

        if (Is<librealsense::global_time_interface>(_owner))
        {
            As<librealsense::global_time_interface>(_owner)->enable_time_diff_keeper(true);
        }
        set_active_streams(requests);
    }

    stream_profiles cs_sensor::init_stream_profiles()
    {
        auto lock = environment::get_instance().get_extrinsics_graph().lock();

        std::unordered_set<std::shared_ptr<video_stream_profile>> profiles;
        power on(std::dynamic_pointer_cast<cs_sensor>(shared_from_this()));

        if (_uvc_profiles.empty()){}
        _uvc_profiles = _device->get_profiles(_cs_stream);

        for (auto&& p : _uvc_profiles)
        {
            const auto&& rs2_fmt = fourcc_to_rs2_format(p.format);
            if (rs2_fmt == RS2_FORMAT_ANY)
                continue;

            auto&& profile = std::make_shared<video_stream_profile>(p);
            profile->set_dims(p.width, p.height);
            profile->set_stream_type(fourcc_to_rs2_stream(p.format));
            profile->set_stream_index(0);
            profile->set_format(rs2_fmt);
            profile->set_framerate(p.fps);
            profiles.insert(profile);
        }

        stream_profiles result{ profiles.begin(), profiles.end() };
        return result;
    }

    rs2_extension cs_sensor::stream_to_frame_types(rs2_stream stream)
    {
        // TODO: explicitly return video_frame for relevant streams and default to an error?
        switch (stream)
        {
            case RS2_STREAM_DEPTH:  return RS2_EXTENSION_DEPTH_FRAME;
            default:                return RS2_EXTENSION_VIDEO_FRAME;
        }
    }

    void cs_sensor::close()
    {
        std::lock_guard<std::mutex> lock(_configure_lock);
        if (_is_streaming)
            throw wrong_api_call_sequence_exception("close() failed. CS device is streaming!");
        else if (!_is_opened)
            throw wrong_api_call_sequence_exception("close() failed. CS device was not opened!");

        for (auto&& profile : _internal_config)
        {
            try // Handle disconnect event
            {
                _device->close(_cs_selected_streams);
            }
            catch (...) 
            {
                LOG_ERROR("Error while closing stream!");
                throw;
            }
        }
        reset_streaming();
        if (Is<librealsense::global_time_interface>(_owner))
        {
            As<librealsense::global_time_interface>(_owner)->enable_time_diff_keeper(false);
        }
        _power.reset();
        _is_opened = false;
        set_active_streams({});
    }

    void cs_sensor::start(frame_callback_ptr callback)
    {
        std::lock_guard<std::mutex> lock(_configure_lock);
        if (_is_streaming)
            throw wrong_api_call_sequence_exception("start_streaming(...) failed. CS device is already streaming!");
        else if(!_is_opened)
            throw wrong_api_call_sequence_exception("start_streaming(...) failed. CS device was not opened!");

        raise_on_before_streaming_changes(true); //Required to be just before actual start allow recording to work
        _source.set_callback(callback);
        _is_streaming = true;
    }

    void cs_sensor::stop()
    {
        std::lock_guard<std::mutex> lock(_configure_lock);
        if (!_is_streaming)
            throw wrong_api_call_sequence_exception("stop_streaming() failed. CS device is not streaming!");

        _is_streaming = false;
        raise_on_before_streaming_changes(false);
    }

    void cs_sensor::acquire_power()
    {
        std::lock_guard<std::mutex> lock(_power_lock);
        if (_user_count.fetch_add(1) == 0)
        {
            if (_device->set_power_state(platform::D0) != platform::D0)
                throw wrong_api_call_sequence_exception("set power state(...) failed. CS device cannot be turned on!");
            //for (auto&& xu : _xus) _device->init_xu(xu);
        }
    }

    void cs_sensor::release_power()
    {
        std::lock_guard<std::mutex> lock(_power_lock);
        if (_user_count.fetch_add(-1) == 1)
        {
            if (_device->set_power_state(platform::D3) != platform::D3)
                throw wrong_api_call_sequence_exception("set power state(...) failed. CS device cannot be turned off!");
        }
    }

    void cs_sensor::reset_streaming()
    {
        _source.flush();
        _source.reset();
        _timestamp_reader->reset();
    }

    cs_stream cs_sensor::get_stream(rs2_stream type, int index)
    {
        switch (type)
        {
        case RS2_STREAM_DEPTH:
            return CS_STREAM_DEPTH;
        case RS2_STREAM_COLOR:
            return CS_STREAM_COLOR;
        case RS2_STREAM_INFRARED:
            return index == 1 ? CS_STREAM_IR_LEFT : CS_STREAM_IR_RIGHT;
        default:
            throw wrong_api_call_sequence_exception("unable to map type and index to stream ID!");
        }
    }

    void cs_sensor::register_pu(rs2_option id)
    {
        register_option(id, std::make_shared<cs_pu_option>(*this, id, _cs_stream));
    }

    void cs_sensor::try_register_pu(rs2_option id)
    {
        auto opt = std::make_shared<cs_pu_option>(*this, id, _cs_stream);
        try
        {
            auto range = opt->get_range();
            if (range.max <= range.min || range.step <= 0 || range.def < range.min || range.def > range.max) return;

            auto val = opt->query();
            if (val < range.min || val > range.max) return;
            opt->set(val);

            register_option(id, opt);
        }
        catch (...)
        {
            LOG_WARNING("Exception was thrown when inspecting " << this->get_info(RS2_CAMERA_INFO_NAME) << " property " << opt->get_description());
        }
    }

	void cs_sensor::set_inter_cam_sync_mode(float value)
	{
        if (_is_streaming)
            throw wrong_api_call_sequence_exception("Unable to set Inter Cam Sync Mode while streaming!");

        _device->update_external_trigger_mode_flag(_cs_stream, value);

        _device->set_trigger_mode(value, _cs_stream);
	}

    float cs_sensor::get_inter_cam_sync_mode()
    {
        float inter_cam_sync_mode = _device->get_trigger_mode(_cs_stream);

        _device->update_external_trigger_mode_flag(_cs_stream, inter_cam_sync_mode);

        return inter_cam_sync_mode;
    }

    bool cs_sensor::query_inter_cam_sync_mode()
    {
        return _device->get_intercam_mode(_cs_stream);
    }

    namespace platform
    {
        std::map<std::string, int> cs_device::_cs_device_num_objects_sn;
        std::map<std::string, bool> cs_device::_cs_device_initialized_sn;
        std::map<std::string, bool> cs_device::_cs_device_option_sw_trigger_all_flag_sn;

        cs_device::cs_device(cs_device_info hwm)
                : _device_info(std::move(hwm)),
                    _power_state(D3),
                    _connected_device(NULL),
                    _rgb_pixel_format(RS2_FORMAT_ANY),
                    _infrared_supported(false),
                    _temperature_supported_checked(false),
                    _temperature_supported(false),
                    _software_trigger_supported_checked(false),
                    _software_trigger_supported(false),
                    _selected_source(0),
                    _selected_source_initialized(false) {
            _smcs_api = smcs::GetCameraAPI();
            auto devices = _smcs_api->GetAllDevices();

            for (int i = 0; i < devices.size(); i++) {
                auto serial = devices[i]->GetSerialNumber();
                if (!serial.compare(_device_info.serial)) {
                    _connected_device = devices[i];

                    if (_connected_device == NULL) {
                        break;
                    } else {
                        if (!_connected_device->IsConnected()) {
                            // try to connect to camera if not already connected
                            if (!_connected_device->Connect()) {
                                throw wrong_api_call_sequence_exception(std::string("Could not connect to device with serial number ") + _connected_device->GetSerialNumber());
                            }
                            else {
                                // initialize this part (inter-packet delay) only once
                                if (get_device_init_flag_sn(_device_info.serial) == false) { 
                                    INT64 numOfCameraStream;
                                    _connected_device->GetIntegerNodeValue("GevStreamChannelCount", numOfCameraStream);
                                    for (int i = 0; i < numOfCameraStream; i++)
                                    {
                                        INT64 packetSize;
                                        select_channel((cs_stream)i);

                                        // set optimal inter-packet delay
                                        _connected_device->GetIntegerNodeValue("GevSCPSPacketSize", packetSize);
                                        int interPacketDelay = get_optimal_inter_packet_delay(packetSize);
                                        _connected_device->SetIntegerNodeValue("GevSCPD", interPacketDelay);
                                    }
                                    set_device_init_flag_sn(_device_info.serial, true);
                                }
                            }
                        }

                        _number_of_streams = CS_STREAM_COUNT;
                        _threads = std::vector<std::unique_ptr <std::thread>>(_number_of_streams);
                        _is_capturing = std::vector<std::atomic<bool>>(_number_of_streams);
                        _callbacks = std::vector<frame_callback>(_number_of_streams);
                        _error_handler = std::vector<std::function<void(const notification &n)>>(_number_of_streams);
                        _profiles = std::vector<stream_profile>(_number_of_streams);
                        _cs_firmware_version = cs_firmware_version(_connected_device);

                        constexpr double S_TO_MS_FACTOR = 1000;
                        INT64 frequency;
                        if (!_connected_device->GetIntegerNodeValue("GevTimestampTickFrequency", frequency))
                            throw io_exception("Unable to read GevTimestampTickFrequency");
                        _timestamp_to_ms_factor = S_TO_MS_FACTOR / frequency;

                        for (int i = 0; i < _number_of_streams; i++)
                        {
                            _threads[i] = nullptr;
                            _is_capturing[i] = false;
                            _callbacks[i] = nullptr;

                            // make sure stream parameters are unlocked
                            stream_params_unlock((cs_stream)i);
                        }

                        // increment device counter for device with current SN
                        inc_device_count_sn(_device_info.serial);
                    }
                    // found device with SN
                    break;
                }
            }
            if (_connected_device == NULL) {    // Could not find device with given SN
                throw wrong_api_call_sequence_exception("Could not create CS device with given SN!");
            }
        }

        cs_device::~cs_device() {
            dec_device_count_sn(_device_info.serial);
            for (int i = 0; i < _number_of_streams; i++) {
                deinit_stream((cs_stream)i);
            }
            
            if (get_device_count_sn(_device_info.serial) == 0) {
                if (_connected_device->IsConnected()) {
                    try {
                        //TODO - close all streams
                        close({ CS_STREAM_DEPTH }); // -> Source0
                        close({ CS_STREAM_COLOR }); // -> Source1
                    }
                    catch (const std::exception& ex) {
                        LOG_ERROR(ex.what());
                    }
                    _connected_device->Disconnect();
                }
            }
        }

        std::vector<stream_profile> cs_device::get_profiles(cs_stream stream)
        {
            std::vector<stream_profile> all_stream_profiles;
            stream_profile profile;
            INT64 int64_value;
            bool is_successful = true;

            if (!set_source_locked(stream, false))
                throw wrong_api_call_sequence_exception("Unable to read profiles!");
            
            if (!disable_source_regions(stream))
                throw wrong_api_call_sequence_exception("Unable to read profiles!");
            
            if (!set_region(stream, true))
                throw wrong_api_call_sequence_exception("Unable to read profiles!");

            if (stream == CS_STREAM_COLOR) {
                std::string pixel_format;
                if (_connected_device->GetStringNodeValue("PixelFormat", pixel_format)
                    && pixel_format.compare("YUV422Packed") == 0)
                    _rgb_pixel_format = RS2_FORMAT_BGR8;
                else
                    _rgb_pixel_format = RS2_FORMAT_RGB8;
            }

            if (stream == CS_STREAM_DEPTH) {
                smcs::StringList regions;
                _infrared_supported =
                    _connected_device->GetEnumNodeValuesList("RegionSelector", regions)
                    && regions.size() >= 4;
            }

            smcs::StringList resolution_list;
            is_successful = is_successful & _connected_device->GetEnumNodeValuesList("Resolution", resolution_list);

            for (const auto& resolution : resolution_list) {

                // Resolution is read-only on D435e with FW 1.3.4.0
                std::string old_resolution;
                if (_connected_device->GetStringNodeValue("Resolution", old_resolution) && old_resolution != resolution)
                    is_successful = is_successful & _connected_device->SetStringNodeValue("Resolution", resolution);

                is_successful = is_successful & _connected_device->GetIntegerNodeValue("Width", int64_value);
                profile.width = (uint32_t) int64_value;

                is_successful = is_successful & _connected_device->GetIntegerNodeValue("Height", int64_value);
                profile.height = (uint32_t) int64_value;

                std::string pixelFormat;
                is_successful = is_successful & _connected_device->GetStringNodeValue("PixelFormat", pixelFormat);

                for (auto frameRate : get_frame_rates()) {

                    profile.fps = frameRate;

                    if (is_successful) {
                        profile.format = cs_pixel_format_to_native_pixel_format(pixelFormat);
                        all_stream_profiles.push_back(profile);
            
                        /**
                         * Adding profiles for IR streams.
                         * GREY is for the left IR stream.
                         * Y8I is for the right IR stream.
                         * GREY must be added before Y8I. Order is important because 
                         * only the first compatible format is selected.
                         */
                        if (is_successful && get_stream_source(stream) == 0) {
                            profile.format = rs_fourcc('G', 'R', 'E', 'Y');
                            all_stream_profiles.push_back(profile);
                            profile.format = rs_fourcc('Y', '8', 'I', ' ');
                            all_stream_profiles.push_back(profile);
                        }
                    }
                }
            }

            set_region(stream, false);

            return all_stream_profiles;
        }

        std::vector<uint32_t> cs_device::get_frame_rates()
        {
            // FrameRate does not exist on D435e with FW 1.3.4.0
            std::vector<uint32_t> frame_rates;
            if (get_frame_rates_from_control(frame_rates))
                return frame_rates;
            else
                return std::vector<uint32_t> {30};                
        }

        bool cs_device::get_frame_rates_from_control(std::vector<uint32_t> &frame_rates)
        {
            std::string frameRateValue;
            if (!_connected_device->GetStringNodeValue("FrameRate", frameRateValue))
                return false;

            smcs::StringList frameRates;
            if (!_connected_device->GetEnumNodeValuesList("FrameRate", frameRates))
                return false;

            std::vector<uint32_t> acquisitionFrameRates;
            for (const auto& frameRate : frameRates) {
                if (!_connected_device->SetStringNodeValue("FrameRate", frameRate))
                    return false;

                double acquisitionFrameRate;
                if (_connected_device->GetFloatNodeValue("AcquisitionFrameRate", acquisitionFrameRate))
                    frame_rates.push_back(static_cast<uint32_t>(acquisitionFrameRate));
                else
                    return false;
            }

            if (!_connected_device->SetStringNodeValue("FrameRate", frameRateValue))
                return false;

            return true;
        }

        bool cs_device::is_profile_format(const smcs::IImageInfo& image_info, const stream_profile& profile)
        {
            UINT32 width = 0, height = 0, format = 0;
            image_info->GetSize(width, height);
            image_info->GetPixelType(format);
            
            return width == profile.width
                && height == profile.height
                && format == native_pixel_format_to_cs_pixel_format(profile.format);
        }

        uint32_t cs_device::cs_pixel_format_to_native_pixel_format(std::string cs_format)
        {
            uint32_t npf;
            if (cs_format.compare("YUV422") == 0)
                npf = rs_fourcc('Y', 'U', 'Y', 'V');
            else if (cs_format.compare("YUV422Packed") == 0)
                npf = rs_fourcc('U', 'Y', 'V', 'Y');
            else if (cs_format.compare("Mono16") == 0)
                npf = rs_fourcc('Z', '1', '6', ' ');
            else if (cs_format.compare("Mono8") == 0)
                npf = rs_fourcc('G', 'R', 'E', 'Y');
            else 
                throw wrong_api_call_sequence_exception("Unsuported image format!");

            return npf;
        }

        uint32_t cs_device::native_pixel_format_to_cs_pixel_format(uint32_t native_format)
        {
            if (native_format == rs_fourcc('Y', '8', 'I', ' ') || native_format == rs_fourcc('Z', '1', '6', ' '))
                return GVSP_PIX_MONO16;
            else if (native_format == rs_fourcc('G', 'R', 'E', 'Y'))
                return GVSP_PIX_MONO8;
            else if (native_format == rs_fourcc('Y', 'U', 'Y', 'V'))
                return GVSP_PIX_YUV422_YUYV_PACKED;
            else if (native_format == rs_fourcc('U', 'Y', 'V', 'Y'))
                return GVSP_PIX_YUV422_PACKED;
            else
                throw wrong_api_call_sequence_exception("Unable to map Realsense pixel format to CameraSuite pixel format!");
        }

        bool cs_device::is_option_supported(rs2_option opt, cs_stream stream)
        {
            switch (opt)
            {
                case RS2_OPTION_ASIC_TEMPERATURE:
                case RS2_OPTION_PROJECTOR_TEMPERATURE:
                    return is_temperature_supported();
                default: return false;
            }
        }

        bool cs_device::get_pu(rs2_option opt, int32_t& value, cs_stream stream)
        {
            return get_cs_param_value(opt, value, stream);
        }

        bool cs_device::set_pu(rs2_option opt, int32_t value, cs_stream stream)
        {
            return set_cs_param(opt, value, stream);
        }

        control_range cs_device::get_pu_range(rs2_option option, cs_stream stream)
        {
            // Auto controls range is trimmed to {0,1} range
            if(option == RS2_OPTION_ENABLE_AUTO_EXPOSURE || option == RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE ||
                    option == RS2_OPTION_BACKLIGHT_COMPENSATION || option == RS2_OPTION_EMITTER_ENABLED || 
                    option == RS2_OPTION_OUTPUT_TRIGGER_ENABLED || option == RS2_OPTION_SOFTWARE_TRIGGER_ALL_SENSORS ||
                    option == RS2_OPTION_AUTO_EXPOSURE_PRIORITY)
            {
                static const int32_t min = 0, max = 1, step = 1, def = 1;
                control_range range(min, max, step, def);

                return range;
            }

            //range copied from asic_and_projector_temperature_options::get_range()
            if (option == RS2_OPTION_ASIC_TEMPERATURE || option == RS2_OPTION_PROJECTOR_TEMPERATURE)
                return control_range{ -40, 125, 0, 0 };

            int32_t min, max, step, value;
            if (!get_cs_param_min(option, min, stream)) min = 0;
            if (!get_cs_param_max(option, max, stream)) max = 0;
            if (!get_cs_param_step(option, step, stream)) step = 1;
            if (!get_cs_param_value(option, value, stream)) value = 0;

            control_range range(min, max, step, value);

            return range;
        }

        bool cs_device::set_cs_param(rs2_option option, int32_t value, cs_stream stream)
        {
            switch(option)
            {
                case RS2_OPTION_WHITE_BALANCE:
                case RS2_OPTION_EXPOSURE:
                case RS2_OPTION_POWER_LINE_FREQUENCY:
                case RS2_OPTION_GAMMA:
                case RS2_OPTION_SHARPNESS:
                case RS2_OPTION_SATURATION:
                case RS2_OPTION_BRIGHTNESS:
                case RS2_OPTION_CONTRAST:
                case RS2_OPTION_GAIN:
                case RS2_OPTION_LASER_POWER:
                case RS2_OPTION_HUE:
                    return _connected_device->SetIntegerNodeValue(
                        get_cs_param_name(option, stream), 
                        round_cs_param(option, value, stream)
                    );
                case RS2_OPTION_ENABLE_AUTO_EXPOSURE:
                case RS2_OPTION_BACKLIGHT_COMPENSATION:
                case RS2_OPTION_EMITTER_ENABLED:
                case RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE:
                case RS2_OPTION_AUTO_EXPOSURE_PRIORITY:
                {
                    if (value == 1) return _connected_device->SetStringNodeValue(get_cs_param_name(option, stream), "On");
                    else if (value == 0) return _connected_device->SetStringNodeValue(get_cs_param_name(option, stream), "Off");
                }
                case RS2_OPTION_PACKET_SIZE:
                {
                    if (_is_capturing[stream])
                        throw wrong_api_call_sequence_exception("Unable to set Packet Size while streaming!");

                    auto enabled = value == 0;

                    auto node = _connected_device->GetStatisticsNode("DetectOptimalPacketSize");
                    if (node != nullptr)
                        node->SetBooleanNodeValue(enabled);

                    if (enabled)
                        return true;
                }
                case RS2_OPTION_INTER_PACKET_DELAY:
                {
                    for (auto member_stream : get_stream_group(stream)) {
                        if (!select_channel(member_stream))
                            return false;

                        if (!_connected_device->SetIntegerNodeValue(get_cs_param_name(option, member_stream), (int)value))
                            return false;
                    }
                    
                    return true;
                }
                case RS2_OPTION_OUTPUT_TRIGGER_ENABLED:
                {
                    if (!select_source(stream))
                        return false;

                    if (value == 1) return _connected_device->SetStringNodeValue(get_cs_param_name(option, stream), "VSync");
                    else if (value == 0) return _connected_device->SetStringNodeValue(get_cs_param_name(option, stream), "UserOutput1");
                }                
				case RS2_OPTION_SOFTWARE_TRIGGER:
                {
                    if (!select_source(stream))
                        return false;

                    if (!_connected_device->CommandNodeExecute("TriggerSoftware"))
                        return false;
                        
                    return true;
                }
                case RS2_OPTION_SOFTWARE_TRIGGER_ALL_SENSORS:
                {
                    if (!select_source(stream))
                        return false;

                    if (value == 1) return _connected_device->SetBooleanNodeValue("TriggerSoftwareAllSources", true);
                    else if (value == 0) return _connected_device->SetBooleanNodeValue("TriggerSoftwareAllSources", false);
                }
                case RS2_OPTION_EXT_TRIGGER_SOURCE:
                {
                    if (!select_source(stream))
                        return false;

                    std::string trigger_mode;
                    _connected_device->GetStringNodeValue("TriggerMode", trigger_mode);
                    if (trigger_mode == "Off") 
                        return false;
                    
                    if (value == 1) return _connected_device->SetStringNodeValue("TriggerSource", "Line1");
                    else if (value == 2) return _connected_device->SetStringNodeValue("TriggerSource", "Software");

                }
                case RS2_OPTION_USER_OUTPUT_LEVEL:
                {
                    if (!select_source(stream))
                        return false;

                    if (value == CS_USER_OUTPUT_LEVEL_LOW) return _connected_device->SetBooleanNodeValue("UserOutputValue", false);
                    else if (value == CS_USER_OUTPUT_LEVEL_HIGH) return _connected_device->SetBooleanNodeValue("UserOutputValue", true);
                }
                default: throw linux_backend_exception(to_string() << "no CS cid for option " << option);
            }
        }

        int32_t cs_device::round_cs_param(rs2_option option, int32_t value, cs_stream stream)
        {
            int32_t min, max, step;
            if (get_cs_param_min(option, min, stream)
                && get_cs_param_max(option, max, stream)
                && get_cs_param_step(option, step, stream)) 
            {
                auto rounded = step * std::round(value / static_cast<double>(step));
                if (rounded < min) return min;
                else if (rounded > max) return max;
                else return static_cast<int32_t>(rounded);
            }
            else
            {
                return value;
            }
        }

        std::string cs_device::get_cs_param_name(rs2_option option, cs_stream stream)
        {
            switch(option)
            {
                case RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE:
                    if (stream == CS_STREAM_COLOR) return std::string("RGB_WhiteBalanceTempAuto");
                case RS2_OPTION_WHITE_BALANCE:
                    if (stream == CS_STREAM_COLOR) return std::string("RGB_WhiteBalanceTemp");
                case RS2_OPTION_ENABLE_AUTO_EXPOSURE:
                    if (stream == CS_STREAM_COLOR) return std::string("RGB_ExposureAuto");
                    else if (stream == CS_STREAM_DEPTH) return std::string("STR_ExposureAuto");
                case RS2_OPTION_EXPOSURE:
                    if (stream == CS_STREAM_COLOR) return std::string("RGB_Exposure");
                    else if (stream == CS_STREAM_DEPTH) return std::string("STR_Exposure");
                case RS2_OPTION_BACKLIGHT_COMPENSATION:
                    if (stream == CS_STREAM_COLOR) return std::string("RGB_BacklightComp");
                case RS2_OPTION_AUTO_EXPOSURE_PRIORITY:
                    if (stream == CS_STREAM_COLOR) return std::string("RGB_LowLightComp");
                case RS2_OPTION_POWER_LINE_FREQUENCY:
                    if (stream == CS_STREAM_COLOR) return std::string("RGB_PowerLineFrequency");
                case RS2_OPTION_GAMMA:
                    if (stream == CS_STREAM_COLOR) return std::string("RGB_Gamma");
                case RS2_OPTION_SHARPNESS:
                    if (stream == CS_STREAM_COLOR) return std::string("RGB_Sharpness");
                case RS2_OPTION_SATURATION:
                    if (stream == CS_STREAM_COLOR) return std::string("RGB_Saturation");
                case RS2_OPTION_BRIGHTNESS:
                    if (stream == CS_STREAM_COLOR) return std::string("RGB_Brightness");
                case RS2_OPTION_CONTRAST:
                    if (stream == CS_STREAM_COLOR) return std::string("RGB_Contrast");
                case RS2_OPTION_GAIN:
                    if (stream == CS_STREAM_COLOR) return std::string("RGB_Gain");
                    if (stream == CS_STREAM_DEPTH) return std::string("STR_Gain");
                case RS2_OPTION_HUE:
                    if (stream == CS_STREAM_COLOR) return std::string("RGB_Hue");
                case RS2_OPTION_EMITTER_ENABLED:
                    if (stream == CS_STREAM_DEPTH) return std::string("STR_LaserEnable");
                case RS2_OPTION_LASER_POWER:
                    if (stream == CS_STREAM_DEPTH) return std::string("STR_LaserPower");
                case RS2_OPTION_INTER_PACKET_DELAY: return std::string("GevSCPD");
                case RS2_OPTION_PACKET_SIZE: return std::string("GevSCPSPacketSize");
                case RS2_OPTION_ASIC_TEMPERATURE: return std::string("IntelASIC");
                case RS2_OPTION_PROJECTOR_TEMPERATURE: return std::string("DepthModule");
                case RS2_OPTION_OUTPUT_TRIGGER_ENABLED: return std::string("LineSource");
                default: throw linux_backend_exception(to_string() << "no CS cid for option " << option);
            }
        }

        bool cs_device::get_cs_param_min(rs2_option option, int32_t &value, cs_stream stream)
        {
            smcs::StringList node_value_list;
            INT64 int_value;
            bool status;

            switch(option)
            {
                case RS2_OPTION_WHITE_BALANCE:
                case RS2_OPTION_EXPOSURE:
                case RS2_OPTION_GAMMA:
                case RS2_OPTION_SHARPNESS:
                case RS2_OPTION_SATURATION:
                case RS2_OPTION_BRIGHTNESS:
                case RS2_OPTION_CONTRAST:
                case RS2_OPTION_LASER_POWER:
                case RS2_OPTION_GAIN:
                case RS2_OPTION_HUE:
                {
                    status = _connected_device->GetIntegerNodeMin(get_cs_param_name(option, stream), int_value);
                    value = static_cast<int32_t>(int_value);
                    return status;
                }
                case RS2_OPTION_POWER_LINE_FREQUENCY:
                    status = _connected_device->GetEnumNodeValuesList(get_cs_param_name(option, stream), node_value_list);
                    value = 0;
                    return status;
                case RS2_OPTION_INTER_PACKET_DELAY:
                case RS2_OPTION_PACKET_SIZE:
                {
                    if (!select_channel(stream))
                        return false;

                    status = _connected_device->GetIntegerNodeMin(get_cs_param_name(option, stream), int_value);
                    value = static_cast<int32_t>(int_value);
                    return status;
                }
                case RS2_OPTION_OUTPUT_TRIGGER_ENABLED:
                    status = _connected_device->GetEnumNodeValuesList(get_cs_param_name(option, stream), node_value_list);
                    value = 0;
                    return status;
                default: throw linux_backend_exception(to_string() << "no CS cid for option " << option);
            }
        }

        bool cs_device::get_cs_param_max(rs2_option option, int32_t &value, cs_stream stream)
        {
            smcs::StringList node_value_list;
            INT64 int_value;
            bool status;

            switch(option)
            {
                case RS2_OPTION_WHITE_BALANCE:
                case RS2_OPTION_EXPOSURE:
                case RS2_OPTION_GAMMA:
                case RS2_OPTION_SHARPNESS:
                case RS2_OPTION_SATURATION:
                case RS2_OPTION_BRIGHTNESS:
                case RS2_OPTION_LASER_POWER:
                case RS2_OPTION_CONTRAST:
                case RS2_OPTION_GAIN:
                case RS2_OPTION_HUE:
                {
                    status = _connected_device->GetIntegerNodeMax(get_cs_param_name(option, stream), int_value);
                    value = static_cast<int32_t>(int_value);
                    return status;
                }
                case RS2_OPTION_POWER_LINE_FREQUENCY:
                    status = _connected_device->GetEnumNodeValuesList(get_cs_param_name(option, stream), node_value_list);
                    value = static_cast<int32_t>(node_value_list.size()-1);
                    return status;
                case RS2_OPTION_INTER_PACKET_DELAY:
                case RS2_OPTION_PACKET_SIZE:
                {
                    if (!select_channel(stream))
                        return false;

                    status = _connected_device->GetIntegerNodeMax(get_cs_param_name(option, stream), int_value);
                    value = static_cast<int32_t>(int_value);
                    return status;
                }
                case RS2_OPTION_OUTPUT_TRIGGER_ENABLED:
                    status = _connected_device->GetEnumNodeValuesList(get_cs_param_name(option, stream), node_value_list);
                    value = 1;
                    return status;
                default: throw linux_backend_exception(to_string() << "no CS cid for option " << option);
            }
        }

        bool cs_device::get_cs_param_step(rs2_option option, int32_t& step, cs_stream stream)
        {
            INT64 int_value;

            switch(option)
            {
                case RS2_OPTION_WHITE_BALANCE:
                case RS2_OPTION_EXPOSURE:
                case RS2_OPTION_GAMMA:
                case RS2_OPTION_SHARPNESS:
                case RS2_OPTION_SATURATION:
                case RS2_OPTION_LASER_POWER:
                case RS2_OPTION_BRIGHTNESS:
                case RS2_OPTION_CONTRAST:
                case RS2_OPTION_GAIN:
                case RS2_OPTION_HUE:
                {
                    auto result = _connected_device->GetIntegerNodeIncrement(get_cs_param_name(option, stream), int_value);
                    step = static_cast<int32_t>(int_value);
                    return result;
                }
                case RS2_OPTION_POWER_LINE_FREQUENCY:
                {
                    step = 1;
                    return true;
                }
                case RS2_OPTION_INTER_PACKET_DELAY:
                case RS2_OPTION_PACKET_SIZE:
                {
                    if (!select_channel(stream)) {
                        step = 1;
                        return true;
                    }

                    auto result = _connected_device->GetIntegerNodeIncrement(get_cs_param_name(option, stream), int_value);
                    step = static_cast<int32_t>(int_value);
                    return true;
                }
                case RS2_OPTION_OUTPUT_TRIGGER_ENABLED:
                {
                    step = 1;
                    return true;
                }
                default: throw linux_backend_exception(to_string() << "no CS cid for option " << option);
            }
        }

        bool cs_device::get_cs_param_value(rs2_option option, int32_t &value, cs_stream stream)
        {
            std::string string_value;
            INT64 int_value;
            bool status;

            switch(option)
            {
                case RS2_OPTION_WHITE_BALANCE:
                case RS2_OPTION_EXPOSURE:
                case RS2_OPTION_POWER_LINE_FREQUENCY:
                case RS2_OPTION_GAMMA:
                case RS2_OPTION_SHARPNESS:
                case RS2_OPTION_SATURATION:
                case RS2_OPTION_BRIGHTNESS:
                case RS2_OPTION_CONTRAST:
                case RS2_OPTION_LASER_POWER:
                case RS2_OPTION_GAIN:
                case RS2_OPTION_HUE:
                {
                    status = _connected_device->GetIntegerNodeValue(get_cs_param_name(option, stream), int_value);
                    value = static_cast<int32_t>(int_value);
                    return status;
                }
                case RS2_OPTION_ENABLE_AUTO_EXPOSURE:
                case RS2_OPTION_BACKLIGHT_COMPENSATION:
                case RS2_OPTION_EMITTER_ENABLED:
                case RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE:
                case RS2_OPTION_AUTO_EXPOSURE_PRIORITY:
                {
                    status = _connected_device->GetStringNodeValue(get_cs_param_name(option, stream), string_value);
                    if (string_value == "Off") value = 0;
                    else value = 1;
                    return status;
                }
                case RS2_OPTION_INTER_PACKET_DELAY:
                case RS2_OPTION_PACKET_SIZE:
                {
                    if (!select_channel(stream))
                        return false;

                    status = _connected_device->GetIntegerNodeValue(get_cs_param_name(option, stream), int_value);
                    value = static_cast<int32_t>(int_value);
                    return status;
                }
                case RS2_OPTION_ASIC_TEMPERATURE:
                case RS2_OPTION_PROJECTOR_TEMPERATURE:
                {
                    if (_connected_device->SetStringNodeValue("DeviceTemperatureSelector", get_cs_param_name(option, stream))) {
                        double temperature;
                        if (_connected_device->GetFloatNodeValue("DeviceTemperature", temperature)) {
                            value = static_cast<int32_t> (temperature);
                            return true;
                        }
                    }
                    return false;
                }
                case RS2_OPTION_OUTPUT_TRIGGER_ENABLED:
                {
                    if (!select_source(stream))
                        return false;

                    status = _connected_device->GetStringNodeValue(get_cs_param_name(option, stream), string_value);
                    if (string_value == "VSync") value = 1;
                    else value = 0;
                    return status;
                }
                case RS2_OPTION_SOFTWARE_TRIGGER:
                {
                    if (!select_source(stream))
                        return false;
                    value = 1;
                    return true;
                }
                case RS2_OPTION_SOFTWARE_TRIGGER_ALL_SENSORS:
                {
                    if (!select_source(stream))
                        return false;

                    bool trigger_all_sensors;
                    status = _connected_device->GetBooleanNodeValue("TriggerSoftwareAllSources", trigger_all_sensors);
                    if (trigger_all_sensors == false) value = 0;
                    else value = 1;
                    return status;
                }
                case RS2_OPTION_EXT_TRIGGER_SOURCE:
                {
                    if (!select_source(stream))
                        return false;

                    std::string trigger_mode, trigger_source;
                    status = _connected_device->GetStringNodeValue("TriggerMode", trigger_mode);
                    if (trigger_mode == "Off")
                    {
                        value = 1;
                        return status;
                    }
                    else
                    {
                        status = _connected_device->GetStringNodeValue("TriggerSource", trigger_source);
                        if (trigger_source == "Line1")
                            value = 1;
                        else if (trigger_source == "Software")
                            value = 2;
                        else
                            value = 1;
                        return status;
                    }
                }
                case RS2_OPTION_USER_OUTPUT_LEVEL:
                {
                    if (!select_source(stream))
                        return false;

                    bool user_output_level;

                    status = _connected_device->GetBooleanNodeValue("UserOutputValue", user_output_level);
                    if (user_output_level == false) value = CS_USER_OUTPUT_LEVEL_LOW;
                    else value = CS_USER_OUTPUT_LEVEL_HIGH;

                    return status;
                }
                default: throw linux_backend_exception(to_string() << "no CS cid for option " << option);
            }
        }

        void cs_device::start_acquisition(cs_stream stream)
        {   
            // set continuous acquisition mode
            _connected_device->SetStringNodeValue("AcquisitionMode", "Continuous");

            if (!_connected_device->CommandNodeExecute("AcquisitionStart"))
                throw wrong_api_call_sequence_exception("Unable to start acquisition!");
        }

        void cs_device::stop_acquisition(cs_stream stream)
        {
            if (!select_source(stream))
                throw wrong_api_call_sequence_exception("Unable to select source!");

            if (!_connected_device->CommandNodeExecute("AcquisitionStop"))
                throw wrong_api_call_sequence_exception("Unable to stop acquisition!");

            if (!set_region(stream, false))
                throw wrong_api_call_sequence_exception("Unable to set_region!");
        }

        bool cs_device::select_source(cs_stream stream)
        {
            auto source = get_stream_source(stream);
            if (_selected_source_initialized && (source == _selected_source))
                return true;
            _selected_source = source;
            _selected_source_initialized = true;
            return _connected_device->SetIntegerNodeValue("SourceControlSelector", source);
        }

        bool cs_device::set_source_locked(cs_stream stream, bool locked)
        {
            if (select_source(stream))
                return _connected_device->SetIntegerNodeValue("TLParamsLocked", locked ? 1 : 0);

            return false;
        }

        bool cs_device::set_region(cs_stream stream, bool enable)
        {
            if (select_region(stream))
                return _connected_device->SetStringNodeValue("RegionMode", enable ? "On" : "Off");

            return false;
        }

        bool cs_device::disable_source_regions(cs_stream stream)
        {
            if (!select_source(stream))
                return false;

            smcs::StringList regions;
            if (!_connected_device->GetEnumNodeValuesList("RegionSelector", regions))
                return false;

            for (const auto& region : regions) {
                if (!_connected_device->SetStringNodeValue("RegionSelector", region))
                    return false;
                if (!_connected_device->SetStringNodeValue("RegionMode", "Off"))
                    return false;
            }

            return true;
        }

        bool cs_device::select_region(cs_stream stream)
        {
            if (select_source(stream))
                return _connected_device->SetIntegerNodeValue("RegionSelector", get_stream_region(stream));

            return false;
        }

        bool cs_device::select_channel(cs_stream stream)
        {
            UINT32 channel;
            if (get_stream_channel(stream, channel))
                return _connected_device->SetIntegerNodeValue("GevStreamChannelSelector", static_cast<INT64>(channel));

            return false;
        }

        INT64 cs_device::get_stream_source(cs_stream stream)
        {
            switch (stream) {
            case CS_STREAM_DEPTH:
            case CS_STREAM_IR_LEFT:
            case CS_STREAM_IR_RIGHT:
                return 0;
            case CS_STREAM_COLOR:
                return 1;
            default:
                throw wrong_api_call_sequence_exception("Unknown stream ID!");
            }
        }

        INT64 cs_device::get_stream_region(cs_stream stream)
        {
            switch (stream) {
            case CS_STREAM_DEPTH:
            case CS_STREAM_COLOR:
                return 0;
            case CS_STREAM_IR_LEFT:
                return 3;
            case CS_STREAM_IR_RIGHT:
                return 2;
            default:
                throw wrong_api_call_sequence_exception("Unknown stream ID!");
            }
        }

        bool cs_device::get_stream_channel(cs_stream stream, UINT32& channel)
        {
            if (_stream_channels.find(stream) == _stream_channels.end()) {
                
                if (!select_source(stream))
                    return false;

                if (!select_region(stream))
                    return false;

                std::string region_destination;
                if (!_connected_device->GetStringNodeValue("RegionDestination", region_destination))
                    return false;

                if (!_connected_device->SetStringNodeValue("TransferSelector", region_destination))
                    return false;

                INT64 transfer_stream_channel;
                if (!_connected_device->GetIntegerNodeValue("TransferStreamChannel", transfer_stream_channel))
                    return false;

                _stream_channels[stream] = static_cast<UINT32>(transfer_stream_channel);
            }

            channel = _stream_channels[stream];
            return true;
        }

        std::vector<cs_stream> cs_device::get_stream_group(cs_stream stream)
        {
            std::vector<cs_stream> group { stream };

            switch (stream) {
            case CS_STREAM_DEPTH:
                group.push_back(CS_STREAM_IR_LEFT);
                group.push_back(CS_STREAM_IR_RIGHT);
                break;
            default:
                break;
            }

            return group;
        }

        void cs_device::stream_on(std::function<void(const notification &n)> error_handler, std::vector<cs_stream> streams, std::vector<platform::stream_profile> profiles)
        {
            if (streams.empty()) return;

            for (auto i = 0; i < streams.size(); ++i)
                set_format(profiles[i], streams[i]);

            if (!select_source(streams[0]))
                throw wrong_api_call_sequence_exception("Unable to select source!");

            // Temporary fix for D435e not setting the format correctly 
            // when programmed in a specific order.
            // Issue is reproducible when depth and left IR are selected 
            // in realsense-viewer.
            // Issue exists only in firmware 1.5.1.0 and earlier. 
            // It is fixed in later firmwares versions.
            auto left_ir = std::find(streams.begin(), streams.end(), CS_STREAM_IR_LEFT);
            if (left_ir != streams.end())
                _connected_device->SetStringNodeValue("RegionSelector", "Region3");

            stream_params_lock(streams[0]);
            start_acquisition(streams[0]);

            for (auto i = 0; i < streams.size(); ++i)
                init_stream(error_handler, streams[i]);
        }

        void cs_device::init_stream(std::function<void(const notification& n)> error_handler, cs_stream stream)
        {
            if (stream < _number_of_streams)
            {
                if (!_is_capturing[stream])
                {
                    UINT32 channel;
                    if (!get_stream_channel(stream, channel))
                        throw wrong_api_call_sequence_exception("Unable to get stream channel!");

                    _error_handler[stream] = error_handler;
                    _is_capturing[stream] = true;
                    _threads[stream] = std::unique_ptr<std::thread>(new std::thread([this, stream, channel]() { capture_loop(stream, channel); }));
                }
            }
            else throw wrong_api_call_sequence_exception("Unsuported streaming type!");
        }

        void cs_device::close(std::vector<cs_stream> streams)
        {
            if (streams.empty()) return;

            for (auto stream : streams)
                deinit_stream(stream);

            stream_params_unlock(streams[0]);
            stop_acquisition(streams[0]);

            for (auto stream : streams)
                if (!set_region(stream, false))
                    throw wrong_api_call_sequence_exception("Unable to set region");
        }

        void cs_device::deinit_stream(cs_stream stream)
        {
            if (stream < _number_of_streams)
            {
                if (_is_capturing[stream])
                {
                    _is_capturing[stream] = false;
                    _threads[stream]->join();
                    _threads[stream].reset();
                }
                if (_callbacks[stream]) _callbacks[stream] = nullptr;
            }
            else throw wrong_api_call_sequence_exception("Unsuported streaming type!");
        }

        power_state cs_device::set_power_state(power_state state)
        {
            std::lock_guard<std::mutex> lock(_power_lock);

            std::string serial;

            if (state == D0 && _power_state == D3)
            {
                _power_state = D0;
            }
            if (state == D3 && _power_state == D0)
            {
                _power_state = D3;
            }

            return _power_state;
        }

        bool cs_device::reset(void)
        {
            return _connected_device->CommandNodeExecute("DeviceReset");
        }

        std::vector<byte> cs_device::send_hwm(std::vector<byte>& buffer)
        {
            std::lock_guard<std::mutex> lock(_hwm_lock);

            //SETRGBAEROI, see enum fw_cmd ind5-private.h
            const uint8_t setrgbaeroi_opcode = 0x75;
            const uint8_t unknown_opcode = 0x00;
            const uint8_t opcode_index = 4;
            const uint8_t opcode = buffer.size() > opcode_index ? buffer[opcode_index] : unknown_opcode;
            if ((opcode == setrgbaeroi_opcode) && (buffer.size() >= (6 * sizeof(uint32_t)))) {
                uint32_t param_index = 8;
                uint32_t param1 = read_from_buffer(buffer, param_index);
                uint32_t param2 = read_from_buffer(buffer, param_index + 4);
                uint32_t param3 = read_from_buffer(buffer, param_index + 8);
                uint32_t param4 = read_from_buffer(buffer, param_index + 12);
                set_rgb_ae_roi(param1, param2, param3, param4);
                return std::vector<byte> {setrgbaeroi_opcode, 0, 0, 0};
            }
            else {
                return send_hwm_to_device(buffer);
            }
        }

        uint32_t cs_device::read_from_buffer(std::vector<byte>& buffer, uint32_t index)
        {
            uint32_t result = 0;

            for (int i = 0; i < sizeof(uint32_t); ++i) {
                result += static_cast<uint32_t>(buffer[index + i]) << (i * 8);
            }

            return result;
        }

        std::vector<byte> cs_device::send_hwm_to_device(std::vector<byte>& buffer)
        {
            UINT64 address, regLength;
            UINT32 restSize;
            UINT8 readBuffer[GVCP_WRITEMEM_MAX_COUNT];
            UINT8 resendCount = 3;
            double maxWaitTime = 5.2;
            UINT16 size = 0;
            GEV_STATUS gevStatus;
            bool status;

            std::vector<byte> out_vec;

            _connected_device->GetNode("STR_HWmTxBuffer")->GetRegisterNodeLength(regLength);
            _connected_device->GetNode("STR_HWmTxBuffer")->GetRegisterNodeAddress(address);
            _connected_device->SetMemory(address, buffer.size(), buffer.data(), &gevStatus, maxWaitTime);
            _connected_device->CommandNodeExecute("STR_HWmTxBufferSend");

            _connected_device->GetNode("STR_HWmRxBuffer")->GetRegisterNodeLength(regLength);
            restSize = static_cast<UINT32>(regLength - 4);
            _connected_device->GetNode("STR_HWmRxBuffer")->GetRegisterNodeAddress(address);

            _connected_device->CommandNodeExecute("STR_HWmRxBufferReceive");
            // Receive HWm command
            while (restSize > 0) {

                size = (restSize > GVCP_WRITEMEM_MAX_COUNT) ? GVCP_WRITEMEM_MAX_COUNT : restSize;

                int tryCount = resendCount;
                bool ok;

                do {
                    ok = true;
                    tryCount--;
                    ok = ok && _connected_device->GetMemory(address, size, (UINT8*)(&readBuffer[0]), &gevStatus, maxWaitTime);
                    if (ok) {
                        // TODO use insert or std::copy instead
                        for (UINT16 i = 0; i < size; i++) {
                            out_vec.push_back(readBuffer[i]);
                        }
                    }
                } while ((tryCount > 0) && (!ok));

                if (!ok) {
                    status = false;
                    break;
                }

                restSize -= size;
                address += size;
            }

            return out_vec;
        }

        void cs_device::set_rgb_ae_roi(uint32_t top, uint32_t bottom, uint32_t left, uint32_t right)
        {
            _connected_device->SetIntegerNodeValue("RGB_ExposureAutoROITop", top);
            _connected_device->SetIntegerNodeValue("RGB_ExposureAutoROILeft", left);
            _connected_device->SetIntegerNodeValue("RGB_ExposureAutoROIBottom", bottom);
            _connected_device->SetIntegerNodeValue("RGB_ExposureAutoROIRight", right);
            _connected_device->CommandNodeExecute("RGB_ExposureAutoROISet");
        }

        void cs_device::stream_params_lock(cs_stream stream)
        {
            if (!set_source_locked(stream, true))
                throw wrong_api_call_sequence_exception("Unable to lock source!");
        }

        void cs_device::stream_params_unlock(cs_stream stream)
        {
            if (!set_source_locked(stream, false))
                throw wrong_api_call_sequence_exception("Unable to unlock source!");
        }

        std::string cs_device::get_device_version()
        {
            return _connected_device->GetDeviceVersion();
        }

        std::string cs_device::get_ip_address()
        {
            return ip_address_to_string(_connected_device->GetIpAddress());
        }

        std::string cs_device::get_subnet_mask()
        {
            return ip_address_to_string(_connected_device->GetSubnetMask());
        }

        std::string cs_device::ip_address_to_string(uint32_t ip_address)
        {
            std::stringstream stream;
            stream << ((ip_address >> 24) & 0xFF) << "." 
                << ((ip_address >> 16) & 0xFF) << "." 
                << ((ip_address >> 8) & 0xFF) << "." 
                << ((ip_address) & 0xFF);
            return stream.str();
        }

        enum rs2_format cs_device::get_rgb_format()
        {
            return _rgb_pixel_format;
        }

        bool cs_device::is_infrared_supported()
        {
            return _infrared_supported;
        }

        bool cs_device::is_temperature_supported()
        {
            if (!_temperature_supported_checked) {
                _temperature_supported = _connected_device->GetNode("DeviceTemperatureSelector") != nullptr;
                _temperature_supported_checked = true;
            }

            return _temperature_supported;
        }

        double cs_device::get_device_timestamp_ms()
        {
            if (!_connected_device->CommandNodeExecute("GevTimestampControlLatch"))
                throw io_exception("Unable to execute GevTimestampControlLatch");

            INT64 timestamp;
            if (!_connected_device->GetIntegerNodeValue("GevTimestampValue", timestamp))
                throw io_exception("Unable to read GevTimestampValue");

            return timestamp * _timestamp_to_ms_factor;
        }

        bool cs_device::is_software_trigger_supported()
        {
            if (!_software_trigger_supported_checked) {
                _software_trigger_supported = 
                    (_connected_device->GetNode("TriggerSoftwareAllSources") != nullptr) && 
                    (_connected_device->GetNode("TriggerSoftware") != nullptr);
                _software_trigger_supported_checked = true;
            }

            return _software_trigger_supported;
        }

        void cs_device::capture_loop(cs_stream stream, UINT32 channel)
        {
            try
            {
                while(_is_capturing[stream])
                {
                    image_poll(stream, channel);
                }
            }
            catch (const std::exception& ex)
            {
                LOG_ERROR(ex.what());

                librealsense::notification n = {RS2_NOTIFICATION_CATEGORY_UNKNOWN_ERROR, 0, RS2_LOG_SEVERITY_ERROR, ex.what()};

                _error_handler[stream](n);
            }
        }

        void cs_device::image_poll(cs_stream stream, UINT32 channel)
        {
            smcs::IImageInfo image_info_ = nullptr;

            if (_connected_device.IsValid() && _connected_device->IsConnected() && _connected_device->IsOnNetwork()) {
                if (_connected_device->WaitForImage(1, channel))
                {
                    _connected_device->GetImageInfo(&image_info_, channel);

                    if (image_info_ != nullptr) {
                        if (!is_profile_format(image_info_, _profiles[stream])) {
                            _connected_device->PopImage(image_info_);
                            return;
                        }
                        auto frame_counter = image_info_->GetImageID();
                        auto timestamp_us = (uint64_t) image_info_->GetCameraTimestamp();
                        double timestamp = timestamp_us * TIMESTAMP_USEC_TO_MSEC;

                        auto im = image_info_->GetRawData();

                        UINT32 pixel_type, width, height;
                        image_info_->GetPixelType(pixel_type);
                        image_info_->GetSize(width, height);
                        auto c = GvspGetBitsPerPixel((GVSP_PIXEL_TYPES) pixel_type) / 8;
                        auto image_size = width * height * c;

                        {
                            std::lock_guard<std::mutex> lock(_stream_lock);

                            _md.header.timestamp = timestamp_us;
                            _md.payload.frame_counter = frame_counter;

                            frame_object fo{image_size, sizeof(metadata_framos_basic), im, &_md, timestamp};

                            _callbacks[stream](_profiles[stream], fo, []() {});
                        }

                        _connected_device->PopImage(image_info_);
                    }
                }
            }
            else {
                throw camera_disconnected_exception("Polling images from disconnected device!");
            }
        }

        void cs_device::probe_and_commit(stream_profile profile, frame_callback callback, cs_stream stream)
        {
            if (stream < _number_of_streams)
            {
                if (!_is_capturing[stream])
                {
                    _profiles[stream] = profile;
                    _callbacks[stream] = callback;
                }
                else {
                    throw wrong_api_call_sequence_exception("Device already streaming!");
                }
            }
            else throw wrong_api_call_sequence_exception("Unsuported streaming type!");
        }

        void cs_device::set_format(stream_profile profile, cs_stream stream)
        {
            if (!set_region(stream, true))
                throw wrong_api_call_sequence_exception("Failed to set region!");

            // Resolution is read-only on D435e with FW 1.3.4.0
            std::string new_resolution = "Res_" + std::to_string(profile.width) + "x" + std::to_string(profile.height);
            std::string old_resolution;
            if (_connected_device->GetStringNodeValue("Resolution", old_resolution) && old_resolution != new_resolution)
                if (!_connected_device->SetStringNodeValue("Resolution", new_resolution))
                    throw wrong_api_call_sequence_exception("Failed to set resolution!");

            _connected_device->GetStringNodeValue("Resolution", old_resolution);

            // FrameRate does not exist on D435e with FW 1.3.4.0
            if (_connected_device->GetNode("FrameRate") != nullptr 
                && !_connected_device->SetStringNodeValue("FrameRate", "FPS_" + std::to_string(profile.fps)))
                throw wrong_api_call_sequence_exception("Failed to set framerate!");
        }

		void cs_device::set_trigger_mode(float mode, cs_stream stream)
		{
            auto sync_mode = static_cast<int>(mode);

            bool operating_mode_supported = _connected_device->GetNode("STR_OperatingMode") != nullptr;

            select_source(stream);

            if (stream == CS_STREAM_COLOR) {
                switch (sync_mode) {
                case CS_INTERCAM_SYNC_EXTERNAL_COLOR:
                    _connected_device->SetStringNodeValue("TriggerType", "ExternalEvent");
                    _connected_device->SetStringNodeValue("TriggerMode", "On");
                    break;
                default:
                    _connected_device->SetStringNodeValue("TriggerType", "MultiCam_Sync");
                    _connected_device->SetStringNodeValue("TriggerMode", "Off");
                }
            }
            else {
                switch (sync_mode) {
                case CS_INTERCAM_SYNC_SLAVE:
                    _connected_device->SetStringNodeValue("TriggerType", "MultiCam_Sync");
                    if (operating_mode_supported) {
                        _connected_device->SetStringNodeValue("STR_OperatingMode", "Slave");
                    }
                    else {
                        _connected_device->SetStringNodeValue("LineSelector", "Line1");
                        _connected_device->SetStringNodeValue("LineSource", "UserOutput1");
                    }
                    _connected_device->SetStringNodeValue("TriggerMode", "On");
                    break;
                case CS_INTERCAM_SYNC_MASTER:
                    _connected_device->SetStringNodeValue("TriggerType", "MultiCam_Sync");
                    if (operating_mode_supported) {
                        _connected_device->SetStringNodeValue("STR_OperatingMode", "Master");
                    }
                    else {
                        _connected_device->SetStringNodeValue("LineSelector", "Line1");
                        _connected_device->SetStringNodeValue("LineSource", "VSync");
                    }
                    _connected_device->SetStringNodeValue("TriggerMode", "Off");

                    break;
                case CS_INTERCAM_SYNC_EXTERNAL:
                    _connected_device->SetStringNodeValue("TriggerType", "ExternalEvent");
                    if (!operating_mode_supported) {
                        _connected_device->SetStringNodeValue("LineSelector", "Line1");
                        _connected_device->SetStringNodeValue("LineSource", "VSync");
                    }
                    _connected_device->SetStringNodeValue("TriggerMode", "On");
                    break;
                default:
                    _connected_device->SetStringNodeValue("TriggerType", "MultiCam_Sync");
                    if (operating_mode_supported) {
                        _connected_device->SetStringNodeValue("STR_OperatingMode", "Default");
                    }
                    else {
                        _connected_device->SetStringNodeValue("LineSelector", "Line1");
                        _connected_device->SetStringNodeValue("LineSource", "UserOutput1");
                    }
                    _connected_device->SetStringNodeValue("TriggerMode", "Off");
                }
            }
		}

        float cs_device::get_trigger_mode(cs_stream stream)
        {
            select_source(stream);

            bool operating_mode_supported = _connected_device->GetNode("STR_OperatingMode") != nullptr;

            std::string trigger_type, trigger_source, line_source, trigger_mode, operating_mode;
            _connected_device->GetStringNodeValue("TriggerType", trigger_type);

            _connected_device->SetStringNodeValue("LineSelector", "Line1");
            _connected_device->GetStringNodeValue("LineSource", line_source);
            _connected_device->GetStringNodeValue("TriggerMode", trigger_mode);
            if (trigger_mode == "On") {
                _connected_device->GetStringNodeValue("TriggerSource", trigger_source);
            }
            if (operating_mode_supported) {
                _connected_device->GetStringNodeValue("STR_OperatingMode", operating_mode);
            }

            switch (stream) {
            case CS_STREAM_COLOR:
                if (trigger_type == "ExternalEvent" && (trigger_source == "Line1" || trigger_source == "Software") && trigger_mode == "On")
                    return CS_INTERCAM_SYNC_EXTERNAL_COLOR;
                else
                    return CS_INTERCAM_SYNC_DEFAULT_COLOR;
            default:
                if (operating_mode_supported) {
                    if (trigger_type == "MultiCam_Sync" && trigger_source == "Line1" && operating_mode == "Slave" && trigger_mode == "On")
                        return CS_INTERCAM_SYNC_SLAVE;
                    else if (trigger_type == "MultiCam_Sync" && operating_mode == "Master" && trigger_mode == "Off")
                        return CS_INTERCAM_SYNC_MASTER;
                    else if (trigger_type == "ExternalEvent" && (trigger_source == "Line1" || trigger_source == "Software") && trigger_mode == "On")
                        return CS_INTERCAM_SYNC_EXTERNAL;
                    else
                        return CS_INTERCAM_SYNC_DEFAULT;
                }
                else {
                    if (trigger_type == "MultiCam_Sync" && trigger_source == "Line1" && line_source == "UserOutput1" && trigger_mode == "On")
                        return CS_INTERCAM_SYNC_SLAVE;
                    else if (trigger_type == "MultiCam_Sync" && line_source == "VSync" && trigger_mode == "Off")
                        return CS_INTERCAM_SYNC_MASTER;
                    else if (trigger_type == "ExternalEvent" && (trigger_source == "Line1" || trigger_source == "Software") && (line_source == "VSync" || line_source == "UserOutput1") && trigger_mode == "On")
                        return CS_INTERCAM_SYNC_EXTERNAL;
                    else
                        return CS_INTERCAM_SYNC_DEFAULT;
                }
            }
        }

        void cs_device::update_external_trigger_mode_flag(cs_stream stream, float value)
        {
            switch (stream)
            {
            case CS_STREAM_COLOR:
                if (value == 1.f) set_device_option_sw_trigger_all_flag_sn(_device_info.serial + "_RGB", true); else set_device_option_sw_trigger_all_flag_sn(_device_info.serial + "_RGB", false);
                break;
            case CS_STREAM_DEPTH:
            case CS_STREAM_IR_LEFT:
            case CS_STREAM_IR_RIGHT:
                if (value == 3.f) set_device_option_sw_trigger_all_flag_sn(_device_info.serial + "_Stereo", true); else set_device_option_sw_trigger_all_flag_sn(_device_info.serial + "_Stereo", false);
                break;
            default: throw linux_backend_exception(to_string() << "wrong stream cid ");
            }

        }

        bool cs_device::get_intercam_mode(cs_stream stream)
        {
            switch (stream) {
            case CS_STREAM_COLOR:
                return get_device_option_sw_trigger_all_flag_sn(_device_info.serial + "_RGB");
            default:
                return get_device_option_sw_trigger_all_flag_sn(_device_info.serial + "_Stereo");;
            }
        }

        int cs_device::get_optimal_inter_packet_delay(int packet_size)
        {
            double eth_packet_size = packet_size + 38;  // 38 bytes overhead
            float ns_per_byte = 8.0;  // for 1Gbps

            double time_to_transfer_packet = (eth_packet_size * ns_per_byte) / 1000.0;  // time in us
            time_to_transfer_packet = ceil(time_to_transfer_packet + 0.5);            // round up

            return static_cast<int>(time_to_transfer_packet);
        }

        bool cs_device::inc_device_count_sn(std::string serial_num)
        {
            bool result = true;

            auto it = _cs_device_num_objects_sn.find(serial_num);
            if (it == _cs_device_num_objects_sn.end()) {    // does not exist
                _cs_device_num_objects_sn.insert({serial_num, 1});
            } 
            else {
                it->second++;
            }

            return result;
        }

        bool cs_device::dec_device_count_sn(std::string serial_num)
        {
            bool result = true;

            auto it = _cs_device_num_objects_sn.find(serial_num);
            if (it == _cs_device_num_objects_sn.end()) {    // does not exist
                result = false;
            }
            else {
                it->second--;
            }

            return result;
        }

        int cs_device::get_device_count_sn(std::string serial_num)
        {
            int devCount = -1;
            
            auto it = _cs_device_num_objects_sn.find(serial_num);
            if (it == _cs_device_num_objects_sn.end()) {    // does not exist
                devCount = -1;
            }
            else {
                devCount = it->second;
            }

            return devCount;
        }

        bool cs_device::set_device_init_flag_sn(std::string serial_num, bool set_init_flag)
        {
            bool result = true;

            auto it = _cs_device_initialized_sn.find(serial_num);
            if (it == _cs_device_initialized_sn.end()) {    // does not exist
                _cs_device_initialized_sn.insert({serial_num, set_init_flag});
            }
            else {
                it->second = set_init_flag;
            }

            return result;
        }

        bool cs_device::get_device_init_flag_sn(std::string serial_num)
        {
            bool flag = false;

            auto it = _cs_device_initialized_sn.find(serial_num);
            if (it == _cs_device_initialized_sn.end()) {    // does not exist
                flag = false;
            }
            else {
                flag = it->second;
            }

            return flag;
        }

        bool cs_device::set_device_option_sw_trigger_all_flag_sn(std::string serial_num, bool set_trigger_all_flag)
        {
            bool result = true;

            auto it = _cs_device_option_sw_trigger_all_flag_sn.find(serial_num);
            if (it == _cs_device_option_sw_trigger_all_flag_sn.end()) {    // does not exist
                _cs_device_option_sw_trigger_all_flag_sn.insert({ serial_num, set_trigger_all_flag });
            }
            else {
                it->second = set_trigger_all_flag;
            }

            return result;
        }

        bool cs_device::get_device_option_sw_trigger_all_flag_sn(std::string serial_num)
        {
            bool flag = false;

            auto it = _cs_device_option_sw_trigger_all_flag_sn.find(serial_num);
            if (it == _cs_device_option_sw_trigger_all_flag_sn.end()) {    // does not exist
                flag = false;
            }
            else {
                flag = it->second;
            }

            return flag;
        }
    }

    std::vector<uint8_t> cs_command_transfer::send_receive(const std::vector<uint8_t>& data, int, bool require_response)
    {
        if (data.size() > HW_MONITOR_BUFFER_SIZE)
        {
            LOG_ERROR("HW monitor command size is invalid");
            throw invalid_value_exception(to_string() << "Requested HW monitor command size " <<
                                                      std::dec << data.size() << " exceeds permitted limit " << HW_MONITOR_BUFFER_SIZE);
        }

        std::vector<uint8_t> transmit_buf(data);

        return static_cast<std::vector<uint8_t>>(_device->send_hwm(transmit_buf));
    }


}
