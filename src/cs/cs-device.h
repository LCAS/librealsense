// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 FRAMOS GmbH.

#pragma once

#include "cs/cs-sensor.h"
#include "device.h"
#include "ds5/ds5-private.h"
#include "global_timestamp_reader.h"
#include "core/advanced_mode.h"
#include <map>
#include "stream.h"
#include "fw-update/fw-update-device-interface.h"

namespace librealsense
{
    class cs_auto_exposure_roi_method : public region_of_interest_method
    {
    public:
        explicit cs_auto_exposure_roi_method(const hw_monitor& hwm,
                                             ds::fw_cmd cmd = ds::fw_cmd::SETAEROI);

        void set(const region_of_interest& roi) override;
        region_of_interest get() const override;
    private:
        const ds::fw_cmd _cmd;
        const hw_monitor& _hw_monitor;
    };

    class cs_device_interface : public global_time_interface
    {
    public:
        cs_device_interface(std::shared_ptr<context> ctx,
                            const platform::backend_device_group& group);

        double get_device_time_ms() override;

    protected:
        std::shared_ptr<hw_monitor> _hw_monitor;

        std::shared_ptr<platform::cs_device> _cs_device;
        platform::cs_device_info _cs_device_info;
    };

    class cs_color : public virtual device, public virtual cs_device_interface
    {
    public:
        cs_color(std::shared_ptr<context> ctx,
                 const platform::backend_device_group& group);

        std::shared_ptr<synthetic_sensor> create_color_device(std::shared_ptr<context> ctx,
                                                              std::shared_ptr<platform::cs_device> cs_device);

		synthetic_sensor& get_color_sensor()
		{
			return dynamic_cast<synthetic_sensor&>(get_sensor(_color_device_idx));
		};
		cs_sensor& get_raw_color_sensor()
		{
			synthetic_sensor& color_sensor = get_color_sensor();
			return dynamic_cast<cs_sensor&>(*color_sensor.get_raw_sensor());
		};
        void color_init(std::shared_ptr<context> ctx, const platform::backend_device_group& group);

    protected:
        std::vector<uint8_t> get_raw_calibration_table(ds::calibration_table_id table_id) const;

        std::shared_ptr<stream_interface> _color_stream;
        uint8_t _color_device_idx;

        friend class cs_color_sensor;

        firmware_version            _fw_version;

        lazy<std::vector<uint8_t>> _color_calib_table_raw;
        std::shared_ptr<lazy<rs2_extrinsics>> _color_extrinsic;
    };

    class cs_depth : public virtual device, public debug_interface, public virtual cs_device_interface
    {
    public:
        cs_depth(std::shared_ptr<context> ctx,
                 const platform::backend_device_group& group);

        std::shared_ptr<synthetic_sensor> create_depth_device(std::shared_ptr<context> ctx,
                                                              std::shared_ptr<platform::cs_device> cs_device);

        synthetic_sensor& get_depth_sensor()
		{
			return dynamic_cast<synthetic_sensor&>(get_sensor(_depth_device_idx));
		};
        cs_sensor& get_raw_depth_sensor()
		{
			synthetic_sensor& depth_sensor = get_depth_sensor();
			return dynamic_cast<cs_sensor&>(*depth_sensor.get_raw_sensor());
		};
        std::vector<uint8_t> send_receive_raw_data(const std::vector<uint8_t>& input) override;

        void create_snapshot(std::shared_ptr<debug_interface>& snapshot) const override;
        void enable_recording(std::function<void(const debug_interface&)> record_action) override;

        void depth_init(std::shared_ptr<context> ctx, const platform::backend_device_group& group);


    protected:
        float get_stereo_baseline_mm() const;
        std::vector<uint8_t> get_raw_calibration_table(ds::calibration_table_id table_id) const;
        std::vector<uint8_t> get_new_calibration_table() const;
        bool is_camera_in_advanced_mode() const;
        processing_blocks get_cs_depth_recommended_proccesing_blocks() const;

        ds::d400_caps  parse_device_capabilities(const uint16_t pid) const;

        friend class cs_depth_sensor;

        firmware_version            _fw_version;
        firmware_version            _recommended_fw_version;
        ds::d400_caps               _device_capabilities;

        std::shared_ptr<stream_interface> _depth_stream;
        std::shared_ptr<stream_interface> _left_ir_stream;
        std::shared_ptr<stream_interface> _right_ir_stream;

        uint8_t _depth_device_idx;

        lazy<std::vector<uint8_t>> _depth_calib_table_raw;
        lazy<std::vector<uint8_t>> _new_calib_table_raw;

        std::shared_ptr<lazy<rs2_extrinsics>> _depth_extrinsic;
    };

    class cs_color_sensor : public synthetic_sensor,
                            public video_sensor_interface,
                            public roi_sensor_base,
                            public color_sensor
    {
    public:
        explicit cs_color_sensor(cs_color* owner,
                                 std::shared_ptr<cs_sensor> cs_sensor,
                                 std::map<uint32_t, rs2_format> cs_color_fourcc_to_rs2_format,
                                 std::map<uint32_t, rs2_stream> cs_color_fourcc_to_rs2_stream)
                : synthetic_sensor("RGB Camera", cs_sensor, owner, cs_color_fourcc_to_rs2_format, cs_color_fourcc_to_rs2_stream),
                  _owner(owner)
        {};
        rs2_intrinsics get_intrinsics(const stream_profile& profile) const override;
        stream_profiles init_stream_profiles() override;
        processing_blocks get_recommended_processing_blocks() const override;

    private:
        const cs_color* _owner;
    };

    class cs_depth_sensor : public synthetic_sensor,
                            public roi_sensor_base,
                            public video_sensor_interface,
                            public depth_stereo_sensor
    {
    public:
        explicit cs_depth_sensor(cs_depth* owner,
                                 std::shared_ptr<cs_sensor> cs_sensor,
                                 std::map<uint32_t, rs2_format> cs_depth_fourcc_to_rs2_format,
                                 std::map<uint32_t, rs2_stream> cs_depth_fourcc_to_rs2_stream)
                : synthetic_sensor("Stereo Module", cs_sensor, owner, cs_depth_fourcc_to_rs2_format, cs_depth_fourcc_to_rs2_stream),
                  _owner(owner),
                  _depth_units(-1)
        {};
        stream_profiles init_stream_profiles() override;
        void open(const stream_profiles& requests) override;
        rs2_intrinsics get_intrinsics(const stream_profile& profile) const override;
        processing_blocks get_recommended_processing_blocks() const override;

        float get_depth_scale() const override
        {
            if (_depth_units < 0)
                _depth_units = get_option(RS2_OPTION_DEPTH_UNITS).query();
            return _depth_units;
        };

        void set_depth_scale(float val){ _depth_units = val; };
        float get_stereo_baseline_mm() const override { return _owner->get_stereo_baseline_mm(); }
        void enable_recording(std::function<void(const depth_stereo_sensor&)> recording_function) override;
        void enable_recording(std::function<void(const depth_sensor&)> recording_function) override;
        void create_snapshot(std::shared_ptr<depth_stereo_sensor>& snapshot) const override;
        void create_snapshot(std::shared_ptr<depth_sensor>& snapshot) const override;
    private:
        const cs_depth* _owner;
        mutable std::atomic<float> _depth_units;
    };

	class cs_external_sync_mode : public option
	{
	public:
		cs_external_sync_mode(hw_monitor& hwm, cs_sensor& depth);
		virtual ~cs_external_sync_mode() = default;
		virtual void set(float value) override;
		virtual float query() const override;
		virtual option_range get_range() const override;
		virtual bool is_enabled() const override { return !_depth.is_streaming(); }

		const char* get_description() const override
		{
			return "Inter-camera synchronization mode: 0:Default, 1:Master, 2:Slave, 3:External Trigger";
		}
		void enable_recording(std::function<void(const option &)> record_action) override
		{
			_record_action = record_action;
		}
	private:
		std::function<void(const option &)> _record_action = [](const option&) {};
		lazy<option_range> _range;
		hw_monitor& _hwm;
		cs_sensor& _depth;
	};

	class cs_external_sync_mode_color : public option
	{
	public:
		cs_external_sync_mode_color(cs_sensor& color);
		virtual ~cs_external_sync_mode_color() = default;
		virtual void set(float value) override;
		virtual float query() const override;
		virtual option_range get_range() const override;
		virtual bool is_enabled() const override { return !_color.is_streaming(); }

		const char* get_description() const override
		{
			return "Inter-camera synchronization mode: 0:Default, 1:External Trigger";
		}
		void enable_recording(std::function<void(const option &)> record_action) override
		{
			_record_action = record_action;
		}
	private:
		std::function<void(const option &)> _record_action = [](const option&) {};
		lazy<option_range> _range;
		cs_sensor& _color;
	};

}

