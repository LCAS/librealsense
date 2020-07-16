// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 FRAMOS GmbH.

#pragma once

#include "cs-private.h"

#include "backend.h"
#include "types.h"
#include "option.h"

namespace librealsense
{
    class cs_asic_and_projector_temperature_options : public readonly_option
    {
    public:
        float query() const override;

        option_range get_range() const override;

        bool is_enabled() const override;

        const char* get_description() const override;

        explicit cs_asic_and_projector_temperature_options(cs_sensor& ep, rs2_option opt, cs_stream stream);

    private:
        cs_sensor&                  _ep;
        rs2_option                  _option;
        cs_stream                   _stream;
    };

    class cs_depth_scale_option : public option, public observable_option
    {
    public:
        cs_depth_scale_option(hw_monitor& hwm);
        virtual ~cs_depth_scale_option() = default;
        virtual void set(float value) override;
        virtual float query() const override;
        virtual option_range get_range() const override;
        virtual bool is_enabled() const override { return true; }

        const char* get_description() const override
        {
            return "Number of meters represented by a single depth unit";
        }
        void enable_recording(std::function<void(const option &)> record_action)
        {
            _record_action = record_action;
        }

    private:
        cs::depth_table_control get_depth_table(cs::advanced_query_mode mode) const;
        std::function<void(const option &)> _record_action = [](const option&) {};
        lazy<option_range> _range;
        hw_monitor& _hwm;
    };

    class cs_pu_option : public option
    {
    public:
        void set(float value) override;

        float query() const override;

        option_range get_range() const override;

        bool is_enabled() const override
        {
            if (_id == RS2_OPTION_PACKET_SIZE) {
                return !_ep.is_streaming();
            }
            else {
                return true;
            }
        }

        cs_pu_option(cs_sensor& ep, rs2_option id, cs_stream stream)
                : _ep(ep), _id(id), _stream(stream)
        {
        }

        cs_pu_option(cs_sensor& ep, rs2_option id, cs_stream stream, const std::map<float, std::string>& description_per_value)
                : _ep(ep), _id(id), _stream(stream), _description_per_value(description_per_value)
        {
        }

        virtual ~cs_pu_option() = default;

        const char* get_description() const override;

        const char* get_value_description(float val) const override
        {
            if (_description_per_value.find(val) != _description_per_value.end())
                return _description_per_value.at(val).c_str();
            return nullptr;
        }
        void enable_recording(std::function<void(const option &)> record_action) override
        {
            _record = record_action;
        }

    private:
        cs_stream _stream;
        rs2_option _id;
        const std::map<float, std::string> _description_per_value;
        std::function<void(const option &)> _record = [](const option &) {};

    protected:
        cs_sensor& _ep;
    };

    class cs_depth_exposure_option : public cs_pu_option
    {
    public:
        cs_depth_exposure_option(cs_sensor& ep, rs2_option id, cs_stream stream)
                : cs_pu_option(ep, id, stream) {}
        const char* get_description() const override { return "Depth Exposure (usec)"; }
    };

    class cs_external_trigger_option : public cs_pu_option
    {
    public:
        cs_external_trigger_option(cs_sensor& ep, rs2_option id, cs_stream stream)
            : cs_pu_option(ep, id, stream) {}

        cs_external_trigger_option(cs_sensor& ep, rs2_option id, cs_stream stream, const std::map<float, std::string>& description_per_value)
            : cs_pu_option(ep, id, stream, description_per_value), _stream(stream), _id(id)
        {
        }

        option_range get_range() const override { return option_range{ 1, 2, 1, 1 }; };

        const char* get_description() const override { return "External Trigger Source"; }
    private:
        cs_stream _stream;
        rs2_option _id;
    };

    class cs_readonly_option : public cs_pu_option
    {
    public:
        bool is_read_only() const override { return true; }

        void set(float) override
        {
            throw not_implemented_exception("This option is read-only!");
        }

        bool is_enabled() const override
        {
            return _ep.is_streaming();
        }

        void enable_recording(std::function<void(const option &)> record_action) override
        {
            //empty
        }

        explicit cs_readonly_option(cs_sensor& ep, rs2_option id, cs_stream stream)
                : cs_pu_option(ep, id, stream) {}
    };

    class cs_software_trigger_option : public cs_pu_option
    {
    public:
        cs_software_trigger_option(cs_sensor& ep, rs2_option id, cs_stream stream)
            : cs_pu_option(ep, id, stream) {}

        cs_software_trigger_option(cs_sensor& ep, rs2_option id, cs_stream stream, const std::map<float, std::string>& description_per_value)
            : cs_pu_option(ep, id, stream, description_per_value), _stream(stream), _id(id) {}

        option_range get_range() const override { return option_range{ 1,1,1,1 }; };

        const char* get_description() const override { return "Software Trigger"; }
    private:
        cs_stream _stream;
        rs2_option _id;
    };

    class cs_software_trigger_all_option : public cs_pu_option
    {
    public:
        cs_software_trigger_all_option(cs_sensor& ep, rs2_option id, cs_stream stream)
            : cs_pu_option(ep, id, stream), _stream(stream), _id(id) {}

        const char* get_description() const override { return "Forwards software trigger signal to all sensors"; }
    private:
        cs_stream _stream;
        rs2_option _id;
    };

    class cs_user_output_level_option : public cs_pu_option
    {
    public:
        cs_user_output_level_option(cs_sensor& ep, rs2_option id, cs_stream stream)
            : cs_pu_option(ep, id, stream) {}

        cs_user_output_level_option(cs_sensor& ep, rs2_option id, cs_stream stream, const std::map<float, std::string>& description_per_value)
            : cs_pu_option(ep, id, stream, description_per_value), _stream(stream), _id(id) {}

        option_range get_range() const override { return option_range{ 1,2,1,1 }; };

        const char* get_description() const override { return "User Output Level"; }
    private:
        cs_stream _stream;
        rs2_option _id;
    };
};