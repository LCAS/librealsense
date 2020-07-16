// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 FRAMOS GmbH.

#include "cs/cs-timestamp.h"

namespace librealsense {
    cs_timestamp_reader_from_metadata::cs_timestamp_reader_from_metadata(std::unique_ptr<frame_timestamp_reader> backup_timestamp_reader)
            :_backup_timestamp_reader(std::move(backup_timestamp_reader)), _has_metadata(pins), one_time_note(false)
    {
        reset();
    }

    bool cs_timestamp_reader_from_metadata::has_metadata(const std::shared_ptr<frame_interface>& frame)
    {
        std::lock_guard<std::recursive_mutex> lock(_mtx);

        auto f = std::dynamic_pointer_cast<librealsense::frame>(frame);
        if (!f)
        {
            LOG_ERROR("Frame is not valid. Failed to downcast to librealsense::frame.");
            return false;
        }
        auto md = f->additional_data.metadata_blob;
        auto mds = f->additional_data.metadata_size;

        for(uint32_t i = 0; i < mds; i++)
        {
            if(md[i] != 0)
            {
                return true;
            }
        }
        return false;
    }

    rs2_time_t cs_timestamp_reader_from_metadata::get_frame_timestamp(const std::shared_ptr<frame_interface>& frame)
    {
        std::lock_guard<std::recursive_mutex> lock(_mtx);

        auto f = std::dynamic_pointer_cast<librealsense::frame>(frame);
        if (!f)
        {
            LOG_ERROR("Frame is not valid. Failed to downcast to librealsense::frame.");
            return 0;
        }
        size_t pin_index = 0;

        if (frame->get_stream()->get_format() == RS2_FORMAT_Z16)
            pin_index = 1;

        if(!_has_metadata[pin_index])
        {
            _has_metadata[pin_index] = has_metadata(frame);
        }

        auto md = (librealsense::metadata_framos_basic*)(f->additional_data.metadata_blob.data());
        if(_has_metadata[pin_index] && md)
        {
            return (double)(md->header.timestamp)*TIMESTAMP_USEC_TO_MSEC;
        }
        else
        {
            if (!one_time_note)
            {
                LOG_WARNING("CS metadata payloads not available.");
                one_time_note = true;
            }
            return _backup_timestamp_reader->get_frame_timestamp(frame);
        }
    }

    unsigned long long cs_timestamp_reader_from_metadata::get_frame_counter(const std::shared_ptr<frame_interface>& frame) const
    {
        std::lock_guard<std::recursive_mutex> lock(_mtx);

        auto f = std::dynamic_pointer_cast<librealsense::frame>(frame);
        if (!f)
        {
            LOG_ERROR("Frame is not valid. Failed to downcast to librealsense::frame.");
            return 0;
        }
        size_t pin_index = 0;

        if (frame->get_stream()->get_format() == RS2_FORMAT_Z16)
            pin_index = 1;

        if(_has_metadata[pin_index] && f->additional_data.metadata_size > platform::cs_header_size)
        {
            auto md = (librealsense::metadata_framos_basic*)(f->additional_data.metadata_blob.data());
            if (md->capture_valid())
            {
                return md->payload.frame_counter;
            }
        }

        return _backup_timestamp_reader->get_frame_counter(frame);
    }

    void cs_timestamp_reader_from_metadata::reset()
    {
        std::lock_guard<std::recursive_mutex> lock(_mtx);
        one_time_note = false;
        for (auto i = 0; i < pins; ++i)
        {
            _has_metadata[i] = false;
        }
    }

    rs2_timestamp_domain cs_timestamp_reader_from_metadata::get_frame_timestamp_domain(const std::shared_ptr<frame_interface>& frame) const
    {
        std::lock_guard<std::recursive_mutex> lock(_mtx);
        auto pin_index = 0;
        if (frame->get_stream()->get_format() == RS2_FORMAT_Z16)
            pin_index = 1;

        return _has_metadata[pin_index] ? RS2_TIMESTAMP_DOMAIN_HARDWARE_CLOCK :
               _backup_timestamp_reader->get_frame_timestamp_domain(frame);
    }

    cs_timestamp_reader::cs_timestamp_reader(std::shared_ptr <platform::time_service> ts)
            : counter(pins), _ts(ts) 
    {
        reset();
    }

    void cs_timestamp_reader::reset() {
        std::lock_guard <std::recursive_mutex> lock(_mtx);
        for (auto i = 0; i < pins; ++i)
        {
            counter[i] = 0;
        }
    }

    rs2_time_t cs_timestamp_reader::get_frame_timestamp(const std::shared_ptr<frame_interface>& frame)
    {
        std::lock_guard <std::recursive_mutex> lock(_mtx);
        return _ts->get_time();
    }

    unsigned long long cs_timestamp_reader::get_frame_counter(const std::shared_ptr<frame_interface>& frame) const
    {
        std::lock_guard <std::recursive_mutex> lock(_mtx);
        auto pin_index = 0;
        if (frame->get_stream()->get_format() == RS2_FORMAT_Z16) // Z16
            pin_index = 1;

        return ++counter[pin_index];
    }

    rs2_timestamp_domain cs_timestamp_reader::get_frame_timestamp_domain(const std::shared_ptr<frame_interface>& frame) const
    {
        return RS2_TIMESTAMP_DOMAIN_SYSTEM_TIME;
    }
}
