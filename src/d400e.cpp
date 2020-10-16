// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020 FRAMOS GmbH.

#include "d400e.h"
#include "smcs_cpp/CameraSDK.h"

namespace librealsense
{
    namespace d400e
    {
        // class heartbeat_time

        heartbeat_time::heartbeat_time()
        {
            constexpr seconds default_heartbeat_time = 3;
            set(default_heartbeat_time);
        }

        heartbeat_time& heartbeat_time::get_instance()
        {
            static heartbeat_time instance;
            return instance;
        }

        void heartbeat_time::set(seconds time)
        {
            smcs::GetCameraAPI()->SetHeartbeatTime(time);
        }

        seconds heartbeat_time::get()
        {
            return smcs::GetCameraAPI()->GetHeartbeatTime();
        }
        
        // !class heartbeat_time

        // class device_diagnostics

        device_diagnostics::device_diagnostics()
        {
        }

        device_diagnostics& device_diagnostics::get_instance()
        {
            static device_diagnostics instance;
            return instance;
        }

        int device_diagnostics::set(const char* dev_serial, int toggle)
        {
            uint64_t dev_serial_u = 0;
            int r_status = dev_diag_status_NOK;

            if (dev_serial == nullptr) {
                return r_status;
            }

            if ((toggle != dev_diag_toggle_ON) && (toggle != dev_diag_toggle_OFF)) {
                return r_status;
            }

            try {
                const std::string& dev_serial_str = {dev_serial};
                dev_serial_u = std::stoll(dev_serial_str, 0, 16);
            }
            catch (...) {
                return r_status;
            }

            smcs::IDevice cs_device = smcs::GetCameraAPI()->GetDeviceByMac(dev_serial_u);
            if (cs_device.IsValid()) {

                if (cs_device->IsConnected()) {

                    bool b_status = false;

                    if (toggle == (int)dev_diag_toggle_ON) {
                        b_status = cs_device->SetStringNodeValue("DebugInformation", "On");
                    }
                    else if (toggle == (int)dev_diag_toggle_OFF) {
                        b_status = cs_device->SetStringNodeValue("DebugInformation", "Off");
                    }

                    if (b_status) {
                        r_status = dev_diag_status_OK;
                    }
                }
            }

            return r_status;
        }

        // !class device_diagnostics
    }
}