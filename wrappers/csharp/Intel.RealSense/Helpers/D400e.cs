// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020 FRAMOS GmbH.

namespace Intel.RealSense
{
    public static class D400e
    {
        public static double GetHeartbeatTime()
        {
            object err;
            return NativeMethods.rs2_d400e_get_heartbeat_time(out err);
        }

        public static void SetHeartbeatTime(double time)
        {
            object err;
            NativeMethods.rs2_d400e_set_heartbeat_time(time, out err);
        }

        public static int GetBufferCount()
        {
            object err;
            return NativeMethods.rs2_d400e_get_buffer_count(out err);
        }

        public static void SetBufferCount(int bufferCount)
        {
            object err;
            NativeMethods.rs2_d400e_set_buffer_count(bufferCount, out err);
        }
    }
}
