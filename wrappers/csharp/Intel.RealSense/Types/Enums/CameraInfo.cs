// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

namespace Intel.RealSense
{
    /// <summary>
    /// Read-only strings that can be queried from the device.
    /// </summary>
    /// <remarks>
    /// Not all information attributes are available on all camera types.
    /// This information is mainly available for camera debug and troubleshooting and should not be used in applications. */
    /// </remarks>
    public enum CameraInfo
    {
        /// <summary> Friendly name</summary>
        Name = 0,

        /// <summary> Device serial number</summary>
        SerialNumber = 1,

        /// <summary> Primary firmware version</summary>
        FirmwareVersion = 2,

        /// <summary> Secondary firmware version, added for ethernet devices</summary>
        DeviceVersion = 3,

        /// <summary> Recommended firmware version</summary>
        RecommendedFirmwareVersion = 4,

        /// <summary> Unique identifier of the port the device is connected to (platform specific)</summary>
        PhysicalPort = 5,

        /// <summary> If device supports firmware logging, this is the command to send to get logs from firmware</summary>
        DebugOpCode = 6,

        /// <summary> True iff the device is in advanced mode</summary>
        AdvancedMode = 7,

        /// <summary> Product ID as reported in the USB descriptor</summary>
        ProductId = 8,

        /// <summary> True iff EEPROM is locked</summary>
        CameraLocked = 9,

        /// <summary> Designated USB specification: USB2/USB3</summary>
        UsbTypeDescriptor = 10,

        /// <summary> Ip address, added for ethernet devices</summary>
        IpAddress = 14,

        /// <summary> Ip address, added for ethernet devices</summary>
        SubnetMask = 15,
    }
}
