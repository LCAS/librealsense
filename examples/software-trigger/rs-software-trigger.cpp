// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.
// Partly modified by Framos GmbH.

#include <librealsense2/rs.hpp>     // Include RealSense Cross Platform API
#include "example.hpp"              // Include short list of convenience functions for rendering
#include <map>
#include <vector>
#include <regex>
#include <librealsense2/hpp/rs_frame.hpp>
#include <librealsense2/hpp/rs_d400e.hpp>
#include "api_how_to.h"
#include "helper.h"

#ifdef __linux__
#include <termios.h>
#include <unistd.h>
#else
#include <conio.h>
#endif

using sensor_action = std::pair<std::string, std::string>;
std::vector<sensor_action> create_sensor_actions();

float getOptimalInterPacketDelay(int num_parallel_streams, int packetSize);
void updateScreenInfo(rs2::sensor sensorStereo, rs2::sensor sensorRGB, std::vector<sensor_action> sensor_actions, bool init);
int get_character_press();

int main(int argc, char* argv[]) try
{
    // Create a simple OpenGL window for rendering:
    window app(1280, 960, "CPP Software-Trigger Example");

    rs2::context                          ctx;        // Create librealsense context for managing devices

    std::map<std::string, rs2::colorizer> colorizers; // Declare map from device serial number to colorizer (utility class to convert depth data RGB colorspace)

    std::vector<rs2::pipeline>            pipelines;

    rs2::sensor sensorDepth;
    rs2::sensor sensorColor;

    rs2::pipeline_profile active_profile;
    rs2::pipeline pipe(ctx);
    rs2::device device;
    std::vector<sensor_action> sensor_actions = create_sensor_actions();

    bool choose_a_device = true;
    while (choose_a_device)
    {
        print_separator();
        //First thing, let's choose a device:
        device = how_to::get_a_realsense_device();

        //Print the device's information
        how_to::print_device_information(device);

        print_separator();

        std::string name = device.get_info(RS2_CAMERA_INFO_NAME);
        std::regex d400e_regex("FRAMOS D4[0-9][0-9]e");
        if (std::regex_search(name, d400e_regex)) {
            for (auto&& sensor : device.query_sensors()) {
                if (sensor && sensor.is<rs2::depth_stereo_sensor>()) {
                    sensor.set_option(RS2_OPTION_INTER_CAM_SYNC_MODE, 3);
                    sensor.set_option(RS2_OPTION_SOFTWARE_TRIGGER_ALL_SENSORS, 1);
                }
                else {
                    sensor.set_option(RS2_OPTION_INTER_CAM_SYNC_MODE, 1);
                }
                // adjust InterPacketDelay option on D400e camera based on PacketSize, number of cameras and number of streams
                // assumptions: 
                //  - Two D400e cameras are streaming to single NIC on PC
                //  - only depth and color streams are enabled on all cameras
                //  - PacketSize is the same on all cameras
                int numParallelStreams = 3; // (2 cameras * 2 streams) - 1
                //float packetSize = 7996;  // Manual - select this value if jumbo frames on NIC are enabled
                //float packetSize = 1500;  // Manual - select this value if jumbo frames on NIC are disabled
                float packetSize = sensor.get_option(RS2_OPTION_PACKET_SIZE);   // Automatic packet size discovery
                float interPacketDelay = getOptimalInterPacketDelay(numParallelStreams, packetSize);

                std::cout << sensor.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << ", Packet Size = " << packetSize << " InterPacketDelay = " << interPacketDelay << std::endl;
                sensor.set_option(RS2_OPTION_PACKET_SIZE, packetSize);
                sensor.set_option(RS2_OPTION_INTER_PACKET_DELAY, interPacketDelay);
            }
        }
        break;
    }

    rs2::config cfg;

    // enable only depth and infrared streams
    cfg.enable_stream(RS2_STREAM_DEPTH, -1, 640, 480, RS2_FORMAT_Z16, 30);
    cfg.enable_stream(RS2_STREAM_INFRARED, 1, 640, 480, RS2_FORMAT_Y8, 30);
    cfg.enable_stream(RS2_STREAM_INFRARED, 2, 640, 480, RS2_FORMAT_Y8, 30);
    cfg.enable_stream(RS2_STREAM_COLOR, -1, 640, 480, RS2_FORMAT_RGB8, 30);

    cfg.enable_device(device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));

    pipe.start(cfg);
    pipelines.emplace_back(pipe);
    // Map from each device's serial number to a different colorizer
    colorizers[device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER)] = rs2::colorizer();

    // We'll keep track of the last frame of each stream available to make the presentation persistent
    std::map<int, rs2::frame> render_frames;

    // get sensor from the active profile, where _is_streaming is correctly updated
    for (auto&& sensor : pipe.get_active_profile().get_device().query_sensors()) {
        if (sensor && sensor.is<rs2::depth_stereo_sensor>()) {
            sensorDepth = sensor;
            sensor.set_option(RS2_OPTION_EXT_TRIGGER_SOURCE, 2);

        }
        else {
            sensorColor = sensor;
            sensor.set_option(RS2_OPTION_EXT_TRIGGER_SOURCE, 2);
        }
    }

    updateScreenInfo(sensorDepth, sensorColor, sensor_actions, true);

    // Main app loop
    while (app)
    {

        updateScreenInfo(sensorDepth, sensorColor, sensor_actions, false);

        // Collect the new frames from all the connected devices
        std::vector<rs2::frame> new_frames;
        for (auto&& pipe : pipelines)
        {
            rs2::frameset fs;
            if (pipe.poll_for_frames(&fs))
            {
                for (const rs2::frame& f : fs)
                    new_frames.emplace_back(f);
            }
        }

        // Convert the newly-arrived frames to render-friendly format
        for (const auto& frame : new_frames)
        {
            // Get the serial number of the current frame's device
            auto serial = rs2::sensor_from_frame(frame)->get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
            // Apply the colorizer of the matching device and store the colorized frame
            render_frames[frame.get_profile().unique_id()] = colorizers[serial].process(frame);
        }

        // Present all the collected frames with openGl mosaic
        app.show(render_frames);

    }

    return EXIT_SUCCESS;
}
catch (const rs2::error& e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    while (1);
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    while (1);
    return EXIT_FAILURE;
}



// calculate optimal InterPacketDelay for D400e camera based on PacketSize and number of parallel streams
float getOptimalInterPacketDelay(int num_parallel_streams, int packetSize)
{
    float interPacketDelay = 0;
    float ethPacketSize = packetSize + 38;  // 38 bytes overhead
    float nsPerByte = 8.0;  // for 1Gbps

    float timeToTransferPacket = (ethPacketSize * nsPerByte) / 1000.0;  // time in us
    timeToTransferPacket = ceil(timeToTransferPacket + 0.5);            // round up
    interPacketDelay = timeToTransferPacket * num_parallel_streams;

    return interPacketDelay;
}

std::vector<sensor_action> create_sensor_actions()
{
    //This function creates several functions ("sensor_action") that takes a device and a sensor,
    // and perform some specific action
    return std::vector<sensor_action> {
        std::make_pair("a|A", "Change Software Trigger Mode For All Sensors "),
            std::make_pair("s|S", "Change Software Trigger Mode For Stereo Sensor Only "),
            std::make_pair("d|D", "Execute Stereo Software Trigger"),
            std::make_pair("r|R", "Execute RGB Software Trigger"),
            std::make_pair("h|H", "Help")
    };
}

void updateScreenInfo(rs2::sensor sensorStereo, rs2::sensor sensorRGB, std::vector<sensor_action> sensor_actions, bool init) {
    auto key = get_character_press();

    if (key == 'a' || key == 'A') {
        sensorStereo.set_option(RS2_OPTION_SOFTWARE_TRIGGER_ALL_SENSORS, 1);
        std::cout << "Software Trigger Forwarded To All Sensors!\n" << std::endl;
    }
    else if (key == 's' || key == 'S') {
        sensorStereo.set_option(RS2_OPTION_SOFTWARE_TRIGGER_ALL_SENSORS, 0);
        std::cout << "Software Trigger Forwarded To Stereo Sensor Only!\n" << std::endl;
    }
    else if (key == 'd' || key == 'D') {
        if (sensorStereo.supports(RS2_OPTION_SOFTWARE_TRIGGER)) {
            sensorStereo.set_option(RS2_OPTION_SOFTWARE_TRIGGER, 1);
            std::cout << "Stereo Software Trigger Executed!\n" << std::endl;
        }
    }
    else if (key == 'r' || key == 'R') {
        if (sensorRGB.supports(RS2_OPTION_SOFTWARE_TRIGGER)) {
            sensorRGB.set_option(RS2_OPTION_SOFTWARE_TRIGGER, 1);
            std::cout << "RGB Software Trigger Executed!\n" << std::endl;
        }
    }
    else if (key == 'h' || key == 'H') {
        print_separator();
        std::cout << "Choose action: \n" << std::endl;
        int i = 0;
        for (auto&& action : sensor_actions)
        {
            std::cout << action.first << " : " << action.second << std::endl;
        }
        std::cout << std::endl;
    }

    if (init) {
        print_separator();
        std::cout << "Choose action: \n" << std::endl;
        int i = 0;
        for (auto&& action : sensor_actions)
        {
            std::cout << action.first << " : " << action.second << std::endl;
        }
        std::cout << std::endl;
    }
}

int get_character_press()
{
#ifdef __linux__
    struct termios old_termios, new_termios;
    tcgetattr(STDIN_FILENO, &old_termios);
    new_termios = old_termios;
    new_termios.c_lflag &= ~(ICANON);
    tcsetattr(STDIN_FILENO, TCSANOW, &new_termios);
    int c = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &old_termios);
    return c;
#else
    int key = 0;
    if (_kbhit()) {
        key = _getch() & 255;
    }
    return key;
#endif
}