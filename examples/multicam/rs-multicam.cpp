// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.
// Partly modified by Framos GmbH.

#include <librealsense2-framos/rs.hpp>     // Include RealSense Cross Platform API
#include "example.hpp"              // Include short list of convenience functions for rendering

#include <map>
#include <vector>
#include <regex>
#include <librealsense2-framos/hpp/rs_frame.hpp>
#include <librealsense2-framos/hpp/rs_d400e.hpp>



float get_optimal_inter_packet_delay(int num_parallel_streams, int packetSize);



int main(int argc, char * argv[]) try
{
    // Create a simple OpenGL window for rendering:
    window app(1280, 960, "CPP Multi-Camera Example");

    rs2::context                          ctx;        // Create librealsense context for managing devices

    std::map<std::string, rs2::colorizer> colorizers; // Declare map from device serial number to colorizer (utility class to convert depth data RGB colorspace)

    std::vector<rs2::pipeline>            pipelines;

    for (auto&& dev : ctx.query_devices()) {
        std::string name = dev.get_info(RS2_CAMERA_INFO_NAME);
        std::regex d400e_regex("FRAMOS D4[0-9][0-9]e");
        if (std::regex_search(name, d400e_regex)) {
            for (auto&& sensor : dev.query_sensors()) {
                // adjust InterPacketDelay option on D400e camera based on PacketSize, number of cameras and number of streams
                // assumptions: 
                //  - Two D400e cameras are streaming to single NIC on PC
                //  - only depth and color streams are enabled on all cameras
                //  - PacketSize is the same on all cameras
                int num_parallel_streams = 3; // (2 cameras * 2 streams) - 1
                //float packet_size = 7996;  // Manual - select this value if jumbo frames on NIC are enabled
                //float packet_size = 1500;  // Manual - select this value if jumbo frames on NIC are disabled
                float packet_size = sensor.get_option(RS2_OPTION_PACKET_SIZE);   // Automatic packet size discovery
                float inter_packet_delay = get_optimal_inter_packet_delay(num_parallel_streams, packet_size);

                std::cout << sensor.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << ", Packet Size = " << packet_size << " InterPacketDelay = " << inter_packet_delay << std::endl;
                sensor.set_option(RS2_OPTION_PACKET_SIZE, packet_size);
                sensor.set_option(RS2_OPTION_INTER_PACKET_DELAY, inter_packet_delay);
            }
        }
    }

    // Start a streaming pipe per each connected device
    for (auto&& dev : ctx.query_devices())
    {
        rs2::pipeline pipe(ctx);
        rs2::config cfg;

        // enable only depth and color streams
        cfg.enable_stream(RS2_STREAM_DEPTH, -1, 640, 480, RS2_FORMAT_Z16, 30);
        cfg.enable_stream(RS2_STREAM_COLOR, -1, 640, 480, RS2_FORMAT_RGB8, 30);

        cfg.enable_device(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));

        pipe.start(cfg);
        pipelines.emplace_back(pipe);
        // Map from each device's serial number to a different colorizer
        colorizers[dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER)] = rs2::colorizer();
    }

    // We'll keep track of the last frame of each stream available to make the presentation persistent
    std::map<int, rs2::frame> render_frames;

    // Main app loop
    while (app)
    {
        // Collect the new frames from all the connected devices
        std::vector<rs2::frame> new_frames;
        for (auto &&pipe : pipelines)
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
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception & e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}



// calculate optimal InterPacketDelay for D400e camera based on PacketSize and number of parallel streams
float get_optimal_inter_packet_delay(int num_parallel_streams, int packetSize)
{
    float inter_packet_delay = 0;
    float eth_packet_size = packetSize + 38;  // 38 bytes overhead
    float ns_per_byte = 8.0;  // for 1Gbps

    float time_to_transfer_packet = (eth_packet_size * ns_per_byte) / 1000.0;  // time in us
    time_to_transfer_packet = ceil(time_to_transfer_packet + 0.5);            // round up
    inter_packet_delay = time_to_transfer_packet * num_parallel_streams;

    return inter_packet_delay;
}
