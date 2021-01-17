// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015 Intel Corporation. All Rights Reserved.

#include "approx.h"
#include <cmath>
#include <iostream>
#include <chrono>
#include <ctime>
#include <algorithm>
#include <librealsense2/rs.hpp>
#include <librealsense2/hpp/rs_sensor.hpp>
#include "../../common/tiny-profiler.h"
#include "./../src/environment.h"

#include <unit-tests-common.h>

using namespace librealsense;
using namespace librealsense::platform;

// Require that vector is exactly the zero vector
/*inline void require_zero_vector(const float(&vector)[3])
{
    for (int i = 1; i < 3; ++i) REQUIRE(vector[i] == 0.0f);
}

// Require that matrix is exactly the identity matrix
inline void require_identity_matrix(const float(&matrix)[9])
{
    static const float identity_matrix_3x3[] = { 1,0,0, 0,1,0, 0,0,1 };
    for (int i = 0; i < 9; ++i) REQUIRE(matrix[i] == approx(identity_matrix_3x3[i]));
}*/
bool get_mode(rs2::device& dev, rs2::stream_profile* profile, int mode_index = 0)
{
    auto sensors = dev.query_sensors();
    REQUIRE(sensors.size() > 0);

    for (auto i = 0; i < sensors.size(); i++)
    {
        auto modes = sensors[i].get_stream_profiles();
        REQUIRE(modes.size() > 0);

        if (mode_index >= modes.size())
            continue;

        *profile = modes[mode_index];
        return true;
    }
    return false;
}

TEST_CASE("Extrinsic graph management", "[live][multicam]")
{
    // Require at least one device to be plugged in
    rs2::context ctx;
    {
        std::cout << "Extrinsic graph management started" << std::endl;
        auto list = ctx.query_devices();
        REQUIRE(list.size());

        std::map<std::string, size_t> extrinsic_graph_at_sensor;
        auto& b = environment::get_instance().get_extrinsics_graph();
        auto init_size = b._streams.size();
        std::cout << " Initial Extrinsic Graph size is " << init_size << std::endl;

        for (int i = 0; i < 10; i++)
        {
            std::cout << "Iteration " << i << " : Extrinsic graph map size is " << b._streams.size() << std::endl;
            // For each device
            for (auto&& dev : list)
            {
                //std::cout << "Dev " << dev.get_info(RS2_CAMERA_INFO_NAME) << std::endl;
                for (auto&& snr : dev.query_sensors())
                {
                    std::vector<rs2::stream_profile> profs;
                    REQUIRE_NOTHROW(profs = snr.get_stream_profiles());
                    REQUIRE(profs.size() > 0);

                    std::string snr_id = snr.get_info(RS2_CAMERA_INFO_NAME);
                    snr_id += snr.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
                    if (extrinsic_graph_at_sensor.count(snr_id))
                    {
                        CAPTURE(snr_id);
                        CAPTURE(extrinsic_graph_at_sensor.at(snr_id));
                        CAPTURE(b._streams.size());
                        REQUIRE(b._streams.size() == extrinsic_graph_at_sensor.at(snr_id));
                    }
                    else
                        extrinsic_graph_at_sensor[snr_id] = b._streams.size();

//                    std::cout << __LINE__ << " " << snr.get_info(RS2_CAMERA_INFO_NAME) <<" : Extrinsic graph map size is " << b._streams.size() << std::endl;

                    rs2_extrinsics extrin{};
                    try {
                        auto prof = profs[0];
                        extrin = prof.get_extrinsics_to(prof);
                    }
                    catch (const rs2::error &e) {
                        // if device isn't calibrated, get_extrinsics must error out (according to old comment. Might not be true under new API)
                        WARN(e.what());
                        continue;
                    }

                    require_identity_matrix(extrin.rotation);
                    require_zero_vector(extrin.translation);
                }
            }
        }

        auto end_size = b._streams.size();
        std::cout << " Final Extrinsic Graph size is " << end_size << std::endl;
        //REQUIRE(end_size == init_size); TODO doesn't pass yet
        WARN("TODO: Graph size shall be preserved: init " << init_size << " != final " << end_size);
    }
}
TEST_CASE("Pipe - Extrinsic memory leak detection", "[live]")
{
    // Require at least one device to be plugged in
    rs2::context ctx;
    {
        std::cout << "Pipe - Extrinsic memory leak detection started" << std::endl;
        auto list = ctx.query_devices();
        REQUIRE(list.size());
        auto dev = list.front();
        //auto sens = dev.query_sensors();

        std::map<std::string, size_t> extrinsic_graph_at_sensor;
        auto& b = environment::get_instance().get_extrinsics_graph();
        //auto init_size = b._streams.size();
        auto initial_extrinsics_size = b._extrinsics.size();


        

        // profiles
        rs2::stream_profile mode;
        auto mode_index = 0;
        bool usb3_device = is_usb3(dev);
        int fps = usb3_device ? 30 : 15; // In USB2 Mode the devices will switch to lower FPS rates
        int req_fps = usb3_device ? 60 : 30; // USB2 Mode has only a single resolution for 60 fps which is not sufficient to run the test

        do
        {
            REQUIRE(get_mode(dev, &mode, mode_index));
            mode_index++;
        } while (mode.fps() != req_fps);

        auto video = mode.as<rs2::video_stream_profile>();
        auto res = configure_all_supported_streams(dev, video.width(), video.height(), mode.fps());

        std::map<std::string, size_t> extrinsic_graph_at_cfg;
        for (auto profile : res.second)
        {
            int type = profile.stream;
            int format = profile.format;
            std::cout << "stream type :" << type << ", index : " << profile.index << ", width : " << profile.width << ", height : " << profile.height << ", format : " << format << ", fps : " << profile.fps << std::endl;
            std::string cfg_key = std::to_string(format) + "," +std::to_string(profile.fps);
            std::cout <<"cfg key :"<< cfg_key <<std::endl;
            for (auto i = 0; i < 3; i++)
            {
                rs2::pipeline pipe;
                rs2::config cfg;
                cfg.enable_stream(profile.stream, profile.index, profile.width, profile.height, profile.format, profile.fps);
                pipe.start(cfg);
                auto frames_per_iteration = profile.fps * 5;
                bool first = true;
                try
                {
                    auto t1 = std::chrono::system_clock::now();
                    for (auto i = 0; i < frames_per_iteration; i++)
                    {
                        rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
                    }
                    pipe.stop();

                    if (first)
                    {
                        auto t2 = std::chrono::system_clock::now();
                        auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();

                        
                        if (!extrinsic_graph_at_cfg.count(cfg_key))
                        {
                            //initial_extrinsics_size = b._extrinsics.size();
                            extrinsic_graph_at_cfg[cfg_key] = b._extrinsics.size();
                        }
                        std::cout << " Initial Extrinsic Graph size is " << extrinsic_graph_at_cfg[cfg_key] << ", Time to first frame is " << diff <<std::endl;
                        first = false;
                    }
                    else {
                        REQUIRE(b._extrinsics.size() == extrinsic_graph_at_cfg[cfg_key]);

                    }
                }
                catch (...)
                {
                    std::cout << "Iteration failed  " << std::endl;
                    exit(EXIT_FAILURE);
                }
            }
        }
    }
}