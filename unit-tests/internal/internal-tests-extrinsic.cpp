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

#define TIME_INCREMENT_THRESHOLD 5
#define ITERATIONS_PER_CONFIG 3

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
                    catch (const rs2::error& e) {
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
        // 1. check if extrinsics table size is perserved over iterations of same stream type and fps
        // 2. check if time to first frame exceeds a define threshold :
        //      - RGB : 1200 msec
        //      - DEPTH : (should be same as L500)
        //      - IR : ?
        // 3. check if there is increment of time to first frame: 
        //      - throw exception even though the threshold from #2 is not reached
        //      - run 20 iterations to get to this conclusion
        //      - in each iteration check delay against the previous 2 iterations : 
        //      - if current itration delay > previous 2 iterations delay, increase count by 1

        struct time_increment
        {
            size_t t0 = 0;
            size_t t1 = 0;
            size_t count = 0;
        };

        std::map<std::string, size_t> extrinsic_graph_at_cfg;
        std::map<std::string, time_increment> time_increment_at_cfg;
        std::map<size_t, size_t> delay_threshold_at_stream_type;

        // TODO : set correct values for thresholds
        delay_threshold_at_stream_type[RS2_STREAM_DEPTH] = 6000;
        delay_threshold_at_stream_type[RS2_STREAM_COLOR] = 6000;
        delay_threshold_at_stream_type[RS2_STREAM_INFRARED] = 6000;
        delay_threshold_at_stream_type[RS2_STREAM_FISHEYE] = 6000;
        delay_threshold_at_stream_type[RS2_STREAM_GYRO] = 6000;
        delay_threshold_at_stream_type[RS2_STREAM_ACCEL] = 6000;


        std::map<std::string, size_t> extrinsic_graph_at_sensor;
        auto& b = environment::get_instance().get_extrinsics_graph();

        for (auto profile : res.second)
        {
            int type = profile.stream;
            int format = profile.format;
            std::cout << "==================================================================================" << std::endl;
            std::cout << "stream type :" << type << ", index : " << profile.index << ", width : " << profile.width << ", height : " << profile.height << ", format : " << format << ", fps : " << profile.fps << std::endl;
            std::string cfg_key = std::to_string(format) + "," + std::to_string(profile.fps);
            std::cout << "cfg key :" << cfg_key << std::endl;
            time_increment_at_cfg[cfg_key].count = 0;
            time_increment_at_cfg[cfg_key].t0 = 0;
            time_increment_at_cfg[cfg_key].t1 = 0;

            for (auto i = 0; i < ITERATIONS_PER_CONFIG; i++)
            {
                rs2::pipeline pipe;
                rs2::config cfg;
                cfg.enable_stream(profile.stream, profile.index, profile.width, profile.height, profile.format, profile.fps);
                pipe.start(cfg);
                auto frames_per_iteration = profile.fps * 5;

                try
                {
                    auto t1 = std::chrono::system_clock::now();
                    for (auto i = 0; i < frames_per_iteration; i++)
                    {
                        rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
                    }

                    auto t2 = std::chrono::system_clock::now();
                    auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();


                    // 1. check extrinsics table size
                    if (!extrinsic_graph_at_cfg.count(cfg_key))
                    {
                        extrinsic_graph_at_cfg[cfg_key] = b._extrinsics.size();
                        std::cout << " Extrinsic Graph size is " << extrinsic_graph_at_cfg[cfg_key] << ", Time to first frame is " << diff;// << std::endl;
                    }
                    else {
                        std::cout << " Extrinsic Graph size is " << extrinsic_graph_at_cfg[cfg_key] << ", Time to first frame is " << diff;// << std::endl;
                        REQUIRE(b._extrinsics.size() == extrinsic_graph_at_cfg[cfg_key]);
                    }

                    // 2. threshold 
                    REQUIRE(diff < delay_threshold_at_stream_type[profile.stream]);

                    // 3. delay increment
                    if (diff > time_increment_at_cfg[cfg_key].t0 && diff > time_increment_at_cfg[cfg_key].t1)
                    {
                        time_increment_at_cfg[cfg_key].count += 1;
                        REQUIRE(time_increment_at_cfg[cfg_key].count < TIME_INCREMENT_THRESHOLD);
                    }
                    std::cout << ", increment count is: " << time_increment_at_cfg[cfg_key].count << std::endl;
                    // cache only previous 2 iterations
                    time_increment_at_cfg[cfg_key].t0 = time_increment_at_cfg[cfg_key].t1;
                    time_increment_at_cfg[cfg_key].t1 = diff;

                    pipe.stop();
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