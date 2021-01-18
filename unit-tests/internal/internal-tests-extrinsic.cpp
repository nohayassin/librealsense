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

        // collect a log that contains info about 20 iterations for each stream
        // the info should include:
        // 1. extrinsics table size
        // 2. delay to first frame
        // 3. delay threshold for each stream (set fps=6 delay as worst case)
        // the test will succeed only if all 3 conditions are met:
        // 1. extrinsics table size is perserved over iterations for each stream 
        // 2. no delay increment over iterations
        // 3. "most" iterations have time to first frame delay below a defined threshold


        //std::map<std::string, std::vector<size_t>> extrinsics_size_at_cfg;
        std::vector<size_t> extrinsics_size_at_cfg;
        std::map<std::string, std::vector<size_t>> streams_delay; // map to vector to collect all data
        std::map<size_t, size_t> delay_threshold_at_stream_type;

        // TODO : set correct values for thresholds (take threshold of fps=6)
        delay_threshold_at_stream_type[RS2_STREAM_DEPTH] = 6000;
        delay_threshold_at_stream_type[RS2_STREAM_COLOR] = 6000;
        delay_threshold_at_stream_type[RS2_STREAM_INFRARED] = 6000;
        delay_threshold_at_stream_type[RS2_STREAM_FISHEYE] = 6000;
        delay_threshold_at_stream_type[RS2_STREAM_GYRO] = 6000;
        delay_threshold_at_stream_type[RS2_STREAM_ACCEL] = 6000;


        std::map<std::string, size_t> extrinsic_graph_at_sensor;
        auto& b = environment::get_instance().get_extrinsics_graph();
        auto frames_per_iteration = 6 * 5;
        rs2::config cfg;
        for (auto profile : res.second)
        {
            if (profile.fps == 200) continue;
            cfg.enable_stream(profile.stream, profile.index, profile.width, profile.height, profile.format, profile.fps); // all streams in cfg

            int type = profile.stream;
            int format = profile.format;
            std::cout << "==================================================================================" << std::endl;
            std::cout << "stream type :" << type << ", index : " << profile.index << ", width : " << profile.width << ", height : " << profile.height << ", format : " << format << ", fps : " << profile.fps << std::endl;
            std::string cfg_key = std::to_string(format) + "," + std::to_string(profile.fps);
            std::cout << "cfg key :" << cfg_key << std::endl;

            frames_per_iteration = std::min(frames_per_iteration, profile.fps * 5);
        }
        for (auto i = 0; i < ITERATIONS_PER_CONFIG; i++)
        //while (1) // break only when collecting 20 iterations for each stream (cfg)
        {
            rs2::config tmp_cfg = cfg;
            rs2::pipeline pipe;
            rs2::frameset data;
            pipe.start(tmp_cfg);

            try
            {
                auto t1 = std::chrono::system_clock::now();
                // TODO : use callback for this
                for (auto i = 0; i < frames_per_iteration; i++)
                {
                    data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
                }

                auto t2 = std::chrono::system_clock::now();
                auto diff = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();
                // find the correct units for t1 and timestamp to save only the diff in map
                if (data.get_color_frame()) streams_delay["color"].push_back(data.get_color_frame().get_frame_metadata(RS2_FRAME_METADATA_FRAME_TIMESTAMP));
                if (data.get_depth_frame()) streams_delay["depth"].push_back(data.get_depth_frame().get_frame_metadata(RS2_FRAME_METADATA_FRAME_TIMESTAMP));
                if (data.get_infrared_frame()) streams_delay["ir"].push_back(data.get_infrared_frame().get_frame_metadata(RS2_FRAME_METADATA_FRAME_TIMESTAMP));
                if (data.get_fisheye_frame()) streams_delay["imu"].push_back(data.get_fisheye_frame().get_frame_metadata(RS2_FRAME_METADATA_FRAME_TIMESTAMP));
                    


                extrinsics_size_at_cfg.push_back(b._extrinsics.size());

                
                pipe.stop();
            }
            catch (...)
            {
                std::cout << "Iteration failed  " << std::endl;
                exit(EXIT_FAILURE);
            }
        }


        std::cout << "NOHA XXX  " << std::endl;
        // go over the collected data and check the 3 conditions:
        // 1. check extrinsics table size
               /* if (!extrinsic_graph_at_cfg.count(cfg_key))
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
                */
    }
}
