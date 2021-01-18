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
#define DELAY_INCREMENT_THRESHOLD 1

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

        std::vector<size_t> extrinsics_table_size;
        std::map<std::string, std::vector<size_t>> streams_delay; // map to vector to collect all data
        //std::map<size_t, size_t> delay_threshold_at_stream_type;
        std::map<std::string, size_t> delay_thresholds;
        std::map<std::string, std::vector<size_t>> frame_number;
        

        // TODO : set correct values for thresholds (take threshold of fps=6)
        delay_thresholds["color"] = 6000;
        delay_thresholds["depth"] = 6000;
        delay_thresholds["ir0"]   = 6000;
        delay_thresholds["ir1"]   = 6000;
        delay_thresholds["accel"] = 6000;
        delay_thresholds["gyro"]  = 6000;


        std::map<std::string, size_t> extrinsic_graph_at_sensor;
        
        auto frames_per_iteration = 6 * 5;
        rs2::config cfg;
        size_t cfg_size = 0;
        for (auto profile : res.second)
        {
            if (profile.fps == 200) continue; // TODO : check correct fps for IMU 
            cfg.enable_stream(profile.stream, profile.index, profile.width, profile.height, profile.format, profile.fps); // all streams in cfg
            frames_per_iteration = std::min(frames_per_iteration, profile.fps * 5);
            cfg_size += 1;
        }

        auto& b = environment::get_instance().get_extrinsics_graph();
        for (auto i = 0; i < ITERATIONS_PER_CONFIG; i++)
        {
            
            rs2::config tmp_cfg = cfg;
            rs2::pipeline pipe;
            rs2::frameset data;
            pipe.start(tmp_cfg);
           
            struct new_frames
            {
                bool color = false;
                bool depth = false;
                bool ir0 = false;
                bool ir1 = false;
                //bool accel = false;
                //bool gyro = false;
            };
            new_frames new_frames_arrival;
            try
            {
                
                // TODO : use callback for this
                for (auto i = 0; i < frames_per_iteration; i++)
                {
                    auto t1 = std::chrono::system_clock::now().time_since_epoch();
                    auto milli = std::chrono::duration_cast<std::chrono::milliseconds>(t1).count();

                    data = pipe.wait_for_frames(); // Wait for next set of frames from the camera

                    // check if frame is new according to its frame number
                    // if condition is met, it means we got a new frame
                    if (!new_frames_arrival.color && (std::find(frame_number["color"].begin(), frame_number["color"].end(), data.get_color_frame().get_frame_number()) == frame_number["color"].end()))
                    {
                        frame_number["color"].push_back(data.get_color_frame().get_frame_number());
                        streams_delay["color"].push_back(data.get_color_frame().get_frame_metadata(RS2_FRAME_METADATA_TIME_OF_ARRIVAL) - milli);
                        new_frames_arrival.color = true;
                    }
                    if (!new_frames_arrival.depth && (std::find(frame_number["depth"].begin(), frame_number["depth"].end(), data.get_depth_frame().get_frame_number()) == frame_number["depth"].end()))
                    {
                        frame_number["depth"].push_back(data.get_depth_frame().get_frame_number());
                        streams_delay["depth"].push_back(data.get_depth_frame().get_frame_metadata(RS2_FRAME_METADATA_TIME_OF_ARRIVAL) - milli);
                        new_frames_arrival.depth = true;
                    }
                    if (!new_frames_arrival.ir0 && (std::find(frame_number["ir0"].begin(), frame_number["ir0"].end(), data.get_infrared_frame(0).get_frame_number()) == frame_number["ir0"].end()))
                    {
                        frame_number["ir0"].push_back(data.get_infrared_frame(0).get_frame_number());
                        streams_delay["ir0"].push_back(data.get_infrared_frame(0).get_frame_metadata(RS2_FRAME_METADATA_TIME_OF_ARRIVAL) - milli);
                        new_frames_arrival.ir0 = true;
                    }
                    if (!new_frames_arrival.ir1 && (std::find(frame_number["ir1"].begin(), frame_number["ir1"].end(), data.get_infrared_frame(0).get_frame_number()) == frame_number["ir1"].end()))
                    {
                        frame_number["ir1"].push_back(data.get_infrared_frame(0).get_frame_number());
                        streams_delay["ir1"].push_back(data.get_infrared_frame(0).get_frame_metadata(RS2_FRAME_METADATA_TIME_OF_ARRIVAL) - milli);
                        new_frames_arrival.ir1 = true;
                    }
                    if (new_frames_arrival.color && new_frames_arrival.depth && new_frames_arrival.ir0 && new_frames_arrival.ir1) break;

                    //TODO :for ACCEL and GYRO check correct fps in rs-enumerate-device tool
                    
                }

                extrinsics_table_size.push_back(b._extrinsics.size());

                pipe.stop();
            }
            catch (...)
            {
                std::cout << "Iteration failed  " << std::endl;
                exit(EXIT_FAILURE);
            }
        }

        std::cout << "Analyzing info ..  " << std::endl;

        // the test will succeed only if all 3 conditions are met:
        // 1. extrinsics table size is perserved over iterations for each stream 
        // 2. no delay increment over iterations
        // 3. "most" iterations have time to first frame delay below a defined threshold

        static const std::string streams[] = { "color", "depth", "ir0", "ir1" };
        CAPTURE(extrinsics_table_size);
        CAPTURE(streams_delay);
        // 1. extrinsics table preserve its size over iterations
        REQUIRE(std::adjacent_find(extrinsics_table_size.begin(), extrinsics_table_size.end(), std::not_equal_to<>()) == extrinsics_table_size.end());
        // 2.  no delay increment over iterations - TODO
        for (auto i = 0; i < cfg_size; i++)
        {
            CAPTURE(streams[i]);
            auto stream = streams[i];
            auto it = streams_delay[stream].begin();
            size_t sum_first_delay = 0;
            size_t sum_last_delay = 0;
            size_t sum_first_x = 0;
            size_t sum_last_x = 0;
            int j = 0;
            size_t first_size = streams_delay[stream].size() / 2;
            size_t last_size = streams_delay[stream].size() - first_size;
            for (; j < streams_delay[stream].size() / 2; j++) {
                sum_first_delay += *(it + j) < delay_thresholds[stream];
                sum_first_x += j;
            }
            for (; j < streams_delay[stream].size(); j++) {
                sum_last_delay += *(it + j) < delay_thresholds[stream];
                sum_last_x += j;
            }

            float first_delay_avg = sum_first_delay / first_size;
            float last_delay_avg = sum_last_delay / last_size;
            float first_x_avg = sum_first_x / first_size;
            float last_x_avg = sum_last_x / last_size;

            // calc dy/dx
            auto dy = std::abs(last_delay_avg - first_delay_avg);
            auto dx = std::abs(last_x_avg - first_x_avg);
            float dy_dx = dy / dx;
            CAPTURE(dy_dx);
            REQUIRE(dy_dx < DELAY_INCREMENT_THRESHOLD); // TODO : set this threshold to fail the test when there is memory leak
        }

        // 3. "most" iterations have time to first frame delay below a defined threshold
        
        for (auto i = 0; i < cfg_size; i++)
        {
            CAPTURE(streams[i]);
            auto stream = streams[i];
            for (auto it = streams_delay[stream].begin(); it != streams_delay[stream].end(); ++it) {
                REQUIRE(*it < delay_thresholds[stream]);
            }
        }

    }
}
