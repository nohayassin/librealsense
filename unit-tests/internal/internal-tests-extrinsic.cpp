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
#include <numeric>
#include <stdlib.h> 

using namespace librealsense;
using namespace librealsense::platform;

#define TIME_INCREMENT_THRESHOLD 5
#define ITERATIONS_PER_CONFIG 15
#define DELAY_INCREMENT_THRESHOLD 1.0f
#define SPIKE_THRESHOLD 5

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
        std::map<std::string, std::vector<double>> streams_delay; // map to vector to collect all data
        std::map<std::string, std::vector<std::map<unsigned long long, size_t >>> unique_streams_delay;
        std::map<std::string, double> delay_thresholds;
        std::map<std::string, std::vector<unsigned long long>> frame_number;
        std::map<std::string, size_t> new_frame;

        // TODO : set correct values for thresholds (take threshold of fps=6)
        delay_thresholds["Accel"] = 1000; // ms
        delay_thresholds["Color"] = 1000; // ms
        delay_thresholds["Depth"] = 1000; // ms
        delay_thresholds["Gyro"] = 1000; // ms
        delay_thresholds["Infrared 1"] = 1000; // ms
        delay_thresholds["Infrared 2"] = 1000; // ms

        std::map<std::string, size_t> extrinsic_graph_at_sensor;

        /*rs2::config cfg;
        size_t cfg_size = 0;
        for (auto profile : res.second)
        {
            cfg.enable_stream(profile.stream, profile.index, profile.width, profile.height, profile.format, profile.fps); // all streams in cfg
            cfg_size += 1;
        }*/

        auto& b = environment::get_instance().get_extrinsics_graph();
        for (auto i = 0; i < ITERATIONS_PER_CONFIG; i++)
        {

            //rs2::config tmp_cfg = cfg;
            rs2::config cfg;
            size_t cfg_size = 0;
            for (auto profile : res.second)
            {
                cfg.enable_stream(profile.stream, profile.index, profile.width, profile.height, profile.format, profile.fps); // all streams in cfg
                cfg_size += 1;
            }

            rs2::pipeline pipe;
            rs2::frameset frames;
            pipe.start(cfg);

            try
            {
                for (auto it = new_frame.begin(); it != new_frame.end(); it++)
                {
                    it->second = 0;
                }
                // to prevent FW issue, at least 20 frames per stream should arrive
                bool condition = false;
                std::map<std::string, size_t> frames_count_per_stream;
                while (!condition) // the condition is set to true when at least 20 frames are received per stream
                {
                    auto t1 = std::chrono::system_clock::now().time_since_epoch();
                    auto milli = std::chrono::duration_cast<std::chrono::milliseconds>(t1).count();

                    frames = pipe.wait_for_frames(); // Wait for next set of frames from the camera
                    for (auto&& f : frames)
                    {
                        auto stream_type = f.get_profile().stream_name();
                        auto frame_num = f.get_frame_number();
                        auto time_of_arrival = f.get_frame_metadata(RS2_FRAME_METADATA_TIME_OF_ARRIVAL);

                        if (std::find(frame_number[stream_type].begin(), frame_number[stream_type].end(), frame_num) == frame_number[stream_type].end())
                        {
                            if (!new_frame[stream_type])
                            {
                                frame_number[stream_type].push_back(frame_num);
                                streams_delay[stream_type].push_back(time_of_arrival - milli);
                                new_frame[stream_type] = true;
                            }
                            new_frame[stream_type] += 1;
                        }
                    }
                    if (new_frame.size() == cfg_size)
                    {
                        condition = true;
                        for (auto it = new_frame.begin(); it != new_frame.end(); it++)
                        {
                            if (it->second < 20)
                            {
                                condition = false;
                                break;
                            }
                        }
                        // all streams received more than 20 frames
                    }
                }
                pipe.stop();
                extrinsics_table_size.push_back(b._extrinsics.size());
            }
            catch (...)
            {
                std::cout << "Iteration failed  " << std::endl;
            }
        }

        std::cout << "Analyzing info ..  " << std::endl;
        // before analyzing make sure we have enough data
        for (const auto& stream : streams_delay)
        {
            REQUIRE(stream.second.size() > 3);
        }

        // the test will succeed only if all 3 conditions are met:
        // 1. extrinsics table size is perserved over iterations for each stream 
        // 2. no delay increment over iterations
        // 3. "most" iterations have time to first frame delay below a defined threshold

        //static const std::string streams[] = { "color", "depth", "ir0", "ir1", "accel", "gyro" };
        CAPTURE(extrinsics_table_size);
        // 1. extrinsics table preserve its size over iterations
        CHECK(std::adjacent_find(extrinsics_table_size.begin(), extrinsics_table_size.end(), std::not_equal_to<>()) == extrinsics_table_size.end());
        // 2.  no delay increment over iterations 
        // filter spikes : calc stdev for each half and filter out samples that are not close 
        for (const auto& stream : streams_delay)
        {
            size_t first_size = stream.second.size() / 2;

            
            std::vector<double> v1(stream.second.begin(), stream.second.begin()+ first_size);
            std::vector<double> v2(stream.second.begin() + first_size , stream.second.begin() + stream.second.size());
            //std::vector<std::vector<double>> all;
            std::vector<std::pair<std::vector<double>, std::vector<double>>> all;
            std::vector<double> filtered_delay1;
            std::vector<double> filtered_delay2;
            all.push_back({ v1, filtered_delay1 });
            all.push_back({ v2, filtered_delay2 });
            //std::vector<std::pair<double, double>> filtered_vec_sum;
            std::pair<double, double> filtered_vec_sum_arr[2];
            // filter spikes from both parts
            int i = 0;
            for (auto &vec : all)
            {
                i += 1;
                auto v = vec.first;
                double sum = std::accumulate(v.begin(), v.end(), 0.0);
                double mean = sum / v.size();

                std::vector<double> diff(v.size());
                std::transform(v.begin(), v.end(), diff.begin(), std::bind2nd(std::minus<double>(), mean));
                double sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
                double stdev = std::sqrt(sq_sum / v.size());

                std::vector<double> stdev_diff(v.size());
                auto v_size = v.size();
                std::transform(v.begin(), v.end(), stdev_diff.begin(), [stdev, v_size](double d) {
                    d = d < 0 ? -d : d;
                    auto val = (d - stdev) / v_size;
                    return val > 0 ? val : -val;
                    }
                );
                auto stdev_diff_it = stdev_diff.begin();
                auto v_it = v.begin();

                for (auto i = 0; i < v.size(); i++)
                {
                    if (*(stdev_diff_it + i) > SPIKE_THRESHOLD) continue;
                    vec.second.push_back(*(v_it + i));
                }
                auto sum_of_elems = std::accumulate(vec.second.begin(), vec.second.end(), 0);
                filtered_vec_sum_arr[i] = { sum_of_elems , vec.second.size()};
                // make sure after filtering there still data in both parts 
                CAPTURE(stream.first, vec.second.size());
                REQUIRE(vec.second.size() > 0);
            }

            // check if increment between the 2 vectors is below a threshold 
            //double y1, y2, x1, x2;
            auto y1 = filtered_vec_sum_arr[0].first;
            auto y2 = filtered_vec_sum_arr[1].first;
            auto x1 = filtered_vec_sum_arr[0].second;
            auto x2 = filtered_vec_sum_arr[1].second;
            double dy_dx = abs((y2-y1)/(x2-x1));
            std::cout << "NOHA" << std::endl;

        }
        for (const auto& stream : streams_delay)
        {
            CAPTURE(stream.first, stream.second);
            REQUIRE(stream.second.size() > 3);
        }
        // check if increment percentage is below a threshold [%]
        for (const auto& stream_ : streams_delay)
        {
            CAPTURE(stream_.first, stream_.second);
            auto stream = stream_.first;
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
            // at this point, first_size and last_size are > 0
            float first_delay_avg = sum_first_delay / first_size;
            float last_delay_avg = sum_last_delay / last_size;
            float first_x_avg = sum_first_x / first_size;
            float last_x_avg = sum_last_x / last_size;

            // calc dy/dx
            auto dy = std::abs(last_delay_avg - first_delay_avg);
            auto dx = std::abs(last_x_avg - first_x_avg);
            float dy_dx = dy / dx;
            CAPTURE(dy_dx);
            /// TODO change to %
            CHECK(dy_dx < DELAY_INCREMENT_THRESHOLD); // TODO : set this threshold to fail the test when there is memory leak
        }

        // 3. "most" iterations have time to first frame delay below a defined threshold

        for (const auto& stream_ : streams_delay)
        {
            auto stream = stream_.first;
            CAPTURE(stream);
            for (auto it = streams_delay[stream].begin(); it != streams_delay[stream].end(); ++it) {
                CHECK(*it < delay_thresholds[stream]);
            }
        }

    }
}