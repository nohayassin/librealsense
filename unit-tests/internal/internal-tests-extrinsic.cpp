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
}
*/
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
TEST_CASE("D455 Frame Drops", "[live]")
{
    // Require at least one device to be plugged in

    //rs2::context ctx;
    //if (make_context(SECTION_FROM_TEST_NAME, &ctx))
    //{
    rs2::context ctx;
    {
        rs2::log_to_file(RS2_LOG_SEVERITY_DEBUG, "lrs_log.txt");

        std::cout << "D455 Frame Drops started" << std::endl;
        for (auto i = 0; i < 1; i++)
                    {
            auto list = ctx.query_devices();
            REQUIRE(list.size());
            auto dev = list.front();
            auto sensors = dev.query_sensors();


            //bool usb3_device = is_usb3(dev);
            int fps = 90; //usb3_device ? 90 : 15; // In USB2 Mode the devices will switch to lower FPS rates
            int req_fps = 90;//usb3_device ? 90 : 30; // USB2 Mode has only a single resolution for 60 fps which is not sufficient to run the test

            int width = 640;
            int height = 360;

            auto res = configure_all_supported_streams(dev, width, height, fps);
            for (auto& s : res.first) s.close();

            rs2::config cfg;
            rs2::pipeline pipe;
            size_t cfg_size = 0;
            rs2::sensor ss;

            std::vector<rs2::stream_profile> sensor_stream_profiles;
            for (auto& profile : res.second)
            {
                auto fps = profile.fps;
                if (profile.stream != RS2_STREAM_COLOR) continue;
                cfg.enable_stream(profile.stream, profile.index, profile.width, profile.height, profile.format, fps); // all streams in cfg
                cfg_size += 1;
                // create stream profiles data structure to open streams per sensor when testing in sensor mode
                for (auto& s : res.first)
                {
                    auto stream_profiles = s.get_stream_profiles();
                    for (auto& sp : stream_profiles)
                    {
                        if (sp.stream_type() != RS2_STREAM_COLOR) continue;
                        if (!(sp.stream_type() == profile.stream && sp.fps() == fps && sp.stream_index() == profile.index && sp.format() == profile.format)) continue;

                        auto vid = sp.as<rs2::video_stream_profile>();
                        auto h = vid.height();
                        auto w = vid.width();
                        if (!(w == profile.width && h == profile.height)) continue;
                        sensor_stream_profiles.push_back(sp);
                        ss = s;
                    }
                }
            }

                auto start_time = std::chrono::system_clock::now().time_since_epoch();
                auto start_time_milli = std::chrono::duration_cast<std::chrono::milliseconds>(start_time).count();
                std::mutex mutex;
                std::mutex mutex_2;
                auto process_frame = [&](const rs2::frame& f)
                {
                    auto now = std::chrono::system_clock::now().time_since_epoch();
                    auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count();
                    std::lock_guard<std::mutex> lock(mutex_2);
                    auto stream_type = f.get_profile().stream_name();
                    auto frame_num = f.get_frame_number();
                    auto time_of_arrival = f.get_frame_metadata(RS2_FRAME_METADATA_TIME_OF_ARRIVAL);
                    std::cout << "frame_num = "<< frame_num <<", time_of_arrival diff=" << diff<<std::endl;

                };
                auto frame_callback = [&](const rs2::frame& f)
                {
                    std::lock_guard<std::mutex> lock(mutex);
                    if (rs2::frameset fs = f.as<rs2::frameset>())
                    {
                        // With callbacks, all synchronized stream will arrive in a single frameset
                        for (const rs2::frame& ff : fs)
                        {
                            process_frame(ff);
                        }
                    }
                    else
                    {
                        // Stream that bypass synchronization (such as IMU) will produce single frames
                        process_frame(f);
                    }
                };

                ss.open(sensor_stream_profiles);
                ss.start(frame_callback);
                std::this_thread::sleep_for(std::chrono::seconds(2));
                ss.stop();

                ss.close();
        }
}
}

