// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2021 Intel Corporation. All Rights Reserved.

//#cmake: static!
//#test:device D400*


#define CATCH_CONFIG_MAIN
#include <stdlib.h> 
#include "../../catch.h"
#include "../../unit-tests-common.h"

using namespace rs2;
constexpr int RECEIVE_FRAMES_TIME = 5;

TEST_CASE("Syncer dynamic FPS - throughput test", "[live]")
{
    // Require at least one device to be plugged in
    rs2::context ctx;
    auto list = ctx.query_devices();
    REQUIRE(list.size());
    auto dev = list.front();
    auto sensors = dev.query_sensors();
    REQUIRE(dev.supports(RS2_CAMERA_INFO_PRODUCT_LINE));
    std::string device_type = list.front().get_info(RS2_CAMERA_INFO_PRODUCT_LINE);
    int width = 848;
    int height = 480;
    if (device_type == "L500")
        width = 640;

    sensor rgb_sensor;
    sensor ir_sensor;
    std::vector<rs2::stream_profile > rgb_stream_profile;
    std::vector<rs2::stream_profile > ir_stream_profile;

    for (auto& s : sensors)
    {
        auto info = std::string(s.get_info(RS2_CAMERA_INFO_NAME));
        auto stream_profiles = s.get_stream_profiles();
        for (auto& sp : stream_profiles)
        {
            auto vid = sp.as<rs2::video_stream_profile>();
            if (!(vid.width() == width && vid.height() == height && vid.fps() == 60)) continue;
            if (sp.stream_type() == RS2_STREAM_COLOR && sp.format() == RS2_FORMAT_RGB8)
                rgb_stream_profile.push_back(sp);
            if (sp.stream_type() == RS2_STREAM_INFRARED)
                ir_stream_profile.push_back(sp);
        }
        if (info == "RGB Camera")
            rgb_sensor = s;

        if (info == "Stereo Module")
            ir_sensor = s;
    }

    typedef enum configuration
    {
        IR_ONLY,
        IR_RGB,
        IR_RGB_EXPOSURE
    }configuration;

    configuration tests[3] = { IR_ONLY, IR_RGB, IR_RGB_EXPOSURE };
    std::map< configuration, std::map<int, std::vector<unsigned long long>>> frames_num_info;
    std::map < configuration, rs2_metadata_type> actual_fps;
    std::map<int, std::vector<unsigned long long>> curr_frames_num_info;
    rs2_metadata_type curr_actual_fps = 0;
    std::mutex mutex;
    std::map<int, std::string> stream_names;
    bool wait_for_exposure = false;

    auto process_frame = [&](const rs2::frame& f)
    {
        auto stream_type = std::string(f.get_profile().stream_name());
        auto frame_num = f.get_frame_number();
        auto unique_id = f.get_profile().unique_id();
        if (!curr_frames_num_info[unique_id].empty() && curr_frames_num_info[unique_id].back() == frame_num) // check if frame is already processed
            return;
        curr_frames_num_info[unique_id].push_back(frame_num);

        // Only IR fps is relevant for this test, IR1 and IR2 have same fps so it is enough to get only one of them
        if (!curr_actual_fps && (stream_type == "Infrared 1" || stream_type == "Infrared 2"))
            curr_actual_fps = f.get_frame_metadata(RS2_FRAME_METADATA_ACTUAL_FPS);
    };

    auto frame_callback = [&](const rs2::frame& f)
    {
        if (wait_for_exposure) // process frames only after exposure is stable
            return;
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

    for (auto& test : tests)
    {
        curr_actual_fps = 0;
        wait_for_exposure = false;

        if(test == IR_RGB_EXPOSURE)
            wait_for_exposure = true; // callback will process frames only after exposure value is set

        ir_sensor.open(ir_stream_profile); // ir streams in all configurations
        ir_sensor.start(frame_callback);
        if (test == IR_RGB || test == IR_RGB_EXPOSURE)
        {
            rgb_sensor.open(rgb_stream_profile);
            rgb_sensor.start(frame_callback);
        }
        if (test == IR_RGB_EXPOSURE)
        {
            std::this_thread::sleep_for(std::chrono::seconds(5)); // modify exposure 5 seconds after start streaming ( according to repro description )
            ir_sensor.set_option(RS2_OPTION_EXPOSURE, 18000); // set exposure value to x > 1000/fps
            std::this_thread::sleep_for(std::chrono::milliseconds(100)); // wait 100 msec to process FW command
            wait_for_exposure = false; // exposure value is set - callback will start processing frames
        }

        std::cout << "==============================================" << std::endl;
        std::cout << "Configuration " << test << std::endl << std::endl;
        curr_frames_num_info.clear();

        std::this_thread::sleep_for(std::chrono::seconds(RECEIVE_FRAMES_TIME));
        ir_sensor.stop();
        ir_sensor.close();
        if (test == IR_RGB || test == IR_RGB_EXPOSURE)
        {
            ir_sensor.set_option(RS2_OPTION_EXPOSURE, 1);
            rgb_sensor.stop();
            rgb_sensor.close();
        }

        for (auto f : curr_frames_num_info)
        {
            std::cout << stream_names[f.first] << "[" << f.first << "]: " << f.second.size() << " [frames] ||";
        }
        std::cout << std::endl;

        frames_num_info[test] = curr_frames_num_info;
        actual_fps[test] = curr_actual_fps;
    }

    // Analysis
    // Check if number of arrived frames for each stream type matches the number of
    // expected number of frames of that stream
    for (auto& test : tests)
    {
        auto expected_frames = actual_fps[test] * RECEIVE_FRAMES_TIME; // 5 seconds
        auto arrived_frames = (double)frames_num_info[test][1].size(); // IR1
        auto ir_ratio = arrived_frames / expected_frames;
        CAPTURE(test, arrived_frames, expected_frames, actual_fps[test], ir_ratio);
        CHECK(ir_ratio > 0.8);
    }
}