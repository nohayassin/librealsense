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
    typedef enum configuration
    {
        IR_ONLY,
        IR_RGB_EXPOSURE,
        STOP
    }configuration;

    class ir_cfg
    {
    public:
        ir_cfg(sensor rgb_sensor,
            sensor ir_sensor,
            std::vector<rs2::stream_profile > rgb_stream_profile,
            std::vector<rs2::stream_profile > ir_stream_profile) : _rgb_sensor(rgb_sensor), _ir_sensor(ir_sensor), 
                                                                   _rgb_stream_profile(rgb_stream_profile), _ir_stream_profile(ir_stream_profile) {}
        void test_configuration(configuration test)
        {
            std::cout << "==============================================" << std::endl;
            std::string cfg = test == IR_ONLY ? "IR Only" : "IR + RGB";
            std::cout << "Configuration " << cfg << std::endl << std::endl;
            
            start_streaming(test);

            bool exposure_cfg[2] = { true, false };
            for (auto exposure : exposure_cfg)
            {
                if (!set_ir_exposure(exposure))
                {
                    std::cout << "ERROR : Setting Exposure value had failed!" << std::endl;
                    std::cout << "Configuration is stopped" << std::endl;
                    return;
                }
                process_all_frames(test);
            }
            stop_streaming(test);
        }

    private:
        bool set_ir_exposure(bool exposure)
        {
            _exposure = 1.0f;
            if (_ir_sensor.supports(RS2_OPTION_EXPOSURE))
            {
                if (exposure)
                {
                    REQUIRE_NOTHROW(_ir_sensor.set_option(RS2_OPTION_EXPOSURE, 18000)); // set exposure value to x > 1000/fps
                    _exposure = 180000.0f;
                }
                else
                    REQUIRE_NOTHROW(_ir_sensor.set_option(RS2_OPTION_EXPOSURE, 1)); // set exposure value to min value
                std::this_thread::sleep_for(std::chrono::milliseconds(200)); // wait 200 msec to process FW command
                return true;
            }
            return false;
        }
        void process_all_frames(configuration test)
        {
            auto process_frame = [&](const rs2::frame& f)
            {
                std::lock_guard<std::mutex> lock(_mutex);
                auto stream_type = std::string(f.get_profile().stream_name());
                // Only IR fps is relevant for this test, IR1 and IR2 have same fps so it is enough to get only one of them
                if (stream_type != "Infrared 1")
                    return;
                auto frame_num = f.get_frame_number();
                auto fps = f.get_frame_metadata(RS2_FRAME_METADATA_ACTUAL_FPS);
                if (!_frames_num_info[fps].empty() && _frames_num_info[fps].back() == frame_num) // check if frame is already processed
                    return;
                _frames_num_info[fps].push_back(frame_num);
            };
            auto t_start = std::chrono::system_clock::now();
            auto t_end = std::chrono::system_clock::now();
            float delta = (float)std::chrono::duration_cast<std::chrono::seconds>(t_end - t_start).count();
            while (delta < RECEIVE_FRAMES_TIME)
            {
                auto fs = sync.wait_for_frames();
                for (const rs2::frame& ff : fs)
                {
                    process_frame(ff);
                }
                t_end = std::chrono::system_clock::now();
                delta = (float)std::chrono::duration_cast<std::chrono::seconds>(t_end - t_start).count();
            }
            validate_ratio(delta, test);
            //bool no_drops = frame_drops();
            //CHECK(no_drops);
            //CHECK(valid_ratio);

            _frames_num_info.clear();
        }
        void start_streaming(configuration test)
        {
            _ir_sensor.set_option(RS2_OPTION_EXPOSURE, 1);
            _ir_sensor.open(_ir_stream_profile); // ir streams in all configurations
            _ir_sensor.start(sync);
            if (test == IR_RGB_EXPOSURE)
            {
                _rgb_sensor.open(_rgb_stream_profile);
                _rgb_sensor.start(sync);
            }
        }
        void stop_streaming(configuration test)
        {
            _ir_sensor.stop();
            _ir_sensor.close();
            if (test == IR_RGB_EXPOSURE)
            {
                _rgb_sensor.stop();
                _rgb_sensor.close();
            }
        }
        void validate_ratio(float delta, configuration test)
        {
            for (auto f : _frames_num_info)
            {
                std::cout << "Infrared 1 : " << f.first << " fps, " << f.second.size() << " frames, "<< _exposure<<" exposure" << std::endl;
                if (test == STOP) // after changing exposure to 18000, fps should be changed 60 -> 30
                {
                    float ratio = (float)f.first / prev_fps;
                    REQUIRE(ratio < 0.6);
                }
                auto actual_fps = f.first;
                auto arrived_frames = f.second.size();
                float calc_fps = (float)f.second.size() / delta;
                float fps_ratio = calc_fps / f.first;
                CAPTURE(calc_fps, arrived_frames, delta, actual_fps);
                REQUIRE(fps_ratio > 0.8);
                prev_fps = actual_fps;
            }
        }
        //bool frame_drops();

        sensor _rgb_sensor;
        sensor _ir_sensor;
        std::vector<rs2::stream_profile > _rgb_stream_profile;
        std::vector<rs2::stream_profile > _ir_stream_profile;
        std::map<long long, std::vector<unsigned long long>> _frames_num_info; // {fps: frames}
        std::map < configuration, rs2_metadata_type> _actual_fps;
        float _exposure;
        //std::map<int, std::string> _stream_names;
        std::mutex _mutex;
        rs2::syncer sync;
        float prev_fps = 0;
    };

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

    // extract IR/RGB sensors
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

    // test configurations
    configuration tests[2] = { IR_ONLY, IR_RGB_EXPOSURE }; // {cfg, exposure}
    for (auto& test : tests)
    {
        CAPTURE(test);
        ir_cfg test_cfg(rgb_sensor, ir_sensor, rgb_stream_profile, ir_stream_profile);
        test_cfg.test_configuration(test);
    }
}
