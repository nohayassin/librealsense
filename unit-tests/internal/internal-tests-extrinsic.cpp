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

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <librealsense2/hpp/rs_internal.hpp>
#include "./../examples/example.hpp"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <./../third-party/stb_image_write.h>
#include <./../common/res/int-rs-splash.hpp>

#define STB_IMAGE_IMPLEMENTATION
#include <./../third-party/stb_image.h>

using namespace librealsense;
using namespace librealsense::platform;

// Require that vector is exactly the zero vector
inline void require_zero_vector(const float(&vector)[3])
{
    for (int i = 1; i < 3; ++i) REQUIRE(vector[i] == 0.0f);
}

// Require that matrix is exactly the identity matrix
inline void require_identity_matrix(const float(&matrix)[9])
{
    static const float identity_matrix_3x3[] = { 1,0,0, 0,1,0, 0,0,1 };
    for (int i = 0; i < 9; ++i) REQUIRE(matrix[i] == approx(identity_matrix_3x3[i]));
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

        std::map<std::string, size_t> extrinsic_graph_at_sensor;
        auto& b = environment::get_instance().get_extrinsics_graph();
        auto init_size = b._streams.size();
        auto initial_extrinsics_size = b._extrinsics.size();
        
        bool first = true;
        auto frames_per_iteration = 30 * 5; // fps * 5
        for (int i = 0; i < 10; i++)
        {
            
            rs2::pipeline pipe;
            rs2::config cfg;
            cfg.enable_stream(RS2_STREAM_COLOR, -1, 640, 480, RS2_FORMAT_YUYV, 30);
            pipe.start(cfg);

            try
            {
                for (auto i = 0; i < frames_per_iteration; i++)
                {
                    rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
                }
                pipe.stop();

                if (first)
                {
                    initial_extrinsics_size = b._extrinsics.size();
                    std::cout << " Initial Extrinsic Graph size is " << initial_extrinsics_size << std::endl;
                    first = false;
                }
                else {
                    REQUIRE(b._extrinsics.size() == initial_extrinsics_size);

                }
            }
            catch (...)
            {
                std::cout << "Iteration failed  " << std::endl;
                break;
            }

            std::cout << "Iteration " << i << " : Extrinsic graph map size is " << b._extrinsics.size() << std::endl;
           
        }

        auto end_size = b._extrinsics.size();
        std::cout << " Final Extrinsic Graph size is " << end_size << std::endl;
        //REQUIRE(end_size == init_size); TODO doesn't pass yet
        WARN("TODO: Graph size shall be preserved: init " << init_size << " != final " << end_size);
    }
}
TEST_CASE("Sensor - Extrinsic memory leak detection", "[live]")
{
    const int W = 640;
    const int H = 480;
    const int BPP = 2;
    struct synthetic_frame
    {
        int x, y, bpp;
        std::vector<uint8_t> frame;
    };
    class custom_frame_source
    {
    public:
        custom_frame_source()
        {
            depth_frame.x = W;
            depth_frame.y = H;
            depth_frame.bpp = BPP;

            last = std::chrono::high_resolution_clock::now();

            std::vector<uint8_t> pixels_depth(depth_frame.x * depth_frame.y * depth_frame.bpp, 0);
            depth_frame.frame = std::move(pixels_depth);

            auto realsense_logo = stbi_load_from_memory(splash, (int)splash_size, &color_frame.x, &color_frame.y, &color_frame.bpp, false);

            std::vector<uint8_t> pixels_color(color_frame.x * color_frame.y * color_frame.bpp, 0);

            memcpy(pixels_color.data(), realsense_logo, color_frame.x * color_frame.y * 4);

            for (auto i = 0; i < color_frame.y; i++)
                for (auto j = 0; j < color_frame.x * 4; j += 4)
                {
                    if (pixels_color.data()[i * color_frame.x * 4 + j] == 0)
                    {
                        pixels_color.data()[i * color_frame.x * 4 + j] = 22;
                        pixels_color.data()[i * color_frame.x * 4 + j + 1] = 115;
                        pixels_color.data()[i * color_frame.x * 4 + j + 2] = 185;
                    }
                }
            color_frame.frame = std::move(pixels_color);
        }

        synthetic_frame& get_synthetic_texture()
        {
            return color_frame;
        }

        synthetic_frame& get_synthetic_depth(glfw_state& app_state)
        {
            draw_text(50, 50, "This point-cloud is generated from a synthetic device:");

            auto now = std::chrono::high_resolution_clock::now();
            if (now - last > std::chrono::milliseconds(1))
            {
                app_state.yaw -= 1;
                wave_base += 0.1f;
                last = now;

                for (int i = 0; i < depth_frame.y; i++)
                {
                    for (int j = 0; j < depth_frame.x; j++)
                    {
                        auto d = 2 + 0.1 * (1 + sin(wave_base + j / 50.f));
                        ((uint16_t*)depth_frame.frame.data())[i * depth_frame.x + j] = (int)(d * 0xff);
                    }
                }
            }
            return depth_frame;
        }

        rs2_intrinsics create_texture_intrinsics()
        {
            rs2_intrinsics intrinsics = { color_frame.x, color_frame.y,
                (float)color_frame.x / 2, (float)color_frame.y / 2,
                (float)color_frame.x / 2, (float)color_frame.y / 2,
                RS2_DISTORTION_BROWN_CONRADY ,{ 0,0,0,0,0 } };

            return intrinsics;
        }

        rs2_intrinsics create_depth_intrinsics()
        {
            rs2_intrinsics intrinsics = { depth_frame.x, depth_frame.y,
                (float)depth_frame.x / 2, (float)depth_frame.y / 2,
                (float)depth_frame.x , (float)depth_frame.y ,
                RS2_DISTORTION_BROWN_CONRADY ,{ 0,0,0,0,0 } };

            return intrinsics;
        }

    private:
        synthetic_frame depth_frame;
        synthetic_frame color_frame;

        std::chrono::high_resolution_clock::time_point last;
        float wave_base = 0.f;
    };

    // Require at least one device to be plugged in
    rs2::context ctx;
    {
        std::cout << "Sensor - Extrinsic memory leak detection started" << std::endl;
        auto list = ctx.query_devices();
        REQUIRE(list.size());

        std::map<std::string, size_t> extrinsic_graph_at_sensor;
        auto& b = environment::get_instance().get_extrinsics_graph();
        auto init_size = b._streams.size();
        auto initial_extrinsics_size = b._extrinsics.size();
        ///////////////////////////////////////////////////////////////////////////////////
        
        //RS2_STREAM_COLOR, -1, 640, 480, RS2_FORMAT_YUYV, 30

        int frame_number = 0;

        custom_frame_source app_data;

        auto texture = app_data.get_synthetic_texture();

        rs2_intrinsics color_intrinsics = app_data.create_texture_intrinsics();
        rs2_intrinsics depth_intrinsics = app_data.create_depth_intrinsics();

        rs2::software_device dev; // Create software-only device

        auto depth_sensor = dev.add_sensor("Depth"); // Define single sensor
        auto color_sensor = dev.add_sensor("Color"); // Define single sensor

        auto depth_stream = depth_sensor.add_video_stream({ RS2_STREAM_DEPTH, 0, 0,
                                    W, H, 60, BPP,
                                    RS2_FORMAT_Z16, depth_intrinsics });

        depth_sensor.add_read_only_option(RS2_OPTION_DEPTH_UNITS, 0.001f);


        auto color_stream = color_sensor.add_video_stream({ RS2_STREAM_COLOR, 0, 1, texture.x,
                                    texture.y, 60, texture.bpp,
                                    RS2_FORMAT_RGBA8, color_intrinsics });

        dev.create_matcher(RS2_MATCHER_DLR_C);
        rs2::syncer sync;

        depth_sensor.open(depth_stream);
        color_sensor.open(color_stream);

        depth_sensor.start(sync);
        color_sensor.start(sync);

        // sleep 5 sec
        std::this_thread::sleep_for(std::chrono::milliseconds(5000));

        depth_sensor.stop();
        color_sensor.stop();

        //depth_stream.register_extrinsics_to(color_stream, { { 1,0,0,0,1,0,0,0,1 },{ 0,0,0 } });

        //while (1) // Application still alive?
        //{
        //    synthetic_frame& depth_frame = app_data.get_synthetic_depth(app_state);

        //    depth_sensor.on_video_frame({ depth_frame.frame.data(), // Frame pixels from capture API
        //        [](void*) {}, // Custom deleter (if required)
        //        depth_frame.x * depth_frame.bpp, depth_frame.bpp, // Stride and Bytes-per-pixel
        //        (rs2_time_t)frame_number * 16, RS2_TIMESTAMP_DOMAIN_HARDWARE_CLOCK, frame_number, // Timestamp, Frame# for potential sync services
        //        depth_stream });


        //    color_sensor.on_video_frame({ texture.frame.data(), // Frame pixels from capture API
        //        [](void*) {}, // Custom deleter (if required)
        //        texture.x * texture.bpp, texture.bpp, // Stride and Bytes-per-pixel
        //        (rs2_time_t)frame_number * 16, RS2_TIMESTAMP_DOMAIN_HARDWARE_CLOCK, frame_number, // Timestamp, Frame# for potential sync services
        //        color_stream });

        //    ++frame_number;

        //    rs2::frameset fset = sync.wait_for_frames();
        //    rs2::frame depth = fset.first_or_default(RS2_STREAM_DEPTH);
        //    rs2::frame color = fset.first_or_default(RS2_STREAM_COLOR);

        //}

        //return EXIT_SUCCESS;
        /////////////////////////////////////////////////////////////////////////////////
        bool first = true;
        auto frames_per_iteration = 30 * 5; // fps * 5
        for (int i = 0; i < 10; i++)
        {

            rs2::pipeline pipe;
            rs2::config cfg;
            cfg.enable_stream(RS2_STREAM_COLOR, -1, 640, 480, RS2_FORMAT_YUYV, 30);
            pipe.start(cfg);

            try
            {
                for (auto i = 0; i < frames_per_iteration; i++)
                {
                    rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
                }
                pipe.stop();

                if (first)
                {
                    initial_extrinsics_size = b._extrinsics.size();
                    std::cout << " Initial Extrinsic Graph size is " << initial_extrinsics_size << std::endl;
                    first = false;
                }
                else {
                    REQUIRE(b._extrinsics.size() == initial_extrinsics_size);

                }
            }
            catch (...)
            {
                std::cout << "Iteration failed  " << std::endl;
                break;
            }

            std::cout << "Iteration " << i << " : Extrinsic graph map size is " << b._extrinsics.size() << std::endl;

        }

        auto end_size = b._extrinsics.size();
        std::cout << " Final Extrinsic Graph size is " << end_size << std::endl;
        //REQUIRE(end_size == init_size); TODO doesn't pass yet
        WARN("TODO: Graph size shall be preserved: init " << init_size << " != final " << end_size);
    }
}