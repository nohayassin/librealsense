// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include "example.hpp"          // Include short list of convenience functions for rendering

// Capture Example demonstrates how to
// capture depth and color video streams and render them to the screen
int main(int argc, char * argv[]) try
{
    rs2::log_to_console(RS2_LOG_SEVERITY_ERROR);
    // Create a simple OpenGL window for rendering:
    window app(1280, 720, "RealSense Capture Example");

    // Declare depth colorizer for pretty visualization of depth data
    rs2::colorizer color_map;
    // Declare rates printer for showing streaming rates of the enabled streams.
    rs2::rates_printer printer;

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    

    // Start streaming with default recommended configuration
    // The default video configuration contains Depth and Color streams
    // If a device is capable to stream IMU data, both Gyro and Accelerometer are enabled by default
    
    std::vector<rs2::config> cfgs;
    auto cfgs_arr = new rs2::config[30];
    /*for (auto i = 0; i < 5; i++)
    {
        cfgs_arr[i].enable_stream(RS2_STREAM_COLOR, -1, 640, 360, RS2_FORMAT_YUYV, 30);
        cfgs.emplace_back(cfgs_arr[i]);
    }*/
    /*{"fmt": rs.format.yuyv, "w": 640, "h": 360, "fps": 6},
{"fmt": rs.format.yuyv, "w": 640, "h": 360, "fps": 15},
{"fmt": rs.format.yuyv, "w": 640, "h": 360, "fps": 30},
{"fmt": rs.format.yuyv, "w": 640, "h": 360, "fps": 60},
{"fmt": rs.format.yuyv, "w": 640, "h": 480, "fps": 6},
{"fmt": rs.format.yuyv, "w": 640, "h": 480, "fps": 15},
{"fmt": rs.format.yuyv, "w": 640, "h": 480, "fps": 30},
{"fmt": rs.format.yuyv, "w": 640, "h": 480, "fps": 60},
{"fmt": rs.format.yuyv, "w": 848, "h": 480, "fps": 6},
{"fmt": rs.format.yuyv, "w": 848, "h": 480, "fps": 15},
{"fmt": rs.format.yuyv, "w": 848, "h": 480, "fps": 30},
{"fmt": rs.format.yuyv, "w": 848, "h": 480, "fps": 60},
{"fmt": rs.format.yuyv, "w": 960, "h": 540, "fps": 6},
{"fmt": rs.format.yuyv, "w": 960, "h": 540, "fps": 15},
{"fmt": rs.format.yuyv, "w": 960, "h": 540, "fps": 30},
{"fmt": rs.format.yuyv, "w": 960, "h": 540, "fps": 60},
{"fmt": rs.format.yuyv, "w": 1280, "h": 720, "fps": 6},
{"fmt": rs.format.yuyv, "w": 1280, "h": 720, "fps": 15},
{"fmt": rs.format.yuyv, "w": 1280, "h": 720, "fps": 30},
*/
    /*cfgs_arr[0].enable_stream(RS2_STREAM_COLOR, -1, 640, 360, RS2_FORMAT_YUYV, 6);
    cfgs_arr[1].enable_stream(RS2_STREAM_COLOR, -1, 640, 360, RS2_FORMAT_YUYV, 15);
    cfgs_arr[2].enable_stream(RS2_STREAM_COLOR, -1, 640, 360, RS2_FORMAT_YUYV, 30);
    cfgs_arr[3].enable_stream(RS2_STREAM_COLOR, -1, 640, 360, RS2_FORMAT_YUYV, 60);
    cfgs_arr[4].enable_stream(RS2_STREAM_COLOR, -1, 640, 480, RS2_FORMAT_YUYV, 6);
    cfgs_arr[5].enable_stream(RS2_STREAM_COLOR, -1, 640, 480, RS2_FORMAT_YUYV, 15);
    cfgs_arr[6].enable_stream(RS2_STREAM_COLOR, -1, 640, 480, RS2_FORMAT_YUYV, 30);
    cfgs_arr[7].enable_stream(RS2_STREAM_COLOR, -1, 640, 480, RS2_FORMAT_YUYV, 60);

    cfgs_arr[8].enable_stream(RS2_STREAM_COLOR, -1, 848, 480, RS2_FORMAT_YUYV, 6);
    cfgs_arr[9].enable_stream(RS2_STREAM_COLOR, -1, 848, 480, RS2_FORMAT_YUYV, 15);
    cfgs_arr[10].enable_stream(RS2_STREAM_COLOR, -1, 848, 480, RS2_FORMAT_YUYV, 30);
    cfgs_arr[11].enable_stream(RS2_STREAM_COLOR, -1, 848, 480, RS2_FORMAT_YUYV, 60);

    cfgs_arr[12].enable_stream(RS2_STREAM_COLOR, -1, 960, 540, RS2_FORMAT_YUYV, 6);
    cfgs_arr[13].enable_stream(RS2_STREAM_COLOR, -1, 960, 540, RS2_FORMAT_YUYV, 15);
    cfgs_arr[14].enable_stream(RS2_STREAM_COLOR, -1, 960, 540, RS2_FORMAT_YUYV, 30);
    cfgs_arr[15].enable_stream(RS2_STREAM_COLOR, -1, 960, 540, RS2_FORMAT_YUYV, 60);

    cfgs_arr[16].enable_stream(RS2_STREAM_COLOR, -1, 1280, 720, RS2_FORMAT_YUYV, 6);
    cfgs_arr[17].enable_stream(RS2_STREAM_COLOR, -1, 1280, 720, RS2_FORMAT_YUYV, 15);
    cfgs_arr[18].enable_stream(RS2_STREAM_COLOR, -1, 1280, 720, RS2_FORMAT_YUYV, 30);
    //cfgs_arr[i].enable_stream(RS2_STREAM_COLOR, -1, 960, 540, RS2_FORMAT_YUYV, 60);*/



    cfgs_arr[0].enable_stream(RS2_STREAM_COLOR, -1, 640, 360, RS2_FORMAT_YUYV, 30);
    cfgs_arr[1].enable_stream(RS2_STREAM_COLOR, -1, 640, 360, RS2_FORMAT_YUYV, 30);
    cfgs_arr[2].enable_stream(RS2_STREAM_COLOR, -1, 640, 360, RS2_FORMAT_YUYV, 30);
    cfgs_arr[3].enable_stream(RS2_STREAM_COLOR, -1, 640, 360, RS2_FORMAT_YUYV, 30);
    cfgs_arr[4].enable_stream(RS2_STREAM_COLOR, -1, 640, 480, RS2_FORMAT_YUYV, 30);
    cfgs_arr[5].enable_stream(RS2_STREAM_COLOR, -1, 640, 480, RS2_FORMAT_YUYV, 30);
    cfgs_arr[6].enable_stream(RS2_STREAM_COLOR, -1, 640, 480, RS2_FORMAT_YUYV, 30);
    cfgs_arr[7].enable_stream(RS2_STREAM_COLOR, -1, 640, 480, RS2_FORMAT_YUYV, 30);

    cfgs_arr[8].enable_stream(RS2_STREAM_COLOR, -1, 848, 480, RS2_FORMAT_YUYV, 30);
    cfgs_arr[9].enable_stream(RS2_STREAM_COLOR, -1, 848, 480, RS2_FORMAT_YUYV, 30);
    cfgs_arr[10].enable_stream(RS2_STREAM_COLOR, -1, 848, 480, RS2_FORMAT_YUYV, 30);
    cfgs_arr[11].enable_stream(RS2_STREAM_COLOR, -1, 848, 480, RS2_FORMAT_YUYV, 30);

    cfgs_arr[12].enable_stream(RS2_STREAM_COLOR, -1, 960, 540, RS2_FORMAT_YUYV, 30);
    cfgs_arr[13].enable_stream(RS2_STREAM_COLOR, -1, 960, 540, RS2_FORMAT_YUYV, 30);
    cfgs_arr[14].enable_stream(RS2_STREAM_COLOR, -1, 960, 540, RS2_FORMAT_YUYV, 30);
    cfgs_arr[15].enable_stream(RS2_STREAM_COLOR, -1, 960, 540, RS2_FORMAT_YUYV, 30);

    cfgs_arr[16].enable_stream(RS2_STREAM_COLOR, -1, 1280, 720, RS2_FORMAT_YUYV, 30);
    cfgs_arr[17].enable_stream(RS2_STREAM_COLOR, -1, 1280, 720, RS2_FORMAT_YUYV, 30);
    cfgs_arr[18].enable_stream(RS2_STREAM_COLOR, -1, 1280, 720, RS2_FORMAT_YUYV, 30);


    cfgs_arr[19].enable_stream(RS2_STREAM_COLOR, -1, 640, 360, RS2_FORMAT_YUYV, 30);
    cfgs_arr[20].enable_stream(RS2_STREAM_COLOR, -1, 640, 360, RS2_FORMAT_YUYV, 30);
    cfgs_arr[21].enable_stream(RS2_STREAM_COLOR, -1, 640, 360, RS2_FORMAT_YUYV, 30);
    cfgs_arr[22].enable_stream(RS2_STREAM_COLOR, -1, 640, 360, RS2_FORMAT_YUYV, 30);
    cfgs_arr[23].enable_stream(RS2_STREAM_COLOR, -1, 640, 480, RS2_FORMAT_YUYV, 30);
    cfgs_arr[24].enable_stream(RS2_STREAM_COLOR, -1, 640, 480, RS2_FORMAT_YUYV, 30);
    cfgs_arr[25].enable_stream(RS2_STREAM_COLOR, -1, 640, 480, RS2_FORMAT_YUYV, 30);
    cfgs_arr[26].enable_stream(RS2_STREAM_COLOR, -1, 640, 480, RS2_FORMAT_YUYV, 30);

    cfgs_arr[27].enable_stream(RS2_STREAM_COLOR, -1, 848, 480, RS2_FORMAT_YUYV, 30);
    cfgs_arr[28].enable_stream(RS2_STREAM_COLOR, -1, 848, 480, RS2_FORMAT_YUYV, 30);
    cfgs_arr[29].enable_stream(RS2_STREAM_COLOR, -1, 848, 480, RS2_FORMAT_YUYV, 30);


    for (auto i = 0; i < 30; i++)
    {
        cfgs.emplace_back(cfgs_arr[i]);
    }

    auto frames_per_iteration = 30 *5; // fps * 5
    auto count = 0;
    //for (auto& cfg : cfgs) {
    for(auto i=0; i<20;i++){
        std::cout << "\nNOHA :: ============== Iteration " <<count << " ========="<< std::endl;
        count += 1;
        try {
            rs2::pipeline pipe; // it will delete previous memory, if issue still reproduce then the delay is not cause because of profiles
            auto t1 = std::chrono::system_clock::now();
            bool first = true;
            rs2::config tmp;
            tmp.enable_stream(RS2_STREAM_COLOR, -1, 640, 480, RS2_FORMAT_YUYV, 30);
            pipe.start(tmp);
            
            auto t2 = std::chrono::system_clock::now();
            auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
            //std::cout << "NOHA :: pipe.start : " << diff << " msec" << std::endl;

            for (auto i = 0; i < frames_per_iteration; i++)
            {
                rs2::frameset data = pipe.wait_for_frames();// .    // Wait for next set of frames from the camera
                    //apply_filter(printer); //.     // Print each enabled stream frame rate
                    //apply_filter(color_map);   // Find and colorize the depth data
              //  std::cout << endl;
                if (first) {
                    auto t2 = std::chrono::system_clock::now();
                    auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
                    std::cout << "NOHA :: Time to first frame : " << diff << " msec" << std::endl;
                    first = false;
                }
            }

            


            pipe.stop();

        }
        catch (...)
        {
            std::cout << "Iteration failed  " << std::endl;
        }
    }
 
    /*while (app) // Application still alive?
    {
        for (auto i = 0; i < 3; i++) {
            auto startTime = std::chrono::system_clock::now();
            rs2::frameset data = pipe.wait_for_frames().    // Wait for next set of frames from the camera
                apply_filter(printer).     // Print each enabled stream frame rate
                apply_filter(color_map);   // Find and colorize the depth data
            if (first) {
                auto endTime = std::chrono::system_clock::now();
                auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count();
                std::cout << "NOHA :: Time to first frame : " << diff << " msec" << std::endl;
                first = false;
            }
        }
        // The show method, when applied on frameset, break it to frames and upload each frame into a gl textures
        // Each texture is displayed on different viewport according to it's stream unique id
        //app.show(data);
    }*/

    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}