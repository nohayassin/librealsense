// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#include <iostream>
#include <iomanip>
#include <map>
#include <utility>
#include <vector>
#include <librealsense2/rs.hpp>
#include <algorithm>
#include "../example.hpp"
#include "api_how_to.h"
#include "helper.h"

int main(int argc, char * argv[]) try
{
    rs2::context                          ctx;
    auto&& device = ctx.query_devices()[0];
    auto sensors = device.query_sensors();
    rs2::sensor depth_sensor;
    float ae_limit;
    for (auto& s : sensors)
    {
        std::string val = s.get_info(RS2_CAMERA_INFO_NAME);
        if (!val.compare("Stereo Module")) {
            depth_sensor = s;
            depth_sensor.set_option(RS2_OPTION_AUTO_EXPOSURE_LIMIT, 10000);
            ae_limit = depth_sensor.get_option(RS2_OPTION_AUTO_EXPOSURE_LIMIT);
        }

    }
    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception & e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
