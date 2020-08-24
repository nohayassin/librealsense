// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2018 Intel Corporation. All Rights Reserved.
// Implementation details of hole-filling modes

#include "../include/librealsense2/hpp/rs_sensor.hpp"
#include "../include/librealsense2/hpp/rs_processing.hpp"
#include "option.h"
#include "environment.h"
#include "context.h"
#include "software-device.h"
#include "proc/synthetic-stream.h"
#include "denoise-autoencoder.h"

namespace librealsense
{

    denoise_autoencoder::denoise_autoencoder() :
        depth_processing_block("Denoise Autoencoder"),
        _width(0), _height(0), _stride(0)
    {

    }



}
