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
        _width(0), _height(0), _stride(0), _bpp(0),
        _extension_type(RS2_EXTENSION_DEPTH_FRAME),
        _current_frm_size_pixels(0)
    {

    }

    rs2::frame denoise_autoencoder::process_frame(const rs2::frame_source& source, const rs2::frame& f)
    {
        update_configuration(f);
        auto tgt = prepare_target_frame(f, source);

        return tgt;
    }

    void  denoise_autoencoder::update_configuration(const rs2::frame& f)
    {
       
    }

    rs2::frame denoise_autoencoder::prepare_target_frame(const rs2::frame& f, const rs2::frame_source& source)
    {
        // Allocate and copy the content of the input data to the target
        rs2::frame tgt = source.allocate_video_frame(_target_stream_profile, f, int(_bpp), int(_width), int(_height), int(_stride), _extension_type);

        memmove(const_cast<void*>(tgt.get_data()), f.get_data(), _current_frm_size_pixels * _bpp);
        return tgt;
    }
}
