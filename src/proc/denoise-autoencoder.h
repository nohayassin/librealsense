#pragma once

namespace librealsense
{
  

    class denoise_autoencoder : public depth_processing_block
    {
    public:
        denoise_autoencoder();

    private:

        size_t                  _width, _height, _stride;
    };
    MAP_EXTENSION(RS2_EXTENSION_DENOISE_AUTOENCODER, librealsense::denoise_autoencoder);
}
