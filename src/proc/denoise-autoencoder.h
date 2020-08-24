#pragma once

namespace librealsense
{
  

    class denoise_autoencoder : public depth_processing_block
    {
    public:
        denoise_autoencoder();

    protected:
        void update_configuration(const rs2::frame& f);
        rs2::frame process_frame(const rs2::frame_source& source, const rs2::frame& f) override;
        rs2::frame prepare_target_frame(const rs2::frame& f, const rs2::frame_source& source);

    private:

        size_t                  _width, _height, _stride;
        size_t                  _bpp;
        rs2_extension           _extension_type;            // Strictly Depth/Disparity
        size_t                  _current_frm_size_pixels;
        rs2::stream_profile     _source_stream_profile;
        rs2::stream_profile     _target_stream_profile;
    };
    MAP_EXTENSION(RS2_EXTENSION_DENOISE_AUTOENCODER, librealsense::denoise_autoencoder);
}
