//// License: Apache 2.0. See LICENSE file in root directory.
//// Copyright(c) 2018 Intel Corporation. All Rights Reserved.

#include "l500-private.h"
#include "fw-update/fw-update-unsigned.h"
#include "context.h"
#include "core/video.h"
#include "depth-to-rgb-calibration.h"
#include "log.h"
#include <chrono>

using namespace std;

namespace librealsense
{
    namespace ivcam2
    {
        rs2_extrinsics get_color_stream_extrinsic(const std::vector<uint8_t>& raw_data)
        {
            if (raw_data.size() < sizeof(pose))
                throw invalid_value_exception("size of extrinsic invalid");
            
            assert( sizeof( pose ) == sizeof( rs2_extrinsics ) );
            auto res = *(rs2_extrinsics*)raw_data.data();
            AC_LOG( DEBUG, "raw extrinsics data from camera:\n" << std::setprecision(15) << res );
            
            float trans_scale = 0.001f; // Convert units from mm to meter
            res.translation[0] *= trans_scale;
            res.translation[1] *= trans_scale;
            res.translation[2] *= trans_scale;

            // This transposing is absolutely mandatory because our internal algorithms are
            // written with a transposed matrix in mind! (see rs2_transform_point_to_point)
            std::swap( res.rotation[1], res.rotation[3] );
            std::swap( res.rotation[2], res.rotation[6] );
            std::swap( res.rotation[5], res.rotation[7] );
            return res;
        }

        bool try_fetch_usb_device(std::vector<platform::usb_device_info>& devices,
            const platform::uvc_device_info& info, platform::usb_device_info& result)
        {
            for (auto it = devices.begin(); it != devices.end(); ++it)
            {
                if (it->unique_id == info.unique_id)
                {

                    result = *it;
                    switch(info.pid)
                    {
                    case L515_PID_PRE_PRQ:
                    case L515_PID:
                        if(result.mi == 7)
                        {
                            devices.erase(it);
                            return true;
                        }
                        break;
                    case L500_PID:
                        if(result.mi == 4 || result.mi == 6)
                        {
                            devices.erase(it);
                            return true;
                        }
                        break;
                    default:
                        break;
                    }
                }
            }
            return false;
        }
        float l500_temperature_options::query() const
        {
            if (!is_enabled())
                throw wrong_api_call_sequence_exception("query option is allow only in streaming!");

#pragma pack(push, 1)
            struct temperatures
            {
                double LLD_temperature;
                double MC_temperature;
                double MA_temperature;
                double APD_temperature;
            };
#pragma pack(pop)

            auto res = _hw_monitor->send(command{ TEMPERATURES_GET });

            if (res.size() < sizeof(temperatures))
            {
                throw std::runtime_error("Invalid result size!");
            }

            auto temperature_data = *(reinterpret_cast<temperatures*>((void*)res.data()));

            switch (_option)
            {
            case RS2_OPTION_LLD_TEMPERATURE:
                return float(temperature_data.LLD_temperature);
            case RS2_OPTION_MC_TEMPERATURE:
                return float(temperature_data.MC_temperature);
            case RS2_OPTION_MA_TEMPERATURE:
                return float(temperature_data.MA_temperature);
            case RS2_OPTION_APD_TEMPERATURE:
                return float(temperature_data.APD_temperature);
            default:
                throw invalid_value_exception(to_string() << _option << " is not temperature option!");
            }
        }

        l500_temperature_options::l500_temperature_options(hw_monitor* hw_monitor, rs2_option opt)
            :_hw_monitor(hw_monitor), _option(opt)
        {
        }

        flash_structure get_rw_flash_structure(const uint32_t flash_version)
        {
            switch (flash_version)
            {
                // { number of payloads in section { ro table types }} see Flash.xml
            case 103: return { 1, { 40, 320, 321, 326, 327, 54} };
            default:
                throw std::runtime_error("Unsupported flash version: " + std::to_string(flash_version));
            }
        }

        flash_structure get_ro_flash_structure(const uint32_t flash_version)
        {
            switch (flash_version)
            {
                // { number of payloads in section { ro table types }} see Flash.xml
            case 103: return { 4, { 256, 257, 258, 263, 264, 512, 25, 2 } };
            default:
                throw std::runtime_error("Unsupported flash version: " + std::to_string(flash_version));
            }
        }

        flash_info get_flash_info(const std::vector<uint8_t>& flash_buffer)
        {
            flash_info rv = {};

            uint32_t header_offset = FLASH_INFO_HEADER_OFFSET;
            memcpy(&rv.header, flash_buffer.data() + header_offset, sizeof(rv.header));

            uint32_t ro_toc_offset = FLASH_RO_TABLE_OF_CONTENT_OFFSET;
            uint32_t rw_toc_offset = FLASH_RW_TABLE_OF_CONTENT_OFFSET;

            auto ro_toc = parse_table_of_contents(flash_buffer, ro_toc_offset);
            auto rw_toc = parse_table_of_contents(flash_buffer, rw_toc_offset);

            auto ro_structure = get_ro_flash_structure(ro_toc.header.version);
            auto rw_structure = get_rw_flash_structure(rw_toc.header.version);

            rv.read_only_section = parse_flash_section(flash_buffer, ro_toc, ro_structure);
            rv.read_only_section.offset = rv.header.read_only_start_address;
            rv.read_write_section = parse_flash_section(flash_buffer, rw_toc, rw_structure);
            rv.read_write_section.offset = rv.header.read_write_start_address;

            return rv;
        }

        freefall_option::freefall_option( hw_monitor & hwm, bool enabled )
            : _hwm( hwm )
            , _enabled( enabled )
        {
            // bool_option initializes with def=true, which is what we want
            assert( is_true() );
        }

        void freefall_option::set( float value )
        {
            bool_option::set( value );

            command cmd{ FALL_DETECT_ENABLE, is_true() };
            auto res = _hwm.send( cmd );
            _record_action( *this );
        }

        void freefall_option::enable( bool e )
        {
            if( e != is_enabled() )
            {
                if( !e  &&  is_true() )
                    set( 0 );
                _enabled = e;
            }
        }

        hw_sync_option::hw_sync_option( hw_monitor& hwm, std::shared_ptr< freefall_option > freefall_opt )
            : bool_option( false )
            , _hwm( hwm )
            , _freefall_opt( freefall_opt )
        {
        }

        void hw_sync_option::set( float value )
        {
            bool_option::set( value );

            // Disable the free-fall option if we're enabled!
            if( _freefall_opt )
                _freefall_opt->enable( ! is_true() );

            command cmd{ HW_SYNC_EX_TRIGGER, is_true() };
            auto res = _hwm.send( cmd );
            _record_action( *this );
        }

        auto_calibration::enabler_option::enabler_option( std::shared_ptr< auto_calibration > const & autocal )
            : bool_option( false )
            , _autocal( autocal )
        {
        }

        void auto_calibration::enabler_option::set( float value )
        {
            bool_option::set( value );
            if( is_true() )
            {
                // We've turned it on -- try to immediately get a special frame
                _autocal->trigger_special_frame();
            }
            _record_action( *this );
        }


        // Implementation class: starts another thread responsible for sending a retry
        // NOTE that it does this as long as our shared_ptr keeps us alive: once it's gone, if the
        // retry period elapses then nothing will happen!
        class auto_calibration::retrier
        {
            auto_calibration & _ac;

            retrier( auto_calibration & ac )
                : _ac( ac )
            {
                AC_LOG( DEBUG, "retrier " << std::hex << this << std::dec );
            }

            void retry()
            {
                AC_LOG( DEBUG, "retrying " << std::hex << this << std::dec );
                _ac.trigger_special_frame( true );
            }

        public:
            ~retrier()
            {
                AC_LOG( DEBUG, "~retrier " << std::hex << this << std::dec );
            }

            static std::shared_ptr< retrier > start( auto_calibration & autocal )
            {
                auto r = std::shared_ptr< retrier >( new retrier( autocal ) );
                std::weak_ptr< retrier > weak { r };
                std::thread( [weak]()
                {
                    std::this_thread::sleep_for( std::chrono::seconds( 2 ) );
                    if( auto r = weak.lock() )
                        r->retry();
                    else
                        AC_LOG( DEBUG, "retry period over; no retry needed" );
                } ).detach();
                return r;
            };
        };


        auto_calibration::auto_calibration( hw_monitor & hwm )
            : _is_processing{ false }
            , _hwm( hwm )
        {
#if 1
            // TODO remove this -- but for now, it's an easy way of seeing the AC stuff
            class ac_logger : public rs2_log_callback
            {
                std::ofstream _f;

            public:
                explicit ac_logger()
                {
                    using namespace std::chrono;
                    std::string filename = to_string()
                        << "C:\\work\\autocal\\data\\"
                        << system_clock::now().time_since_epoch().count() * system_clock::period::num / system_clock::period::den
                        << ".ac_log";
                   
                    _f.open( filename );
                    if( _f )
                        std::cout << "-D- AC log is being written to: " << filename << std::endl;
                }

                void on_log( rs2_log_severity severity, rs2_log_message const & msg ) noexcept override
                {
                    log_message const & wrapper = (log_message const &)(msg);
                    char const * raw = wrapper.el_msg.message().c_str();
                    if( strncmp( "AC1: ", raw, 5 ) )
                        return;
                    std::ostringstream ss;
                    ss << "-" << "DIWE"[severity] << "- ";
                    ss << (raw + 5);
                    std::string text = ss.str();
                    std::cout << text << std::endl;
                    if( _f )
                        _f << text << std::endl;
                }

                void release() override { delete this; }
            };
            librealsense::log_to_callback( RS2_LOG_SEVERITY_ALL,
                { new ac_logger, []( rs2_log_callback * p ) { p->release(); } }
            );
#endif
        }

        auto_calibration::~auto_calibration()
        {
            if( _worker.joinable() )
            {
                _is_processing = false;  // Signal the thread that we want to stop!
                _worker.join();
            }
        }

        void auto_calibration::trigger_special_frame( bool is_retry )
        {
            if( _is_processing )
            {
                AC_LOG( ERROR, "Failed to trigger_special_frame: auto-calibration is already in-process" );
                return;
            }
            if( is_retry )
            {
                if( _n_retries > 4 )
                {
                    AC_LOG( ERROR, "too many retries; aborting" );
                    call_back( RS2_CALIBRATION_FAILED );
                    return;
                }
                ++_n_retries;
                AC_LOG( DEBUG, "GET_SPECIAL_FRAME (retry " << _n_retries << ")" );
                call_back( RS2_CALIBRATION_RETRY );
            }
            else
            {
                _n_retries = 0;
                AC_LOG( DEBUG, "GET_SPECIAL_FRAME" );
            }
            command cmd { GET_SPECIAL_FRAME, 0x5F, 1 };  // 5F = SF = Special Frame, for easy recognition
            auto res = _hwm.send( cmd );
            // Start a timer: enable retries if something's wrong with the special frame
            _retrier = retrier::start( *this );
        }

        void auto_calibration::set_special_frame( rs2::frameset const& fs )
        {
            AC_LOG( DEBUG, "special frame received :)" );
            // Notify of the special frame -- mostly for validation team so they know to expect a frame drop...
            call_back( RS2_CALIBRATION_SPECIAL_FRAME );

            if( _is_processing )
            {
                AC_LOG( ERROR, "already processing; ignoring special frame!" );
                return;
            }
            auto irf = fs.get_infrared_frame();
            if( ! irf )
            {
                AC_LOG( ERROR, "no IR frame found; ignoring special frame!" );
                call_back( RS2_CALIBRATION_FAILED );
                return;
            }
            auto df = fs.get_depth_frame();
            if( !df )
            {
                AC_LOG( ERROR, "no depth frame found; ignoring special frame!" );
                call_back( RS2_CALIBRATION_FAILED );
                return;
            }

            _sf = fs;
            _retrier.reset();

            if( check_color_depth_sync() )
                start();
        }

        void auto_calibration::set_color_frame( rs2::frame const& f )
        {
            if( _is_processing )
                // No error message -- we expect to get new color frames while processing...
                return;

            _pcf = _cf;
            _cf = f;
            if( check_color_depth_sync() )
                start();
        }


        bool auto_calibration::check_color_depth_sync()
        {
            if( !_sf )
            {
                //std::cout << "-D- no special frame yet" << std::endl;
                return false;
            }
            if( !_cf )
            {
                //std::cout << "-D- no color frame yet" << std::endl;
                return false;
            }
            return true;
        }

        void auto_calibration::start()
        {
            AC_LOG( DEBUG, "starting processing ..." );
            _is_processing = true;
            if( _worker.joinable() )
                _worker.join();
            _worker = std::thread( [&]()
            {
                try
                {
                    AC_LOG( DEBUG, "auto calibration has started ...");
                    call_back( RS2_CALIBRATION_STARTED );

                    auto df = _sf.get_depth_frame();
                    auto irf = _sf.get_infrared_frame();
                    depth_to_rgb_calibration algo( df, irf, _cf, _pcf );
                    _from_profile = algo.get_from_profile();
                    _to_profile = algo.get_to_profile();

                    bool retry = false;
                    auto status = algo.optimize();
                    switch( status )
                    {
                    case RS2_CALIBRATION_SUCCESSFUL:
                        AC_LOG( DEBUG, "optimization successful!" );
                        /*  auto prof = _cf.get_profile().get()->profile;
                        auto&& video = dynamic_cast<video_stream_profile_interface*>(prof);
                        if (video)
                            video->set_intrinsics([new_calib]() {return new_calib.intrinsics;});
                        _df.get_profile().register_extrinsics_to(_cf.get_profile(), new_calib.extrinsics);*/
                        _extr = algo.get_extrinsics();
                        _intr = algo.get_intrinsics();
                        break;
                    case RS2_CALIBRATION_NOT_NEEDED:
                        // This is the same as SUCCESSFUL, except there was no change because the
                        // existing calibration is good enough. We notify and exit.
                        break;
                    case RS2_CALIBRATION_BAD_RESULT:
                    case RS2_CALIBRATION_SCENE_INVALID:
                        // These two require a retry
                        retry = true;
                        break;
                    case RS2_CALIBRATION_FAILED:
                        // This is not expected; really only possible when the algorithm crashed or had
                        // some unusual condition -- we just report it and quit (TODO maybe retry?)
                        break;
                    default:
                        // All the rest of the codes are not end-states of the algo, so we don't expect
                        // to get here!
                        AC_LOG( ERROR, "Unexpected status '" << status << "' received from AC algo; stopping!" );
                        break;
                    }
                    call_back( status );
                    reset();
                    if( retry )
                    {
                        trigger_special_frame( true );
                    }
                }
                catch( std::exception& ex )
                {
                    AC_LOG( ERROR, "caught exception: " << ex.what() );
                    call_back( RS2_CALIBRATION_FAILED );
                    reset();
                }
                catch (...)
                {
                    AC_LOG( ERROR, "unknown exception in AC!!!" );
                    call_back( RS2_CALIBRATION_FAILED );
                    reset();
                }});
        }

        void auto_calibration::reset()
        {
            _sf = rs2::frame{};
            _cf = rs2::frame{};;
            _pcf = rs2::frame{};

            _is_processing = false;
            AC_LOG( DEBUG, "reset()" );
        }

        autocal_depth_processing_block::autocal_depth_processing_block(
            std::shared_ptr< auto_calibration > autocal
        )
            : generic_processing_block( "Auto Calibration (depth)" )
            , _autocal{ autocal }
        {
        }

        static bool is_special_frame( rs2::depth_frame const& f )
        {
            if( !f )
                return false;

            if( f.supports_frame_metadata( RS2_FRAME_METADATA_FRAME_LASER_POWER_MODE ))
            {
                // We specified 0x5F (SF = Special Frame) when we asked for the SPECIAL_FRAME
                auto mode = f.get_frame_metadata( RS2_FRAME_METADATA_FRAME_LASER_POWER_MODE );
                if( 0x5F == mode )
                {
                    AC_LOG( DEBUG, "frame " << f.get_frame_number() << " is our special frame" );
                    return true;
                }
                return false;
            }

            // When the frame doesn't support metadata, we have to look at its
            // contents: we take the first frame that has 100% fill rate, i.e.
            // has no pixels with 0 in them...
            uint16_t const * pd = reinterpret_cast< const uint16_t * >( f.get_data() );
            for( size_t x = f.get_data_size() / sizeof( *pd ); x--; ++pd )
            {
                if( !*pd )
                    return false;
            }
            AC_LOG( DEBUG, "frame " << f.get_frame_number() << " has no metadata but 100% fill rate -> assuming special frame" );
            return true;  // None zero, assume special frame...
        }

        rs2::frame autocal_depth_processing_block::process_frame( const rs2::frame_source& source, const rs2::frame& f )
        {
            // AC can be triggered manually, too, so we do NOT check whether the option is on!

            auto fs = f.as< rs2::frameset >();
            if( fs )
            {
                auto df = fs.get_depth_frame();
                if( _autocal->is_expecting_special_frame()  &&  is_special_frame( df ))
                    _autocal->set_special_frame( f );
                // Disregard framesets: we'll get those broken down into individual frames by generic_processing_block's on_frame
                return rs2::frame{};
            }

            if( _autocal->is_expecting_special_frame()  &&  is_special_frame( f.as< rs2::depth_frame >() ) )
                // We don't want the user getting this frame!
                return rs2::frame{};
            
            return f;
        }

        bool autocal_depth_processing_block::should_process( const rs2::frame & frame )
        {
            return true;
        }

        rs2::frame autocal_depth_processing_block::prepare_output( const rs2::frame_source& source, rs2::frame input, std::vector<rs2::frame> results )
        {
            // The default prepare_output() will send the input back as the frame if the results are empty
            if( results.empty() )
                return rs2::frame{};

            return source.allocate_composite_frame( results );
        }

        autocal_color_processing_block::autocal_color_processing_block(
            std::shared_ptr< auto_calibration > autocal
        )
            : generic_processing_block( "Auto Calibration (color)" )
            , _autocal{ autocal }
        {
        }

        rs2::frame autocal_color_processing_block::process_frame( const rs2::frame_source& source, const rs2::frame& f )
        {
            // AC can be triggered manually, too, so we do NOT check whether the option is on!

            // Disregard framesets: we'll get those broken down into individual frames by generic_processing_block's on_frame
            if( f.is< rs2::frameset >() )
                return rs2::frame{};
            
            // We record each and every color frame
            _autocal->set_color_frame( f );

            // Return the frame as is!
            return f;
        }

        bool autocal_color_processing_block::should_process( const rs2::frame& frame )
        {
            return true;
        }
    } // librealsense::ivcam2
} // namespace librealsense
