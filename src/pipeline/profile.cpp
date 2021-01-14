// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015 Intel Corporation. All Rights Reserved.

#include "profile.h"
#include "media/record/record_device.h"
#include "media/ros/ros_writer.h"

namespace librealsense
{
    namespace pipeline
    {
        profile::profile(std::shared_ptr<device_interface> dev,
            util::config config,
            const std::string& to_file) :
            _dev(dev), _to_file(to_file)
        {
            //std::cout << "NOHA ::  profile::profile (0): " << std::endl;
            auto t1 = std::chrono::system_clock::now();
            if (!to_file.empty())
            {
                if (!dev)
                    throw librealsense::invalid_value_exception("Failed to create a profile, device is null");

                _dev = std::make_shared<record_device>(dev, std::make_shared<ros_writer>(to_file, dev->compress_while_record()));

                auto t2 = std::chrono::system_clock::now();
                auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
                //std::cout << "NOHA ::  profile::profile (1): " << diff << " msec" << std::endl;
            }

            t1 = std::chrono::system_clock::now();

            auto val = _dev.get();

            auto t2 = std::chrono::system_clock::now();
            auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
            //std::cout << "NOHA ::  profile::profile (2.0): " << diff << " msec" << std::endl;


            t1 = std::chrono::system_clock::now();
            _multistream = config.resolve(val);

            t2 = std::chrono::system_clock::now();
            diff = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
            //std::cout << "NOHA ::  profile::profile (2): " << diff << " msec" << std::endl;
        }

        std::shared_ptr<device_interface> profile::get_device()
        {
            //profile can be retrieved from a pipeline_config and pipeline::start()
            //either way, it is created by the pipeline

            //TODO: handle case where device has disconnected and reconnected
            //TODO: remember to recreate the device as record device in case of to_file.empty() == false
            if (!_dev)
            {
                throw std::runtime_error("Device is unavailable");
            }
            return _dev;
        }

        void profile::reset_device()
        {
            //profile can be retrieved from a pipeline_config and pipeline::start()
            //either way, it is created by the pipeline

            //TODO: handle case where device has disconnected and reconnected
            //TODO: remember to recreate the device as record device in case of to_file.empty() == false
            if (!_dev)
            {
                throw std::runtime_error("Device is unavailable");
            }
            _dev.reset();
        }

        stream_profiles profile::get_active_streams() const
        {
            auto profiles_per_sensor = _multistream.get_profiles_per_sensor();
            stream_profiles profiles;
            for (auto&& kvp : profiles_per_sensor)
                for (auto&& p : kvp.second)
                    profiles.push_back(p);

            return profiles;
        }
    }
}
