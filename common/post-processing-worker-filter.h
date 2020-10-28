// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 Intel Corporation. All Rights Reserved.

#pragma once

#include "post-processing-filter.h"


/*
    A post-processing filter that employs a separate thread to do its work.
*/
class post_processing_worker_filter : public post_processing_filter
{
    std::atomic_bool _alive { true };
    std::thread _worker;
    rs2::frame_queue _queue;   // size 1!

protected:
    post_processing_worker_filter( std::string const & name )
        : post_processing_filter( name )
    {
    }
    ~post_processing_worker_filter()
    {
        _alive = false;
        if (_worker.joinable())
        {
            // joinable() only checks that the thread's alive -- but we don't want to join
            // if we're here from within the worker thread itself!
            if (std::this_thread::get_id() != _worker.get_id())
                _worker.join();
            else 
                _worker.detach();
        }
    }

public:
    void start( rs2::subdevice_model & model ) override
    {
        post_processing_filter::start(model);
        // worker_thread accesses worker-filter's variables and functions. 
        // If worker-filter's destructoer is called while worker_thread runs, it will cause memory access issues.
        // So it is important to delay the destruction of worker filter by defining a shared pointer to it.
        // By defining a shared pointer, refrence count to worker filter will stay 1 until worker_thread is done.
        // After worker_thread is done, the weak pointer is released and refrence count becomes 0, then worker-filter's
        // destructor is called.
        auto filter_p = this->shared_from_this();
        auto weak_p = std::weak_ptr<post_processing_filter>(filter_p);
        _worker = std::thread([&, weak_p]()
            {
                try
                {
                    if (auto shared_p = weak_p.lock())
                    {
                        worker_start();
                    }
                }
                catch (std::exception const& e)
                {
                    // Most likely file not found, if the user didn't set up his .xml/.bin files right
                    LOG(ERROR) << "Cannot start " << get_name() << ": " << e.what();
                    return;
                }
                while (_alive)
                {
                    rs2::frame f;
                    if (!_queue.try_wait_for_frame(&f))
                        continue;
                    if (!f)
                        continue;
                    if (auto shared_p = weak_p.lock())
                    {
                        worker_body(f.as< rs2::frameset >());
                    }
                }
                LOG(DEBUG) << "End of worker loop in " + get_name();
                if (auto shared_p = weak_p.lock())
                {
                    worker_end();
                }
            });
    }

protected:
    rs2::frameset process_frameset( rs2::frameset fs ) override
    {
        _queue.enqueue( fs );
        return fs;
    }

    virtual void worker_start() {}
    virtual void worker_end() {}

    virtual void worker_body( rs2::frameset fs ) = 0;
};
