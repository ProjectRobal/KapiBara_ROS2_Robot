#pragma once

#include <thread>
#include <mutex>
#include <condition_variable>
#include <chrono>

template<typename T>
class Event
{
    std::mutex m_lock;
    std::condition_variable var;
    bool start;
    T value;

    public:

    Event()
    {
        this->start = false;
    }

    bool wait_for(T& data, int64_t seconds = 60 )
    {
        std::unique_lock<std::mutex> u_lock(this->m_lock);

        // this->var.wait(u_lock,[this]{ return this->start;});

        if(this->var.wait_for(u_lock,std::chrono::seconds(seconds)) == std::cv_status::no_timeout )
        {
            this->start = false;
            data = this->value;
            return true;
        }
        
        this->start = false;
        return false;
    }

    const T& wait()
    {
        std::unique_lock<std::mutex> u_lock(this->m_lock);

        this->var.wait(u_lock,[this]{ return this->start;});
        
        this->start = false;

        return this->value;
    }

    void notify(const T& value)
    {
        std::unique_lock<std::mutex> u_lock(this->m_lock);

        this->start = true;

        this->value = value;

        this->var.notify_one();
    }


};