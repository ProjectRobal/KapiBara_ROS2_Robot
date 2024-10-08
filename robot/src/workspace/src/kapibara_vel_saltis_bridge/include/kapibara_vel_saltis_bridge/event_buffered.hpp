#pragma once

#include <thread>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <cstring>

template<size_t Size>
class EventBuffered
{
    std::mutex m_lock;
    std::condition_variable var;
    bool start;
    uint8_t buffer[Size];

    public:

    EventBuffered()
    {
        this->start = false;
    }

    bool wait_for(int64_t seconds = 60 )
    {
        std::unique_lock<std::mutex> u_lock(this->m_lock);

        // this->var.wait(u_lock,[this]{ return this->start;});

        if(this->var.wait_for(u_lock,std::chrono::seconds(seconds)) == std::cv_status::no_timeout )
        {
            this->start = false;
            return true;
        }
        
        this->start = false;
        return false;
    }

    void wait()
    {
        std::unique_lock<std::mutex> u_lock(this->m_lock);

        this->var.wait(u_lock,[this]{ return this->start;});
        
        this->start = false;
    }

    void notify(const uint8_t* data)
    {
        std::unique_lock<std::mutex> u_lock(this->m_lock);

        this->start = true;

        for(size_t i=0;i<Size;++i)
        {
            this->buffer[i] = data[i];
        }

        this->var.notify_one();
    }

    uint8_t operator[](size_t i)
    {
        if(i>=Size)
        {
            return 0;
        }

        return this->buffer[i];
    }
};