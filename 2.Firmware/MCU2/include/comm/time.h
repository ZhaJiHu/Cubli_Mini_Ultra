#pragma once
#include <Arduino.h>

namespace Cubli {
namespace Comm {

class Time
{
   public:
    Time()
    {
        rt_time_ = millis();
        last_time_ = millis();
    }
    void reset()
    {
        rt_time_ = millis();
        last_time_ = millis();
    }

    float GetTimeUs()
    {
        rt_time_ = micros();
        float ts = (rt_time_ - last_time_) * 1e-6;
        if (ts <= 0)
        {
            ts = 1e-3f;
        }
        last_time_ = rt_time_;
        return ts;
    }

    float GetTimeMs()
    {
        rt_time_ = millis();
        float ts = (rt_time_ - last_time_) * 1e-3;
        if (ts <= 0)
        {
            ts = 1e-3f;
        }
        last_time_ = rt_time_;
        return ts;
    }

    unsigned long GetRtTimeMs()
    {
        return millis();
    }

   private:
    long rt_time_;
    long last_time_;
};


class Time2
{
   public:
    Time2()
    {
        time_cout_ms_ = 0.0f;
        time_.reset();
    }

    void reset()
    {
        time_cout_ms_ = 0.0f;
        time_.reset();
    }

    float GetTimeS()
    {
        time_cout_ms_ += time_.GetTimeMs();
        return time_cout_ms_;
    }
   private:
    float time_cout_ms_;
    Time time_;
};

} // namespace Cubli
} // namespace Comm