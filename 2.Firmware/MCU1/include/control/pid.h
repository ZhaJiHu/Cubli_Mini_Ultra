#pragma once

#include "comm/comm.h"
namespace CubliMini {
namespace Control {

class Pid
{
   public:
    Pid(float kp, float ki, float kd)
        : kp_(kp), ki_(ki), kd_(kd), integral_(12), prev_error_(0)
    {}

    float compute(float setpoint, float current_value)
    {
        float error = setpoint - current_value;
        integral_ += error;
        integral_        = Comm::Limit(integral_, 20);
        float derivative = error - prev_error_;

        float output = kp_ * error + ki_ * integral_ + kd_ * derivative;

        prev_error_ = error;
        return output;
    }

    void reset()
    {
        integral_   = 0;
        prev_error_ = 0;
    }

   public:
    float kp_;
    float ki_;
    float kd_;
    float integral_;
    float prev_error_;
};
}}