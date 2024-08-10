#pragma once
#include "comm/comm.h"
#include "comm/time.h"

namespace CubliMini {
namespace Control {

class AngleOffsetCalibration
{
   public:
    AngleOffsetCalibration(float &cal_angle_kp)
        : cal_angle_kp_(cal_angle_kp),angle_update_step_size_(0.01), speed_threshold_(10), update_interval_(0.5), default_update_interval_(0.5)
    {}
    ~AngleOffsetCalibration() = default;
    // speed_threshold_: rad/s
    // update_interval: s
    // angle_update_step_size: angle
    void Loop(float speed, float &angle_offset, bool is_static, float angle) // angle value
    {
        static float last_temp_angle_offset = 0;
        if (!is_static)
        {
            time_cal_.reset();
            time_.reset();
            speed_dt_ = 0;
            reset_flag = true;
            return;
        }
        if (fabs(speed) < speed_threshold_)
        {
            time_cal_.reset();
            time_.reset();
            speed_dt_ = 0;
            return;
        }
        if(reset_flag)
        {
            update_interval_ = default_update_interval_ * 2;
            reset_flag = false;
        }
        speed_dt_ += speed * time_.GetTimeS();
        if (time_.GetTimeS() <= update_interval_)
        {
            return;
        }
        time_.reset();
        float temp_angle_offset = speed_dt_ * cal_angle_kp_;
        float detela = (temp_angle_offset - last_temp_angle_offset) * 0.0001;
        if(fabs(speed) < 20)
        {
            update_interval_ = default_update_interval_;
            time_cal_.reset();
            if (speed > 0)
            {
                angle_offset -= angle_update_step_size_;
            }
            if (speed < 0)
            {
                angle_offset += angle_update_step_size_;
            }
        }
        else
        {
            update_interval_ = default_update_interval_;
            angle_offset += temp_angle_offset;
        }
        speed_dt_ = 0;
        last_temp_angle_offset = temp_angle_offset;
    }

    float angle_update_step_size_;
    float speed_threshold_;
    float update_interval_;
    float default_update_interval_;
    bool reset_flag = true;
    float speed_dt_;
    float &cal_angle_kp_;
   private:
    Comm::CumulativeTime time_;
    Comm::CumulativeTime time_cal_;
};

}  // namespace Control
}  // namespace CubliMini