#pragma once

#include "comm/comm.h"
#include "control/angle_offset_calibration.h"
#include "control/control_base.h"

namespace CubliMini {
namespace Control {

#define U_BALABCE_PROTECTION_THRESHOLD 20
#define U_BALABCE_OUTPUT_LIMIT         10

class UBalanceControl
{
   public:
    UBalanceControl(AxisParam_t &lqr_param, float &cal_angle_kp)
        : lqr_param_(lqr_param),
          cube_is_upside_dowm_(false),
          output_limit_(U_BALABCE_OUTPUT_LIMIT),
          angle_protection_threahold_(U_BALABCE_PROTECTION_THRESHOLD),
          angle_offset_auto_cal_(cal_angle_kp)
    {}

    float Loop(const AxisSensor_t &sensor, bool is_static)
    {
        float value = 0.0f;
        if (is_static)
        {
            cube_is_upside_dowm_ = false;
        }

        if (fabs(sensor.angle - lqr_param_.angle_offset) < fabs(angle_protection_threahold_) &&
            cube_is_upside_dowm_ == false)
        {
            value = AxisLqr(sensor, lqr_param_);
        }
        else
        {
            cube_is_upside_dowm_ = true;
        }

        angle_offset_auto_cal_.Loop(-sensor.speed, lqr_param_.angle_offset, is_static, value);

        return Comm::Limit(value, output_limit_);
    }

   public:
    AxisParam_t &lqr_param_;
    float output_limit_;
    float angle_protection_threahold_;
    AngleOffsetCalibration angle_offset_auto_cal_;

   private:
    bool cube_is_upside_dowm_;
};

}  // namespace Control
}  // namespace CubliMini