#pragma once

#include "comm/comm.h"
#include "control/angle_offset_calibration.h"
#include "control/control_base.h"

namespace CubliMini {
namespace Control {

#define P_BALABCE_PROTECTION_THRESHOLD 20
#define P_BALABCE_OUTPUT_LIMIT         15

class PBalanceControl
{
   public:
    struct PAxisParam
    {
        AxisParam_t x;
        AxisParam_t y;
        AxisParam_t z;
    };

    struct PSensor
    {
        AxisSensor_t x;
        AxisSensor_t y;
        AxisSensor_t z;
    };

    struct MotorSpeed
    {
        float ch1;
        float ch2;
        float ch3;
        uint8_t value;
        uint8_t control_mode;
    };

    struct AxisSpeed
    {
        float x;
        float y;
        float z;
    };

    using AxisControl  = AxisSpeed;
    using MotorControl = MotorSpeed;

    PBalanceControl(PAxisParam &lqr_param, float &cal_angle_kp)
        : lqr_param_(lqr_param),
          cube_is_upside_dowm_(false),
          output_limit_(P_BALABCE_OUTPUT_LIMIT),
          angle_protection_threahold_(P_BALABCE_PROTECTION_THRESHOLD),
          x_angle_offset_auto_cal_(cal_angle_kp),
          y_angle_offset_auto_cal_(cal_angle_kp)
    {}

    MotorControl Loop(PSensor &sensor, const MotorSpeed &motor_speed, bool is_static, float yaw_control)
    {
        MotorControl motor_control = {0, 0, 0};
        if (is_static)
        {
            cube_is_upside_dowm_ = false;
        }

        if (fabs(sensor.x.angle - lqr_param_.x.angle_offset) < fabs(angle_protection_threahold_) &&
            fabs(sensor.y.angle - lqr_param_.y.angle_offset) < fabs(angle_protection_threahold_) &&
            cube_is_upside_dowm_ == false)
        {
            AxisSpeed axis_speed;
            GetAxisMoveSpeed(motor_speed, axis_speed);
            sensor.x.speed = axis_speed.x;
            sensor.y.speed = axis_speed.y;
            sensor.z.speed = axis_speed.z;

            AxisControl axis_control;
            axis_control.x = AxisLqr(sensor.x, lqr_param_.x);
            axis_control.y = AxisLqr(sensor.y, lqr_param_.y);
            axis_control.z = AxisLqr(sensor.z, lqr_param_.z) + yaw_control;
            GetMotorMoveSpeed(axis_control, motor_control);

            x_angle_offset_auto_cal_.Loop(-axis_speed.x, lqr_param_.x.angle_offset, is_static, sensor.x.angle);
            // y_angle_offset_auto_cal_.Loop(-axis_speed.y, lqr_param_.y.angle_offset, is_static, sensor.y.angle);
        }
        else
        {
            motor_control.ch1    = 0;
            motor_control.ch2    = 0;
            motor_control.ch3    = 0;
            cube_is_upside_dowm_ = true;
        }

        motor_control.ch1 = Comm::Limit(motor_control.ch1, output_limit_);
        motor_control.ch2 = Comm::Limit(motor_control.ch2, output_limit_);
        motor_control.ch3 = Comm::Limit(motor_control.ch3, output_limit_);
        return motor_control;
    }

   private:
    void GetAxisMoveSpeed(const MotorSpeed &_get_speed, AxisSpeed &_axis_speed)
    {
        _axis_speed.x = _get_speed.ch2 - _get_speed.ch3 * sin((30) / 57.3f) -
                        _get_speed.ch1 * sin((30) / 57.3f);
        _axis_speed.y = (_get_speed.ch3 - _get_speed.ch1) * cos((30) / 57.3f);
        _axis_speed.z = (_get_speed.ch2 + _get_speed.ch3 + _get_speed.ch1);
    }

    void GetMotorMoveSpeed(const AxisControl &axis_control, MotorControl &motor_control)
    {
        motor_control.ch1 = -axis_control.y * cos((30) / 57.3f) -
                            axis_control.x * sin((30) / 57.3f) + axis_control.z / 3.0f;
        motor_control.ch2 = axis_control.x + axis_control.z / 3.0f;
        motor_control.ch3 = -axis_control.x * sin((30) / 57.3f) +
                            axis_control.y * cos((30) / 57.3f) + axis_control.z / 3.0f;
    }

   public:
    PAxisParam &lqr_param_;
    float output_limit_;
    float angle_protection_threahold_;
    AngleOffsetCalibration x_angle_offset_auto_cal_;
    AngleOffsetCalibration y_angle_offset_auto_cal_;

   private:
    bool cube_is_upside_dowm_;
};

}  // namespace Control
}  // namespace CubliMini