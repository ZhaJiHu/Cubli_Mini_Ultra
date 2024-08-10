#pragma once

#include <math.h>

namespace CubliMini {
namespace Control {

enum MotorControlMode
{
    TOUCH_e = 0,
    VELOCITY_e = 1,
    BRAKE_e = 2,
};

struct MotorTransMode
{
    uint8_t motor1_mode;
    uint8_t motor2_mode;
    uint8_t motor3_mode;

    static MotorTransMode Get(uint8_t value)
    {
        MotorTransMode this_mode;
        this_mode.motor1_mode = value & 0x03;
        this_mode.motor2_mode = (value >> 2) & 0x03;
        this_mode.motor3_mode = (value >> 4) & 0x03;
        return this_mode;
    }

    static uint8_t GetValue(MotorTransMode & other)
    {
        uint8_t value;
        value = other.motor1_mode | (other.motor2_mode << 2) | (other.motor3_mode << 4);
        return value;
    }
};

struct AxisSensor_t
{
    float angle;
    float gyro;
    float speed;
};

struct AxisParam_t
{
    float kp;
    float kv;
    float ks;
    float angle_offset;
};

struct ServoInitialPosition_t
{
    uint8_t ch1;
    uint8_t ch2;
    uint8_t ch3;
};

inline float AxisLqr(const AxisSensor_t &_sensor, const AxisParam_t &_param)
{
    return (_sensor.angle - _param.angle_offset) * _param.kp + _sensor.gyro * _param.kv +
            _sensor.speed * _param.ks;
}

}  // namespace Control
}  // namespace CubliMini