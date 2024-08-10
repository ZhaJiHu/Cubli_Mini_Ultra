#pragma once

#include <Arduino.h>

#include "comm/time.h"
#include "imu/imu_driver_base.h"

using namespace CubliMini::ImuDriver;
namespace CubliMini {
namespace Imu {

// #define Kp 5.0f   // proportional gain governs rate of convergence to accelerometer/magnetometer
// #define Ki 0.01f  // integral gain governs rate of convergence of gyroscope biases

struct Q_t
{
    volatile float q0;
    volatile float q1;
    volatile float q2;
    volatile float q3;
};

class AHRS
{
   public:
    AHRS(ImuDriverBase *imu_driver, float imu_static_gyro_threshold);

    void AhrsLoop();
    bool IsStatic() const { return is_static_; }
    float ConvAngle(float raw_angle);

   public:
    ImuData_t imu_data_;

   private:
    ImuDriverBase *imu_driver_;
    Q_t q_;
    CubliMini::Comm::Time time_;
    float kp_;
    float ki_;
    bool is_static_;
    float imu_static_gyro_threshold_;

   private:
    void RunImuIsStatic();
    float invSqrt(float x);
    void ImuAHRSUpdate(Q_t &_q, const ImuData_t &_imu_data);
    void ConvertToEulerAmgleByQ(EulerAngle_t &_angle, const Q_t &_q);
};

}  // namespace Imu
}  // namespace CubliMini