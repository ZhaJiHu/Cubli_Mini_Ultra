#pragma once
#include <Arduino.h>

namespace CubliMini {
namespace Comm {

class ConvertAngularVelocity
{
   public:
    ConvertAngularVelocity() = default;
    void Loop(
        double theta_deg,
        double omega_x,
        double omega_y,
        double omega_z,
        float &yaw_rate,
        float &pitch_rate)
    {
        double theta = DegToRad(theta_deg);

        // 计算绕Y轴旋转的旋转矩阵的元素
        double R[3][3] = {
            {cos(theta),  0, sin(theta)},
            {0,           1, 0         },
            {-sin(theta), 0, cos(theta)}
        };

        double omega_IMU[3]    = {omega_x, omega_y, omega_z};
        double omega_global[3] = {0, 0, 0};

        // 进行矩阵乘法 R * omega_IMU = omega_global
        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                omega_global[i] += R[i][j] * omega_IMU[j];
            }
        }

        yaw_rate   = omega_global[2];  // 绕z轴的角速度
        pitch_rate = omega_global[0];  // 绕x轴的角速度
    }

   private:
    double DegToRad(double degrees) { return degrees * PI / 180.0; }
};

inline float Limit(float _data, float _limit)
{
    if (_data >= _limit)
    {
        _data = _limit;
    }
    if (_data <= -_limit)
    {
        _data = -_limit;
    }
    return _data;
}

}  // namespace Comm
}  // namespace CubliMini