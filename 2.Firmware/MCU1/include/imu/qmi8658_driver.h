#pragma once
#include <Arduino.h>

#include "SensorQMI8658.hpp"
#include "config/config.h"
#include "imu/imu_driver_base.h"

namespace CubliMini {
namespace ImuDriver {

// inline void setFlag(void)
// {
//     xSemaphoreGive(*GetImuBaseSemaphore());
// }

class Qmi8658Driver : public ImuDriverBase
{
   public:
    Qmi8658Driver(
        TwoWire *write,
        uint8_t addr = QMI8658_H_SLAVE_ADDRESS,
        int sda      = IMU_SDA_PIN,
        int scl      = IMU_SCL_PIN,
        int int1     = IMU_INTERRUPT_1_PIN,
        int int2     = IMU_INTERRUPT_2_PIN)
        : ImuDriverBase(IMU_INIT_GYRO_THRESHOLD),
          wire_ptr_(write),
          addr_(addr),
          sda_(sda),
          scl_(scl),
          int1_(int1),
          int2_(int2)
    {}

    int Init() override
    {
        // int cs, int mosi = -1, int miso = -1, int sck = -1,
        if (!qmi_.begin(sda_, int1_, int2_, scl_))
        // if (!qmi_.begin(*wire_ptr_, addr_, sda_, scl_))
        {
            return -1;
        }
        // pinMode(int1_, INPUT_PULLUP);
        // pinMode(int2_, INPUT_PULLUP);
        // attachInterrupt(int1_, setFlag, RISING);
        // attachInterrupt(int2_, setFlag, RISING);

        qmi_.configAccelerometer(
            SensorQMI8658::ACC_RANGE_2G,
            SensorQMI8658::ACC_ODR_1000Hz,
            SensorQMI8658::LPF_MODE_2,
            true);
        qmi_.configGyroscope(
            SensorQMI8658::GYR_RANGE_512DPS,
            SensorQMI8658::GYR_ODR_896_8Hz,
            SensorQMI8658::LPF_MODE_2,
            true);
        qmi_.enableGyroscope();
        qmi_.enableAccelerometer();
        // qmi_.disableSyncSampleMode();
        // qmi_.enableINT(SensorQMI8658::IntPin1);
        // qmi_.enableINT(SensorQMI8658::IntPin2);
        qmi_.dumpCtrlRegister();
        init_ = true;
        delay(1000);
        CalGyroOffset();
        return 0;
    }
    bool Ready() override { return qmi_.getDataReady(); }

   protected:
    bool ReadRawData(ImuData_t &raw_data) override
    {
        float ax, ay, az;
        bool is_read_data = false;
        if (qmi_.getAccelerometer(ax, ay, az))
        {
            raw_data.acc.ax = ax;
            raw_data.acc.ay = ay;
            raw_data.acc.az = az;
            is_read_data = true;
        }
        float gx, gy, gz;
        if (qmi_.getGyroscope(gx, gy, gz))
        {
            raw_data.gyro.gx = gx;
            raw_data.gyro.gy = gy;
            raw_data.gyro.gz = gz;
            is_read_data = true;
        }
        return is_read_data;
    }

   private:
    TwoWire *wire_ptr_ = NULL;
    uint8_t addr_;
    int sda_;
    int scl_;
    int int1_;
    int int2_;
    SensorQMI8658 qmi_;
};
}  // namespace ImuDriver
}  // namespace CubliMini