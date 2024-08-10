#pragma once
#include <Arduino.h>

#include <MPU9250_WE.h>
#include "config/config.h"
#include "imu/imu_driver_base.h"

namespace CubliMini {
namespace ImuDriver {

#define MPU6500_ADDR 0x68

class Mpu6500Driver : public ImuDriverBase
{
   public:
    Mpu6500Driver(
        TwoWire *write,
        uint8_t addr = MPU6500_ADDR,
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
          int2_(int2),
          mpu6500_(write, addr)
    {}

    int Init() override
    {
        wire_ptr_->begin(sda_, scl_);
        if (!mpu6500_.init())
        {
            return -1;
        }

        mpu6500_.enableGyrDLPF();
        mpu6500_.setGyrDLPF(MPU6500_DLPF_1);
        mpu6500_.setAccDLPF(MPU6500_DLPF_1);
        mpu6500_.setSampleRateDivider(0);
        mpu6500_.setGyrRange(MPU6500_GYRO_RANGE_1000);
        mpu6500_.setAccRange(MPU6500_ACC_RANGE_4G);
        mpu6500_.enableAccDLPF(true);

        mpu6500_.autoOffsets();
        /*  Set the interrupt pin:
        *  MPU9250_ACT_LOW  = active-low
        *  MPU9250_ACT_HIGH = active-high (default) 
        */
        // mpu6500_.setIntPinPolarity(MPU9250_ACT_HIGH);
        // mpu6500_.enableIntLatch(true);
        // mpu6500_.enableClearIntByAnyRead(false);  
        // mpu6500_.enableInterrupt(MPU9250_DATA_READY);

        // pinMode(int1_, INPUT_PULLUP);
        // attachInterrupt(digitalPinToInterrupt(int1_), setFlag, RISING);

        init_ = true;
        delay(1000);
        CalGyroOffset();
        return 0;
    }
    bool Ready() override { return true; }

   protected:
    bool ReadRawData(ImuData_t &raw_data) override
    {
        xyzFloat acc = mpu6500_.getGValues();
        xyzFloat gyr = mpu6500_.getGyrValues();
        raw_data.acc.ax = acc.x;
        raw_data.acc.ay = acc.y;
        raw_data.acc.az = acc.z;
        raw_data.gyro.gx = gyr.x;
        raw_data.gyro.gy = gyr.y;
        raw_data.gyro.gz = gyr.z;
        return true;
    }

   private:
    TwoWire *wire_ptr_ = NULL;
    uint8_t addr_;
    int sda_;
    int scl_;
    int int1_;
    int int2_;
    MPU6500_WE mpu6500_;
};
}  // namespace ImuDriver
}  // namespace CubliMini