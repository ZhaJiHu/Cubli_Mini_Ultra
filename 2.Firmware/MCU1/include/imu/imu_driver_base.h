#pragma once
#include <Arduino.h>

namespace CubliMini {
namespace ImuDriver {

struct RawGyro_t
{
    volatile int16_t gx;
    volatile int16_t gy;
    volatile int16_t gz;
};

struct RawAcc_t
{
    volatile int16_t ax;
    volatile int16_t ay;
    volatile int16_t az;
};

struct ImuRawData_t
{
    volatile int16_t temp;
    RawGyro_t gyro;
    RawAcc_t acc;
};

struct EulerAngle_t
{
    float roll;
    float pitch;
    float yaw;
};

struct Gyro_t
{
    float gx;
    float gy;
    float gz;
};

struct Acc_t
{
    float ax;
    float ay;
    float az;
};

struct ImuData_t
{
    bool is_static;
    float temp;
    Acc_t acc;
    Gyro_t gyro;
    EulerAngle_t angle;
};

inline SemaphoreHandle_t g_x_semaphore_imu_base;
inline SemaphoreHandle_t * GetImuBaseSemaphore()
{
    return &g_x_semaphore_imu_base;
}

inline void setFlag(void)
{
    printf("imu xSemaphoreGive\n");
    xSemaphoreGive(*GetImuBaseSemaphore());
}

// 行
#define MEAN_FILTER_ROWS 6
// 列
#define MEAN_FILTER_COLS 8

class ImuDriverBase
{
   public:
    ImuDriverBase(
        float imu_init_gyro_threshold,
        int mean_filter_rows = MEAN_FILTER_ROWS,
        int mean_filter_cols = MEAN_FILTER_COLS)
        : imu_init_gyro_threshold_(imu_init_gyro_threshold),
          init_(false),
          mean_filter_rows_(mean_filter_rows),
          mean_filter_cols_(mean_filter_cols)
    {
        memset(&gyro_offset_, 0, sizeof(gyro_offset_));
        memset(&mean_filter_fifo_, 0, sizeof(mean_filter_fifo_));
        g_x_semaphore_imu_base = xSemaphoreCreateBinary();
    }

    virtual ~ImuDriverBase() = default;
    virtual int Init()       = 0;
    virtual bool Ready()     = 0;

    void ReadData(ImuData_t &data)
    {
        if (ReadRawData(imu_raw_data_))
        {
            MeanFilter(imu_raw_data_);
        }
        data.acc.ax  = mean_imu_raw_data_.acc.ax;
        data.acc.ay  = mean_imu_raw_data_.acc.ay;
        data.acc.az  = mean_imu_raw_data_.acc.az;
        data.gyro.gx = mean_imu_raw_data_.gyro.gx - gyro_offset_.gx;
        data.gyro.gy = mean_imu_raw_data_.gyro.gy - gyro_offset_.gy;
        data.gyro.gz = mean_imu_raw_data_.gyro.gz - gyro_offset_.gz;
    }

    // void attachInterrupt(std::function<void(void)> intRoutine) { intRoutine_ = intRoutine; }

   protected:
    virtual bool ReadRawData(ImuData_t &) = 0;

    void CalGyroOffset()
    {
        bool cal_gyro_offset_seccess = false;
        ImuData_t offset_data;
        Gyro_t gyro_sum;
        uint32_t count    = 0;
        uint32_t cal_time = 0;
        if (!init_)
            return;
        printf("start cal gyro offset .");
        while (cal_gyro_offset_seccess == false)
        {
            ReadData(offset_data);
            if (fabs(offset_data.gyro.gx) < imu_init_gyro_threshold_ &&
                fabs(offset_data.gyro.gy) < imu_init_gyro_threshold_ &&
                fabs(offset_data.gyro.gz) < imu_init_gyro_threshold_)
            {
                count++;
                gyro_sum.gx += offset_data.gyro.gx;
                gyro_sum.gy += offset_data.gyro.gy;
                gyro_sum.gz += offset_data.gyro.gz;
                if(count % 150 == 0)
                {
                    printf(".");
                }
                if (count > 1500)
                {
                    cal_gyro_offset_seccess = true;
                    gyro_offset_.gx         = gyro_sum.gx / 1500;
                    gyro_offset_.gy         = gyro_sum.gy / 1500;
                    gyro_offset_.gz         = gyro_sum.gz / 1500;
                    printf(
                        "\nIMU: calibrate offset seccess, gx: %f gy: %f gz: %f\r\n",
                        gyro_offset_.gx,
                        gyro_offset_.gy,
                        gyro_offset_.gz);
                }
            }
            else
            {
                if (cal_time % 200 == 0)
                {
                    printf("IMU: calibrate offset fail, please keep imu static!\r\n");
                    printf(
                        "rt_gx: %f gy: %f gz: %f\r\n",
                        offset_data.gyro.gx,
                        offset_data.gyro.gy,
                        offset_data.gyro.gz);
                }
                count       = 0;
                gyro_sum.gx = 0;
                gyro_sum.gy = 0;
                gyro_sum.gz = 0;
            }

            if (cal_time > 10 * 1000)  // 10s
            {
                printf("IMU: caloffset fail, timeout!\r\n");
                gyro_offset_.gx = 0;
                gyro_offset_.gy = 0;
                gyro_offset_.gz = 0;
                break;
            }
            cal_time++;
            delay(3);
        }
    }

    void MeanFilter(ImuData_t &_imu_raw_data)
    {
        for (int rows = 0; rows < mean_filter_rows_; ++rows)
        {
            for (int cols = 1; cols < mean_filter_cols_; ++cols)
            {
                mean_filter_fifo_[rows][cols - 1] = mean_filter_fifo_[rows][cols];
            }
        }

        mean_filter_fifo_[0][mean_filter_cols_ - 1] = _imu_raw_data.acc.ax;
        mean_filter_fifo_[1][mean_filter_cols_ - 1] = _imu_raw_data.acc.ay;
        mean_filter_fifo_[2][mean_filter_cols_ - 1] = _imu_raw_data.acc.az;
        mean_filter_fifo_[3][mean_filter_cols_ - 1] = _imu_raw_data.gyro.gx;
        mean_filter_fifo_[4][mean_filter_cols_ - 1] = _imu_raw_data.gyro.gy;
        mean_filter_fifo_[5][mean_filter_cols_ - 1] = _imu_raw_data.gyro.gz;

        for (int rows = 0; rows < mean_filter_rows_; ++rows)
        {
            float sum = 0;
            for (int cols = 0; cols < mean_filter_cols_; ++cols)
            {
                sum += mean_filter_fifo_[rows][cols];
            }
            mean_filter_fifo_[rows][mean_filter_cols_] = sum / mean_filter_cols_;
        }

        mean_imu_raw_data_.acc.ax  = mean_filter_fifo_[0][mean_filter_cols_];
        mean_imu_raw_data_.acc.ay  = mean_filter_fifo_[1][mean_filter_cols_];
        mean_imu_raw_data_.acc.az  = mean_filter_fifo_[2][mean_filter_cols_];
        mean_imu_raw_data_.gyro.gx = mean_filter_fifo_[3][mean_filter_cols_];
        mean_imu_raw_data_.gyro.gy = mean_filter_fifo_[4][mean_filter_cols_];
        mean_imu_raw_data_.gyro.gz = mean_filter_fifo_[5][mean_filter_cols_];
    }

    bool init_;

   private:
    std::function<void(void)> intRoutine_;
    ImuData_t mean_imu_raw_data_;
    Gyro_t gyro_offset_;
    float mean_filter_fifo_[6][15];
    float imu_init_gyro_threshold_;
    int mean_filter_rows_;
    int mean_filter_cols_;
    ImuData_t imu_raw_data_;
};

}  // namespace ImuDriver
}  // namespace CubliMini
