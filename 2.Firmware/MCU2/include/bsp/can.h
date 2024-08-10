#pragma once
#include <Arduino.h>
#include <CAN_config.h>
#include <ESP32CAN.h>

#include "comm/type.h"
#include "comm/time.h"
#include "config/config.h"

namespace Cubli {
namespace Bsp {

using namespace Cubli::Comm;

#define CAN_OFFLINE_COUNT 500  // 250次,0.5s

enum CanFrameId_e
{
    eCAN_SEND_MOTOR_SPEED_FRAME = 0x101,
    eCAN_GET_MOTOR_SPEED_FRAME  = 0x100,
    eCAN_GET_MOTOR_HEALT_FRAME  = 0x102,
    eCAN_GET_SERVO_MOTOR_FRAME  = 0x103
};

const bool k_BRAKE = true;
const bool k_NORMAL = false;

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
};

class CanDriver
{
   public:
    CanDriver()
    {
        rx_frame_count_ = 0;
        can_is_online_  = eOFF_LINE;
    }
    void Init(int _can_txd_pin, int _can_rxd_pin, CAN_speed_t _can_speed);

    void CanSendMotorSpeed(float _send_ch1_speed, float _send_ch2_speed, float _send_ch3_speed);
    bool CanGetMotorSpeed(float &_set_ch1_speed, float &_set_ch2_speed, float &_set_ch3_speed, uint8_t& motor_mode, uint8_t &control);

    CanStatus_e CanIsOnline();
    CanStatus_e can_is_online_;

   private:
    void SendMessage(uint32_t msg_id, uint8_t msggage[8]);

    uint32_t rx_frame_count_;
    Time2 lost_can_time_;
};

}  // namespace Bsp
}  // namespace Cubli