#pragma once
#include <Arduino.h>
#include "bsp/led.h"
#include "bsp/can.h"
#include "config/config.h"

using namespace Cubli::Config;
namespace Cubli {
namespace Bsp {

class BspDriver
{
   public:
    void Init()
    {
        motor_green_led_.Init(GREEN_LED_PIN);
        motor_red_led_.Init(RED_LED_PIN);
        can_.Init(CAN_TXD_PIN, CAN_RXD_PIN, CAN_SPEED_1000KBPS);
    }
    LedDriver motor_green_led_;
    LedDriver motor_red_led_;
    CanDriver can_;
};

} // namespace Cubli
} // namespace Bsp