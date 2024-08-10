#pragma once
#include <Arduino.h>
#include "bsp/led.h"
#include "bsp/can.h"
#include "config/config.h"

using namespace CubliMini::Config;
namespace CubliMini {
namespace Bsp {

class BspDriver
{
   public:
    void Init()
    {
        can_.Init(CAN_TXD_PIN, CAN_RXD_PIN, CAN_SPEED_1000KBPS);
    }
    CanDriver can_;
};

} // namespace CubliMini
} // namespace Bsp