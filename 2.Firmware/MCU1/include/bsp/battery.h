#pragma once
#include <Arduino.h>

#include "bsp/adc.h"
#include "config/config.h"
namespace CubliMini {
namespace Bsp {  

class Battery
{
    public:
        Battery()
        {
            bat_voltage_.Init(BAT_VOLTAGE_PIN);
        }

        float GetBatVoltage()
        {
            int adc_data = bat_voltage_.GetAdcValue();
            return (float)adc_data  / (4096.0f / 3.3f) / (10.0f / (42.2f + 10.0f)) * 1.075f;
        }
    private:
        AdcDriver bat_voltage_;
};

}}