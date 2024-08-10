#include <Arduino.h>

#include "config/config.h"

namespace CubliMini {
namespace Bsp {
class ServoMotor
{
    enum
    {
        INIT_DEGREE = 90
    };

   public:
    ServoMotor(int pin, int channel, int freq = 50, int resolution = 8)
        : pin_(pin), freq_(freq), resolution_(resolution), channel_(channel)
    {
        init();
        setDegree(INIT_DEGREE);
    }

    void re_init()
    {
        setDegree(INIT_DEGREE);
    }

    void setDegree(float degree)
    {
        // 对应0.5ms（0.5ms/(20ms/256）) 舵机转动角度与占空比的关系：(角度/90+0.5)*1023/20
        const float deadZone = 6.4;
        const float max      = 32;  // 对应2.5ms
        if (degree < 0)
            degree = 0;
        if (degree > 90)
            degree = 90;
        int temp_degree = (int)(((float)(max - deadZone) / 90.0f) * degree + deadZone);
        ledcWrite(channel_, temp_degree);
    }

   private:
    void init()
    {
        ledcSetup(channel_, freq_, resolution_);
        ledcAttachPin(pin_, channel_);
    }

    int channel_;
    int pin_;
    int freq_;
    int resolution_;
};
}  // namespace Bsp
}  // namespace CubliMini