#pragma once
#include <Arduino.h>

namespace Cubli {
namespace Comm {

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
}  // namespace Cubli