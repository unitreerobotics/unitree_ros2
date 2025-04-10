#pragma once

namespace unitree
{
namespace common
{

double clamp(double value, double low, double high)
{
    if (value < low)
        return low;
    if (value > high)
        return high;
    return value;
}

}
}