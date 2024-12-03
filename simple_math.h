#ifndef __CHARLES_SIMPLE_MATH_H__
#define __CHARLES_SIMPLE_MATH_H__

namespace charles_math
{

template <typename T>
bool intersect(const T& min1, const T& max1, const T& min2, const T& max2)
{
    if(max1 > min2 && min1 < max2)
    {
        return true;
    }
    if(max2 > min1 && min2 < max1)
    {
        return true;
    }
    return false;
}

template <typename T>
bool in_interval(const T& value, const T& bound1, const T& bound2)
{
    const T& lower = std::min(bound1, bound2);
    const T& upper = std::max(bound1, bound2);
    return (value >= lower && value <= upper);
}

};





#endif