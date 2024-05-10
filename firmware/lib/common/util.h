#ifndef UTIL_H
#define UTIL_H

#include <Arduino.h>

inline int sgni(int x)
{
    return (x > 0) - (x < 0);
}

inline float sgnf(float x)
{
    return (x > 0.0f) - (x < 0.0f);
}

#endif // UTIL_H