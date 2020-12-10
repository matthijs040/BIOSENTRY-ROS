#ifndef MATH_UTILS_H
#define MATH_UTILS_H

#include "math.h"

double DegToRad(double degrees)
{
    return ( degrees / 180 * M_PI );
}

double RadToDeg(double radians)
{
    return ( radians / M_PI * 180 );
}

#endif // MATH_UTILS_H