#pragma once

#include <cmath>

#include <algorithm>

#include <string>

class Servo
{
    public:

    double angle;

    std::string name;

    Servo()
    {
        this->angle = 0.f;
    }
 
    void setAngle(double _angle)
    {
        this->angle = _angle;
    }

    double getAngle()
    {
        double out = this->angle*(255.f/M_PI);

        return std::max<double>(std::min<double>(out,255.f),0.f);
    }
};  