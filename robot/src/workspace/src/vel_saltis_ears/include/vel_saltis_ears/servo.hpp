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
        double out = this->angle*(M_PI/180.f);

        return std::max<double>(std::min<double>(out,180.f),0.f);
    }
};  