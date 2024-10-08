#ifndef WHEEL_HPP
#define WHEEL_HPP

#include <string>
#include <cmath>


#include "Config.hpp"

namespace vel_saltis_drive 
{

    class Wheel
    {
        public:

        std::string name;
        double radius;

        uint32_t lastEncoderValue;

        double EncoderToAngelRatio;

        double position;
        double cmd;

        double velocity;

        Wheel()
        {

        }

        Wheel(const std::string& name,const double EncoderResolution)
        {
            setup(name,EncoderResolution);
        }

        void setup(const std::string& name,const uint32_t EncoderResolution)
        {
            this->name=name;
            this->EncoderToAngelRatio=(2*M_PI)/EncoderResolution;
            
            this->cmd=0;
        }


        void update(const uint16_t CurrentEncoderValue,const double dt)
        {
            int16_t dEncoder = CurrentEncoderValue - this->lastEncoderValue;

            this->lastEncoderValue = CurrentEncoderValue;

            double dPos = dEncoder*this->EncoderToAngelRatio;

            this->position += dPos;

            this->velocity = dPos/dt;
        }


        double targetVelocity()
        {
            return this->cmd;///this->EncoderToAngelRatio;
        }

    };

}

#endif