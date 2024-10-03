#ifndef WHEEL_HPP
#define WHEEL_HPP

#include <string>
#include <cmath>

#include "tb6612_drive/pwm_interface.hpp"
#include "tb6612_drive/gpio_interface.hpp"

#include "Config.hpp"

namespace tb612_drive
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

        gpio::PWM pwm;

        gpio::GPIO MotorA;
        gpio::GPIO MotorB;

        Wheel()
        {

        }

        Wheel(const std::string& name,const double& EncoderResolution,const std::string& gpioA,const std::string& gpioB,const std::string& PWMChannel)
        {
            setup(name,EncoderResolution,gpioA,gpioB,PWMChannel);
        }

        void setup(const std::string& name,const uint32_t& EncoderResolution,const std::string& gpioA,const std::string& gpioB,const std::string& PWMChannel)
        {
            this->name=name;
            this->EncoderToAngelRatio=(2*M_PI)/EncoderResolution;

            this->pwm.setup(PWMChannel);
            this->pwm.setPeroid(PWM_PERIOD);
            this->pwm.setDuty(0);
            
            this->MotorA.setup(gpioA,gpio::Direction::OUTPUT);
            this->MotorB.setup(gpioB,gpio::Direction::OUTPUT);
            
            this->cmd=0;
        }

        void setPower(const double& pwr)
        {
            if(pwr>0)
            {
                this->forward();
            }
            else if(pwr<0)
            {
                this->backward();
            }
            else
            {
                this->stop();
            }

            this->pwm.setDuty(abs(pwr)*PWM_PERIOD);

        }

        void forward()
        {
            this->MotorA.enable();
            this->MotorB.disable();
        }

        void backward()
        {
            this->MotorA.disable();
            this->MotorB.enable();
        }

        void stop()
        {
            this->MotorA.enable();
            this->MotorB.enable();
        }

        void update(const uint16_t& CurrentEncoderValue,const double& dt)
        {
            int16_t dEncoder = CurrentEncoderValue - this->lastEncoderValue;

            this->lastEncoderValue = CurrentEncoderValue;

            double dPos = dEncoder*this->EncoderToAngelRatio;

            this->position += dPos;

            this->velocity = dPos/dt;
        }


        double targetVelocity()
        {
            return this->cmd/this->EncoderToAngelRatio;
        }

    };

}

#endif