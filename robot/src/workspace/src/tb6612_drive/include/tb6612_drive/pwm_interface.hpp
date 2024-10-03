#ifndef PWM_INTERFACE_HPP
#define PWM_INTERFACE_HPP

#include <string>
#include <iostream>

namespace gpio
{

    #define DEBUG

    constexpr const char* PWM_EXPORT_PATH="/sys/class/pwm/pwmchip2/export";
    constexpr const char* PWM_UNEXPORT_PATH="/sys/class/pwm/pwmchip2/unexport";
    constexpr const char* PWM_BASE_PATH="/sys/class/pwm/pwmchip2/";


    class PWM
    {

        private:

        int duty_fd;
        int period_fd;
        int enable_fd;

        std::string pwm_channel;

        public:

        PWM(){};

        PWM(const std::string& pwm_channel);

        // enable pwm interface
        void setup(const std::string& pwm_channel);

        void setDuty(const uint32_t& duty);
        void setPeroid(const uint32_t& period);
        void enable();
        void disable();
        void shutdown();

        ~PWM();
        

    };

}

#endif