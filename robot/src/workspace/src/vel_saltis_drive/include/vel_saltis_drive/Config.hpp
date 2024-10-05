#ifndef CONFIG_HPP
#define CONFIG_HPP

#include <string>
#include <cstdint>

#define PWM_PERIOD 20000000

namespace tb612_drive
{

    struct Config
    {
        // in nanoseconds it translets to 50Hz
        // const uint32_t pwm_period=20000000;
        float loop_rate = 30;
        uint32_t encoder_resolution = 4096;
        std::string left_wheel_name = "left_wheel";
        std::string right_wheel_name = "right_wheel";

        std::string can_device = "can0";

    };

}


#endif