#ifndef CONFIG_HPP
#define CONFIG_HPP

#include <string>
#include <cstdint>

namespace vel_saltis_ears
{

    struct Config
    {
        // in nanoseconds it translets to 50Hz
        float loop_rate = 30;
        std::string left_ear_name = "left_ear";
        std::string right_ear_name = "right_ear";

        std::string can_device = "can0";

    };

}


#endif        // const uint32_t pwm_period=20000000;
