#ifndef CONFIG_HPP
#define CONFIG_HPP

#include <string>
#include <cstdint>

namespace vel_saltis_drive
{

    struct Config
    {
        // in nanoseconds it translets to 50Hz
        float loop_rate = 30;
        uint32_t encoder_resolution = 4096;
        std::string left_wheel_name = "left_wheel";
        std::string right_wheel_name = "right_wheel";
        std::string left_ear = "left_ear_joint";
        std::string right_ear = "right_ear_joint";

        std::string can_device = "can0";

    };

}


#endif        // const uint32_t pwm_period=20000000;
