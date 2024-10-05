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
        const uint32_t pwm_period=20000000;
        float loop_rate = 30;
        uint32_t encoder_resolution = 4096;
        std::string left_wheel_name = "left_wheel";
        std::string right_wheel_name = "right_wheel";

        // it will Vector2 / Vector3 message
        std::string encoder_topic="/kapibara/encoder";

        std::string left_gpioA;
        std::string left_gpioB;

        std::string right_gpioA;
        std::string right_gpioB;

        double left_P=0.5f;
        double left_I=0.f;
        double left_D=4.f;

        double right_P=0.5f;
        double right_I=0.f;
        double right_D=4.f;

    };

}


#endif