#include "tb6612_drive/pwm_interface.hpp"

extern "C"
{

    #include <sys/stat.h>
    #include <sys/types.h>
    #include <fcntl.h>
    #include <stdio.h>
    #include <stdlib.h>
    #include <unistd.h>
    #include <poll.h>

}

namespace gpio
{
    
    PWM::PWM(const std::string& pwm_channel)
    {
        this->setup(pwm_channel);
    }

    void PWM::setup(const std::string& pwm_channel)
    {
        // export pwm interface

        std::string pwm_path=PWM_EXPORT_PATH;

        int export_fd=open(pwm_path.c_str(),O_WRONLY);

        if( export_fd == -1 )
		{
			#ifdef DEBUG

				std::cerr<<"Cannot export pwm channel "<<pwm_channel<<std::endl;

			#endif 
			return;
		}

        write(export_fd,pwm_channel.c_str(),pwm_channel.size());

        close(export_fd);

        // open duty descriptor

        pwm_path=PWM_BASE_PATH;

        pwm_path+="pwm"+pwm_channel+"/duty_cycle";

        this->duty_fd=open(pwm_path.c_str(),O_WRONLY);

        // open period descriptor

        pwm_path=PWM_BASE_PATH;
        pwm_path+="pwm"+pwm_channel+"/period";

        this->period_fd=open(pwm_path.c_str(),O_WRONLY);

        // open enable descriptor

        pwm_path=PWM_BASE_PATH;
        pwm_path+="pwm"+pwm_channel+"/enable";

        this->enable_fd=open(pwm_path.c_str(),O_WRONLY);

        this->pwm_channel=pwm_channel;

    }

    void PWM::setDuty(const uint32_t& duty)
    {

        std::string duty_str=std::to_string(duty);

        write(this->duty_fd,duty_str.c_str(),duty_str.size());

    }

    void PWM::setPeroid(const uint32_t& period)
    {
        std::string period_str=std::to_string(period);

        write(this->period_fd,period_str.c_str(),period_str.size());
    }

    void PWM::enable()
    {
        write(this->enable_fd,"1",1);
    }

    void PWM::disable()
    {
        write(this->enable_fd,"0",1);
    }

    void PWM::shutdown()
    {
        close(this->enable_fd);
        close(this->period_fd);
        close(this->duty_fd);

        int unexport_fd=open(PWM_UNEXPORT_PATH,O_WRONLY);

		write(unexport_fd,this->pwm_channel.c_str(),this->pwm_channel.size());

		close(unexport_fd);
    }

    PWM::~PWM()
    {
        shutdown();
    }
}

