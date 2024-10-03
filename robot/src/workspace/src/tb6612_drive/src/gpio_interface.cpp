#include "tb6612_drive/gpio_interface.hpp"

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

#include <exception>


namespace gpio
{

    GPIO::GPIO(const std::string& gpio_name,Direction dir)
    {
        this->setup(gpio_name,dir);
    }

    void GPIO::setup(const std::string& gpio_name,Direction dir)
		{
			// export gpio to use it
			int fd=open(GPIO_EXPORT,O_WRONLY);

			if( fd == -1 )
			{
				#ifdef DEBUG

					std::cerr<<"Cannot export gpio!"<<std::endl;

				#else

					throw std::runtime_error("Cannot export gpio: "+gpio_name+" !");

				#endif
				return;
			}

			write(fd,gpio_name.c_str(),gpio_name.size());

			close(fd);

			std::string gpio_path=GPIO_BASE;
			gpio_path+="gpio"+gpio_name+"/direction";

			// set direction
			fd=open(gpio_path.c_str(),O_WRONLY);

			if( fd == -1 )
			{
				#ifdef DEBUG

					std::cerr<<"Cannot set gpio direction! name: "<<gpio_name<<std::endl;

				#else

					throw std::runtime_error("Cannot set gpio direction! name: "+gpio_name+" !");

				#endif
				return;
			}

			switch(dir)
			{
				case Direction::INPUT:
					write(fd,"in",2);
				break;

				case Direction::OUTPUT:
					write(fd,"out",3);
				break;
			}

			close(fd);

			// open value file descriptor

			gpio_path=GPIO_BASE;
			gpio_path+="gpio"+gpio_name+"/value";

			this->value_fd=open(gpio_path.c_str(),O_RDWR);

			if( fd == -1 )
			{
				#ifdef DEBUG

					std::cerr<<"Cannot open gpio value!"<<gpio_name<<std::endl;

				#else

					throw std::runtime_error("Cannot open gpio value! name: "+gpio_name+" !");

				#endif
			}

			this->gpio_name=gpio_name;
			
		}

    bool GPIO::state()
    {
        char c;

        read(this->value_fd,&c,1);

        return c-48;
    }

	void GPIO::setState(bool state)
    {
        char c=48+state;

        write(this->value_fd,&c,1);
    }

	void GPIO::shutdown()
	{
		close(this->value_fd);

		int unexport_fd=open(GPIO_UNEXPORT,O_WRONLY);

		write(unexport_fd,this->gpio_name.c_str(),this->gpio_name.size());

		close(unexport_fd);
	}

    GPIO::~GPIO()
    {
		shutdown();	
    }
}