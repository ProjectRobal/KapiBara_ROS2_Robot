#ifndef GPIO_INTERFACE_HPP
#define GPIO_INTERFACE_HPP

#include <string>
#include <iostream>

namespace gpio
{
	//#define DEBUG

	constexpr const char* GPIO_EXPORT="/sys/class/gpio/export";
	constexpr const char* GPIO_UNEXPORT="/sys/class/gpio/unexport";
	constexpr const char* GPIO_BASE="/sys/class/gpio/";

	enum Direction
	{
		INPUT,
		OUTPUT
	};

	class GPIO
	{
		protected:

		int value_fd;

		std::string gpio_name;

		public:

		GPIO(){}

		GPIO(const std::string& gpio_name,Direction dir);		

		void setup(const std::string& gpio_name,Direction dir);
		
		bool state();

		void setState(bool state);

		void shutdown();

		void enable()
		{
			setState(true);
		}

		void disable()
		{
			setState(false);
		}

		operator bool()
		{
			return state();
		}

		~GPIO();

	};

}

#endif