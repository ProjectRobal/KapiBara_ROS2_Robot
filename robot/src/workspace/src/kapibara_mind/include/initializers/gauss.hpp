#pragma once

#include <random>

#include "initializer.hpp"

#include "config.hpp"

namespace snn
{
    template<number mean,number std>
    class GaussInit
    {

        std::normal_distribution<number> gauss;

        std::mt19937 gen; 

        public:

        GaussInit()
        {
            std::random_device rd;

            this->gen = std::mt19937(rd());

            this->gauss = std::normal_distribution<number>(mean,std);  
        }

        number init()
        {
            return this->gauss(this->gen);
        }
    };
}