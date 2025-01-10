#pragma once

#include <random>
#include <cmath>

#include "initializer.hpp"

#include "config.hpp"

namespace snn
{
    template<size_t inputSize>
    class HuInit
    {

        std::mt19937 gen; 

        std::normal_distribution<number> global;

        public:

        HuInit()
        {
            std::random_device rd;

            this->gen = std::mt19937(rd());

            this->global = std::normal_distribution<number>(0,std::sqrt(2.f/static_cast<number>(inputSize)));
        }

        number init()
        {
            return this->global(this->gen);
        }

    };
}