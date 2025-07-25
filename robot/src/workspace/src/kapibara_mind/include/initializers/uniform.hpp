#pragma once

#include <random>

#include "initializer.hpp"

#include "config.hpp"

namespace snn
{
    template<number A,number B>
    class UniformInit
    {

        std::uniform_real_distribution<number> uniform;
        std::mt19937 gen; 

        public:

        UniformInit()
        {
            std::random_device rd; 

            this->gen = std::mt19937(rd());

            this->uniform = std::uniform_real_distribution<number> (A,B);
        }

        number init()
        {
            return this->uniform(this->gen);
        }
    };
}