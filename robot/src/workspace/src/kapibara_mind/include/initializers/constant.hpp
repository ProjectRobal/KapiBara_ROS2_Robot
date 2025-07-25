#pragma once

#include <random>

#include "initializer.hpp"

#include "config.hpp"

namespace snn
{
    template<number Value>
    class ConstantInit
    {
        public:

        number init()
        {
            return Value;
        }
    };
}