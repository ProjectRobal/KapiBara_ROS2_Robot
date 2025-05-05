#pragma once

#include <iostream>
#include <cstdint>

namespace snn
{
    class Layer
    {
        public:

        virtual void setup() = 0;

        virtual void applyReward(long double reward) = 0;

        virtual void shuttle() = 0;

        virtual int8_t load() = 0;

        virtual int8_t save() const = 0;

        virtual int8_t load(std::istream& in) = 0;

        virtual int8_t save(std::ostream& out) const = 0;

    };
}