#pragma once

#include <iostream>
#include <cstdint>

namespace snn
{
    class Layer
    {
        protected:

        bool trainable;

        public:

        Layer()
        {
            trainable = true;
        }

        void set_trainable(bool _trainable)
        {
            this->trainable = _trainable;
        }

        bool get_trainable()
        {
            return this->trainable;
        }

        virtual void setup() = 0;

        virtual void applyReward(long double reward) = 0;

        virtual void shuttle() = 0;

        virtual int8_t load() = 0;

        virtual int8_t save() const = 0;

        virtual int8_t load(std::istream& in) = 0;

        virtual int8_t save(std::ostream& out) const = 0;

    };
}