#pragma once

#include "activation.hpp"

namespace snn
{
    class Linear : public Activation
    {
        public:

        inline void activate(SIMDVector& vec)
        {
            // do nothing, placeholder activation
        }

        inline void inverse(SIMDVector& vec)
        {
            // do nothing, placeholder activation
        }
    };
}