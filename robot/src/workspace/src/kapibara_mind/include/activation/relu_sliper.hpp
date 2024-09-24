#pragma once

#include "activation.hpp"

namespace snn
{
    class SlipyReLu : public Activation
    {
        public:

        inline void activate(SIMDVector& vec)
        {
            SIMDVector filtered=vec>0;

            vec=vec*filtered - ((filtered-1)*0.001);
        }

        inline void inverse(SIMDVector& vec)
        {
            // clear values that are less than zero
            SIMDVector filtered=vec>0;

            vec=vec*filtered;

            
        }
    };
}