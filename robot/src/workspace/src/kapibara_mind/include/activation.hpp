#pragma once

#include "simd_vector.hpp"

namespace snn
{

    class Activation
    {
        public:

        /*
            Activation function
        */
        virtual inline void activate(SIMDVector& vec)=0;

        /*
            Inverse function of activation function
        */
        virtual inline void inverse(SIMDVector& vec)=0;
    };

}