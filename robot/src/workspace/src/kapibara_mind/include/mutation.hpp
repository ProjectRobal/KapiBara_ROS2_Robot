#pragma once

#include "simd_vector.hpp"
#include "config.hpp"

namespace snn
{

    class Mutation
    {
        public:

        virtual void mutate(SIMDVector& vec)=0;

        virtual void mutate(number& num)=0;
    };

}