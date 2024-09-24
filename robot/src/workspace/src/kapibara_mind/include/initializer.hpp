#pragma once

#include <simd_vector.hpp>

#include <config.hpp>

namespace snn
{
    
    class Initializer
    {
        public:

        virtual void init(SIMDVector& vec,size_t N)=0;

        virtual void init(number& n)=0;
    };

}