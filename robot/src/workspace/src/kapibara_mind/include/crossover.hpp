#pragma once

#include "simd_vector.hpp"

namespace snn
{
    class Crossover
    {
        public:

        virtual SIMDVector cross(const SIMDVector& a,const SIMDVector& b)=0;

        SIMDVector operator()(const SIMDVector& a,const SIMDVector& b)
        {
            return this->cross(a,b);
        }
    };
} // namespace snn
