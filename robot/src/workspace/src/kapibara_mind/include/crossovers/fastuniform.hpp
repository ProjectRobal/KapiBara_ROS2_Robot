#pragma once

#include "crossover.hpp"
#include "simd_vector.hpp"

namespace snn
{
    class FastUniform : public Crossover
    {
        SIMDVector mask;
        SIMDVector invmask;
        public:

        FastUniform(size_t max_size)
        {
            int indicator=0;

            while(max_size--)
            {
                mask.append(indicator);
                indicator=1-indicator;
            }

            invmask=(-mask)+1;
        }

        SIMDVector cross(const SIMDVector& a,const SIMDVector& b)
        {
            return a*this->mask + b*this->invmask;
        }
    };
}