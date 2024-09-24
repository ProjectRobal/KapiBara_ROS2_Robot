#pragma once

#include "crossover.hpp"
#include "simd_vector.hpp"

namespace snn
{
    class FastOnePoint : public Crossover
    {

        SIMDVector mask1;
        SIMDVector mask2;        

        public:

        FastOnePoint(size_t max_size)
        {
            size_t i=0;

            for(;i<max_size/2;++i)
            {
                mask1.append(1);
            }

            for(;i<max_size;++i)
            {
                mask1.append(0);
            }

            mask2=(-mask1)+1;
        }

        SIMDVector cross(const SIMDVector& a,const SIMDVector& b)
        {
            return a*mask1 + b*mask2;
        }
    };
}