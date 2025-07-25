#pragma once

#include "misc.hpp"

namespace snn
{
    class Sigmoid
    {
        
        public:

        static inline void activate(SIMDVector& vec)
        {
            SIMDVector v=exp(vec);

            vec=v/(v+1);
        }

        template<size_t Size>
        static inline void activate(SIMDVectorLite<Size>& vec)
        {
            SIMDVectorLite<Size> v=exp(vec);

            vec=v/(v+1);
        }

        static inline void inverse(SIMDVector& vec)
        {
            
        }
    };
}