#pragma once

#include "misc.hpp"

namespace snn
{
    class SiLu
    {
        
        public:

        static inline void activate(SIMDVector& vec)
        {
            SIMDVector v=exp(vec);

            vec=(v/(v+1))*vec;
        }

        template<size_t Size>
        static inline void activate(SIMDVectorLite<Size>& vec)
        {
            SIMDVectorLite<Size> v=exp(vec);

            vec=(v/(v+1))*vec;
        }

        static inline void inverse(SIMDVector& vec)
        {
            
            
            
        }
    };
}