#pragma once

#include "misc.hpp"

namespace snn
{
    class Exp
    {
        
        public:

        static inline void activate(SIMDVector& vec)
        {
            exp(vec);
        }

        template<size_t Size>
        static inline void activate(SIMDVectorLite<Size>& vec)
        {
            exp(vec);
        }

        static inline void inverse(SIMDVector& vec)
        {
            
        }
    };
}