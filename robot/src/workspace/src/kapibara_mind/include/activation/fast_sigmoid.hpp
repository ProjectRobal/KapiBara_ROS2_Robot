#pragma once

#include "misc.hpp"

namespace snn
{
    class FastSigmoid
    {
        
        public:

        static inline void activate(SIMDVector& vec)
        {

            vec=vec/(abs(vec)+1);
            
        }

        template<size_t Size>
        static inline void activate(SIMDVectorLite<Size>& vec)
        {
            vec=vec/(abs(vec)+1);   
        }

        // add boundary check
        static inline void inverse(SIMDVector& vec)
        {
            
            vec=(vec/(vec+1))+(vec/(-vec+1));
            
        }
    };
}