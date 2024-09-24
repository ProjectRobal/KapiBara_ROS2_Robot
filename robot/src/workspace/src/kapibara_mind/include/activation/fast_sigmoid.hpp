#pragma once

#include "misc.hpp"

#include "activation.hpp"

namespace snn
{
    class FastSigmoid : public Activation
    {
        
        public:

        inline void activate(SIMDVector& vec)
        {

            vec=vec/(abs(vec)+1);
            
        }

        // add boundary check
        inline void inverse(SIMDVector& vec)
        {
            
            vec=(vec/(vec+1))+(vec/(-vec+1));
            
        }
    };
}