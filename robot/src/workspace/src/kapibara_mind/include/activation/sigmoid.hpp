#pragma once

#include "misc.hpp"

#include "activation.hpp"

namespace snn
{
    class Sigmoid : public Activation
    {
        
        public:

        inline void activate(SIMDVector& vec)
        {
            SIMDVector v=exp(vec);

            vec=v/(v+1);
        }

        inline void inverse(SIMDVector& vec)
        {
            
            
            
        }
    };
}