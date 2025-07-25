#pragma once

#include "misc.hpp"

namespace snn
{
    class SoftMax
    {
        
        public:

        static inline void activate(SIMDVector& vec)
        {
            SIMDVector v=exp(vec);

            number sum = v.reduce();

            vec = v / sum;
        }

        template<size_t Size>
        // think about it!
        static inline void activate(SIMDVectorLite<Size>& vec)
        {
            number max = vec[0];
            number min = vec[0];

            for(size_t i=0;i<vec.size();++i)
            {
                if(vec[i]>max)
                {
                    max = vec[i];
                }

                if(vec[i]<min)
                {
                    min = vec[i];
                }
            }

            vec = (vec - min)/(max - min);

            // SIMDVectorLite<Size> v=exp(vec);

            number sum = vec.reduce();

            vec = vec / sum;
        }

        static inline void inverse(SIMDVector& vec)
        {
            
            
            
        }
    };
}