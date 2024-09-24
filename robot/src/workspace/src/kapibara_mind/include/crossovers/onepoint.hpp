#pragma once

#include "crossover.hpp"
#include "simd_vector.hpp"

namespace snn
{
    class OnePoint : public Crossover
    {
        const SIMDVector& compare_and_ret(const SIMDVector& a,const SIMDVector& b)
        {
            if(a.size()>b.size())
            {
                return b;
            }
            else
            {
                return a;
            }
        }

        public:

        OnePoint()
        {
            
        }


        SIMDVector cross(const SIMDVector& a,const SIMDVector& b)
        {
            SIMDVector output;

            size_t elem_count=std::min(a.size(),b.size());

            if(elem_count==1)
            {
                output=(a+b)/2;
            }

            size_t size=std::min(a.block_count(),b.block_count());

            size_t i=0;

            size_t mid_point=elem_count/(2*MAX_SIMD_VECTOR_SIZE);
            
            for(;i<mid_point;++i)
            {
                output.append(a.get_block(i));
            }

            if( elem_count%MAX_SIMD_VECTOR_SIZE != 0)
            {
                size_t p=(elem_count/2) % MAX_SIMD_VECTOR_SIZE;
                
                SIMD mid;
                
                size_t o=0;
                for(;o<p;++o)
                {
                    mid[o]=a.get_block(mid_point)[o];
                }

                for(;o<MAX_SIMD_VECTOR_SIZE;++o)
                {
                    mid[o]=b.get_block(mid_point)[o];
                }

                output.append(mid);
                
                i++;
            }

            for(;i<size;++i)
            {
                output.append(b.get_block(i));
            }

            output.copy_metadata(this->compare_and_ret(a,b));

            return output;
        }
    };
}