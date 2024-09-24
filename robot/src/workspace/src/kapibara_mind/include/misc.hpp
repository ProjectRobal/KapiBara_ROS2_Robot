#pragma once

#include <cstdint>
#include <cmath>
#include <climits>
#include "simd_vector.hpp"

#include "config.hpp"

#define SERIALIZED_NUMBER_SIZE 2*sizeof(int64_t)

namespace snn
{
    // serialize number of type T, and return it as serialized stream
    template<typename T>
    char* serialize_number(const T& num)
    {
        int exp=0;

        int64_t mant=std::numeric_limits<std::int64_t>::max()*std::frexp(num,&exp);

        int64_t _exp=exp;

        char* output = new char[2*sizeof(int64_t)];
        
        memmove(output,reinterpret_cast<char*>(&mant),sizeof(int64_t));
        memmove(output+sizeof(int64_t),reinterpret_cast<char*>(&_exp),sizeof(int64_t));

        return output;
    }

    // deserialize number of type T
    template<typename T>
    number deserialize_number(char* data)
    {
        int64_t exp=0;
        int64_t mant=0;

        memmove(reinterpret_cast<char*>(&mant),data,sizeof(uint64_t));
        memmove(reinterpret_cast<char*>(&exp),data+sizeof(uint64_t),sizeof(uint64_t));

        return static_cast<T>(std::ldexp(static_cast<T>(mant) / std::numeric_limits<std::int64_t>::max() ,exp));
    }


    SIMDVector power(const SIMDVector& vec, size_t N)
        {
            SIMDVector out=vec;

            while(--N)
            {
                out=out*vec;
            }

            return out;
        }

        SIMDVector exp(const SIMDVector& vec)
        {
            size_t n=1;

            SIMDVector x1=vec;

            SIMDVector sum=x1+1;

            while(n < 20)
            {
                x1=x1*(vec/(++n));
                sum+=x1;
            };

            return sum;
            
        }

        SIMDVector simd_abs(const SIMDVector& vec)
        {
            SIMDVector check = ((vec<0)*-2)+1;

            return vec*check;     
            
        }
};