#pragma once

#include <random>
#include <cmath>

#include "initializer.hpp"

#include "config.hpp"

namespace snn
{
    class HuInit : public Initializer
    {

        public:

        HuInit()
        {

        }

        void init(SIMDVector& vec,size_t N)
        {
            std::random_device rd; 

            // Mersenne twister PRNG, initialized with seed from previous random device instance
            std::mt19937 gen(rd()); 
            
            std::normal_distribution<number> gauss(0.f,std::sqrt(2.f/static_cast<number>(N)));

            for(size_t i=0;i<N;++i)
            {
                vec.append(gauss(gen));
            }
        }

        void init(number& n)
        {
            std::random_device rd; 

            // Mersenne twister PRNG, initialized with seed from previous random device instance
            std::mt19937 gen(rd()); 

            std::normal_distribution<number> gauss(0.f,std::sqrt(2.f));

            n=gauss(gen);
        }
    };
}