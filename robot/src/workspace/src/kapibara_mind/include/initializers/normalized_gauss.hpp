#pragma once

#include <random>

#include "initializer.hpp"

#include "config.hpp"

namespace snn
{
    class NormalizedGaussInit : public Initializer
    {

        std::normal_distribution<number> gauss;

        public:

        NormalizedGaussInit(number mean,number std)
        : gauss(mean,std)
        {

        }

        void init(SIMDVector& vec,size_t N)
        {
            std::random_device rd; 

            // Mersenne twister PRNG, initialized with seed from previous random device instance
            std::mt19937 gen(rd()); 

            for(size_t i=0;i<N;++i)
            {
                vec.append(this->gauss(gen));
            }

            number mean=vec.dot_product();

            if(mean==0)
            {
                mean=1;
            }

            vec/=mean;
        }

        void init(number& n)
        {
            std::random_device rd; 

            // Mersenne twister PRNG, initialized with seed from previous random device instance
            std::mt19937 gen(rd()); 

            n=this->gauss(gen);

            n=n/abs(n);
        }
    };
}