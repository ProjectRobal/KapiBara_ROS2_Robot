#pragma once

#include <random>

#include "mutation.hpp"

#include "config.hpp"


namespace snn
{
    class GaussMutation : public Mutation
    {
        std::normal_distribution<number> gauss;

        std::uniform_real_distribution<double> uniform;

        const double mutation_threshold;

        public:

        GaussMutation(number mean,number std,double _mutation_threshold)
        : gauss(mean,std),
        uniform(0.f,1.f),
        mutation_threshold(_mutation_threshold)
        {

        }

        void mutate(number& num)
        {
            std::random_device rd; 

            // Mersenne twister PRNG, initialized with seed from previous random device instance
            std::mt19937 gen(rd()); 

            if(uniform(gen)<this->mutation_threshold)
            {
                num+=gauss(gen);
            }
        }

        void mutate(SIMDVector& vec)
        {
            std::random_device rd; 

            // Mersenne twister PRNG, initialized with seed from previous random device instance
            std::mt19937 gen(rd()); 

            for(size_t i=0;i<vec.size();++i)
            {
                if(uniform(gen)<this->mutation_threshold)
                {
                    vec.set(vec[i]+gauss(gen),i);
                }
            }
        }
    };

}