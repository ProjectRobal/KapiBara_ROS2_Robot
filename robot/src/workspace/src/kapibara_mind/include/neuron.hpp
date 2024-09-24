#pragma once

#include <iostream>
#include <memory>
#include <fstream>

#include "simd_vector.hpp"
#include "initializer.hpp"
#include "crossover.hpp"
#include "mutation.hpp"

#include "config.hpp"

#include "misc.hpp"

namespace snn
{

    class Neuron
    {
        protected:

        long double score;
        size_t use_count;

        public:

        Neuron()
        : score(0.f),
        use_count(0)
        {}

        virtual std::shared_ptr<Neuron> crossover(std::shared_ptr<Crossover> cross,const Neuron& neuron)=0;

        virtual void mutate(std::shared_ptr<Mutation> mutate)=0;

        virtual void setup(std::shared_ptr<Initializer> init)=0;

        virtual void update(const SIMDVector& weight,const number& bias)=0;

        virtual SIMDVector fire(const SIMDVector& input)
        {
            return SIMDVector(0,1);
        }

        virtual number fire1(const SIMDVector& input)
        {
            return 0;
        }

        virtual const snn::SIMDVector& get_weights()=0;

        virtual void update_weights(const snn::SIMDVector& dweight)=0;

        virtual const number& get_bias()=0;

        virtual void update_bias(const number& b)=0;

        virtual size_t input_size()=0;

        virtual size_t output_size()=0;

        virtual void save(std::ofstream& out) const
        {
            char* data_score = snn::serialize_number<number>(this->score);

            out.write(reinterpret_cast<const char*>(data_score),SERIALIZED_NUMBER_SIZE);

            delete data_score;

            uint64_t used=this->use_count;

            char out_used[sizeof(uint64_t)]={0};

            memmove(out_used,reinterpret_cast<const char*>(&used),sizeof(uint64_t));

            out.write(out_used,sizeof(uint64_t));
        };

        virtual void load(std::ifstream& in)
        {
            char data_score[SERIALIZED_NUMBER_SIZE]={0};

            in.read(data_score,SERIALIZED_NUMBER_SIZE);

            this->score = snn::deserialize_number<number>(data_score);

            char used_in[sizeof(uint64_t)]={0};

            in.read(used_in,sizeof(uint64_t));

            uint64_t used=0;

            memmove(reinterpret_cast<char*>(&used),used_in,sizeof(uint64_t));

            this->use_count=used;
        };

        virtual void giveReward(const long double& score)
        {
            this->score+=score;
        }

        const long double& reward() const
        {
            return this->score;
        }

        virtual void use()
        {
            ++this->use_count;
        }

        const size_t& used() const
        {
            return this->use_count;
        }

        Neuron& operator++()
        {
            this->use();

            return *this;
        }

        virtual void reset()
        {
            this->score=0;
            this->use_count=0;
        }

        virtual bool operator < (const Neuron& neuron) const
        {
            return this->score < neuron.score;
        }

        virtual bool operator > (const Neuron& neuron) const
        {
            return this->score > neuron.score;
        }

        virtual bool operator <= (const Neuron& neuron) const
        {
            return this->score <= neuron.score;
        }

        virtual bool operator >= (const Neuron& neuron) const
        {
            return this->score >= neuron.score;
        }

        virtual bool operator == (const Neuron& neuron) const
        {
            return this->score == neuron.score;
        }

        virtual bool operator != (const Neuron& neuron) const
        {
            return this->score != neuron.score;
        }

        virtual ~Neuron(){}
    };

}