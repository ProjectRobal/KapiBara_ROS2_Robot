#pragma once

#include <algorithm>
#include <vector>
#include <array>
#include <memory>
#include <random>
#include <fstream>
#include <algorithm>

#include "simd_vector.hpp"
#include "simd_vector_lite.hpp"

#include "initializer.hpp"

#include "config.hpp"

#include "misc.hpp"

#include "initializers/hu.hpp"

namespace snn
{
    class BlockCounter
    {
        public:

        static size_t BlockID;

        BlockCounter()
        {
            this->BlockID++;
        }
    };

    template<size_t inputSize,size_t Populus,class weight_initializer=HuInit<inputSize>>
    class BlockKAC
    {   
        /*
            It will store each genome with a counter which indicate how many times entity was tested.
        */

        SIMDVectorLite<inputSize> worker;

        std::mt19937 gen; 

        weight_initializer global;

        std::uniform_real_distribution<float> uniform;

        typedef struct weight
        {
            number weight;
            number reward;
        } weight_t;

        // there will be inputsize amount of block
        typedef struct block
        {
            uint32_t id;
            uint32_t swap_count;
            // number rewards[Populus];
            // number weights[Populus];

            weight_t weights[Populus];
        } block_t;

        SIMDVectorLite<inputSize> best_weights;

        SIMDVectorLite<inputSize> curr_rewards;

        block_t block[inputSize];

        // now since each sub block gets the same reward we will just them a pointer to it.
        long double reward;

        size_t collectd_weights;

        public:

        size_t Id;

        BlockKAC()
        {
            this->reward = 0.f;

            std::random_device rd;

            this->gen = std::mt19937(rd());

            this->uniform = std::uniform_real_distribution<float>(0.f,1.f);

            this->worker = SIMDVectorLite<inputSize>(0);

            this->best_weights = SIMDVectorLite<inputSize>(0);

            this->curr_rewards = SIMDVectorLite<inputSize>(0);

            this->Id = BlockCounter::BlockID;

            this->collectd_weights = 0;

            // std::cout<<this->Id<<std::endl;

            BlockCounter::BlockID++;

        }

        void setup()
        {
            // create file if not exits

            // std::cout<<this->Id<<std::endl;

            size_t i=0;

            for(block_t& block : this->block)
            {
                block.id = static_cast<uint32_t>(std::round(this->uniform(this->gen)*(Populus-1)));
                block.swap_count = 0;

                for(size_t w=0;w<Populus;w++)
                {
                    block.weights[w].weight = this->global.init();
                    block.weights[w].reward = 0.f;
                }

                this->worker[i] = block.weights[block.id].weight;

                ++i;
            }
 
        }

        number give_new_weight(block_t& victim)
        {
            if( victim.collected_b_weight > 0 )
            {
                if( this->uniform(this->gen) >= 0.1f )
                {
                    return victim.best_weight_buffer / static_cast<number>(victim.collected_b_weight);
                }

                return (victim.best_weight_buffer / static_cast<number>(victim.collected_b_weight)) + this->global.init();
            }

            return this->global.init();
        }

        static int compare_weights(const void *p1, const void *p2)
        {
            const weight_t *w1 = (const weight_t*)p1;
            const weight_t *w2 = (const weight_t*)p2;

            return w1->reward > w2->reward;
        }


        void chooseWorkers()
        {
            // if(this->Id==1)
            // {
            //     std::cout<<"Choose workers"<<std::endl;
            // }
            size_t iter=0;

            if( this->reward == 0 )
            {
                return;
            }

            if( this->reward > 0 )
            {
                // iter = static_cast<size_t>(std::round(this->uniform(this->gen)*(inputSize/4 + 1)));

                // we only update network partially

                this->best_weights*= this->collectd_weights;

                this->best_weights += this->worker;

                this->collectd_weights++;

                this->best_weights /= this->collectd_weights;

                if( this->collectd_weights == 10 )
                {

                    this->best_weights /= this->collectd_weights;

                    this->collectd_weights = 1;
                }
                
                return;
            }

            float switch_probability = std::min<float>(REWARD_TO_SWITCH_PROBABILITY*this->reward,MAX_SWITCH_PROBABILITY);

            // size_t step = inputSize/(static_cast<size_t>(switch_probability*inputSize) + 1);

            size_t step = static_cast<size_t>(1.0/switch_probability);

            size_t start = std::min(step,inputSize-1);

            size_t i = static_cast<size_t>(std::round(this->uniform(this->gen)*start));

            uint32_t block_step = static_cast<uint32_t>(std::round(this->uniform(this->gen)*Populus));

            for(;i<inputSize;i+=step)
            {

                block_t& _block = this->block[i];

                _block.weights[_block.id].reward = this->curr_rewards[i];

                _block.id += (block_step+i);

                _block.id = _block.id % Populus;

                this->worker[i] = _block.weights[_block.id].weight;

                this->curr_rewards[i] = _block.weights[_block.id].reward;

                _block.swap_count++;


                // sort weights based on thier rewards

                if( _block.swap_count >= Populus*4 )
                {
                    qsort(_block.weights,Populus,sizeof(weight_t),compare_weights);

                    size_t w=0;

                    size_t nudge = this->uniform(this->gen) > 0.5 ? 1 : 0;

                    for(;w<Populus/2;++w)
                    {
                        if( this->best_weights[i] != 0 && (w+nudge) % 2 == 0 )
                        {

                            number new_best = 0.5f*_block.weights[_block.id].weight + 0.5f*this->best_weights[i];
                            _block.weights[w].weight = new_best;

                        }

                        number mutation_power = std::max(1.f - std::exp(this->curr_rewards[i]*0.0002f), 0.f);

                        number mutation = this->global.init()/10.f; 

                        _block.weights[w].weight += mutation*mutation_power;
                        _block.weights[w].reward = 0;

                    }

                    for(;w<Populus;++w)
                    {
                        _block.weights[w].reward = 0;
                    }

                    // if(this->Id == 1)
                    // {
                    //     std::cout<<"Sorted, first element reward: "<<_block.weights[0].reward<<std::endl;
                    // }

                    _block.swap_count = 0;

                }

                iter++;

            }

            this->reward = 0;
        }

        void giveReward(long double reward) 
        {          
            this->reward += reward;
            this->curr_rewards += reward/Populus;
        }

        number fire(const SIMDVectorLite<inputSize>& input)
        {
            SIMDVectorLite<inputSize> res = this->worker*input;
            return ( this->worker*input ).reduce();// + this->bias;
        }       


        void dump(std::ostream& out) const
        {
            // save best weights

            out.write((const char*)&this->collectd_weights,sizeof(size_t));

            for(size_t i=0;i<inputSize;++i)
            {
                char* buff = serialize_number<number>(this->best_weights[i]);

                out.write(buff,SERIALIZED_NUMBER_SIZE);

                delete [] buff;
            }

            out.write((char*)&this->block,sizeof(block_t)*inputSize);

        }

        void load(std::istream& in)
        {
            // load best weights
            in.read((char*)&this->collectd_weights,sizeof(size_t));

            char buff[SERIALIZED_NUMBER_SIZE];

            for(size_t i=0;i<inputSize;++i)
            {
                in.read(buff,SERIALIZED_NUMBER_SIZE);

                number num = deserialize_number<number>(buff);

                this->best_weights[i] = num;
            }


            // load blocks
            for(size_t i=0;i<inputSize;++i)
            {
                in.read((char*)&this->block[i],sizeof(block_t));

                this->worker[i] = this->block[i].weights[this->block[i].id].weight;
                this->curr_rewards[i] = this->block[i].weights[this->block[i].id].reward;
            }   
        }
        
    };
}