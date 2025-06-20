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

        block_t block[inputSize+1];

        // now since each sub block gets the same reward we will just them a pointer to it.
        long double reward;

        size_t collectd_weights;

        number last_value;

        public:

        size_t Id;

        BlockKAC()
        {
            this->last_value = 0.f;

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

                if( i!= inputSize )
                {

                    this->worker[i] = block.weights[block.id].weight;

                }

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

            return w1->reward < w2->reward;
        }


        void chooseWorkers()
        {
            // if(this->Id==1)
            // {
            //     std::cout<<"Choose workers"<<std::endl;
            // }
            size_t iter=0;

            if( this->reward >= 0 )
            { 
                return;
            }



            float switch_probability = std::min<float>(REWARD_TO_SWITCH_PROBABILITY*this->reward,MAX_SWITCH_PROBABILITY);

            // size_t step = inputSize/(static_cast<size_t>(switch_probability*inputSize) + 1);

            size_t step = static_cast<size_t>(1.0/switch_probability);

            size_t start = std::min(step,inputSize-1);

            size_t i = static_cast<size_t>(std::round(this->uniform(this->gen)*start));

            uint32_t block_step = static_cast<uint32_t>(std::round(this->uniform(this->gen)*Populus));


            block_t& bias = this->block[inputSize];

            bias.weights[bias.id].reward = this->curr_rewards[i];

            bias.id += (block_step+i);

            bias.id = bias.id % Populus;

            bias.swap_count++;


            for(;i<inputSize;i+=step)
            {


                block_t& _block = this->block[i];

                _block.weights[_block.id].reward = this->curr_rewards[i];

                _block.id += (block_step+i);

                _block.id = _block.id % Populus;

                if( i != inputSize )
                {

                    this->worker[i] = _block.weights[_block.id].weight;

                    this->curr_rewards[i] = _block.weights[_block.id].reward;

                }

                _block.swap_count++;


                // sort weights based on thier rewards

                if( _block.swap_count >= Populus*4 )
                {
                    
                    qsort(_block.weights,Populus,sizeof(weight_t),compare_weights);

                    // if( this->Id == 0 )
                    // {
                        
                    //     std::cout<<"Block: "<<i<<_block.weights[0].reward<<", "<<_block.weights[1].reward<<", "<<_block.weights[2].reward<<std::endl;
                    // }


                    // calculate weighted average of the weights, first half of the best population

                    number best_weight = 0;

                    const size_t best_population_count = Populus/2;

                    const float best_population_discount = 1.f/(static_cast<float>(best_population_count));

                    number sum = 0;

                    for(size_t w=0;w<best_population_count;++w)
                    {
                        number exped = 1/std::pow(2,w+1);

                        best_weight += ( _block.weights[w].weight * exped );

                        sum += exped;

                        _block.weights[w].reward = 0;
                    }

                    best_weight /= sum;


                    _block.weights[best_population_count].weight = best_weight;

                    _block.weights[best_population_count].reward = 0;


                    for(size_t w=best_population_count+1;w<Populus;++w)
                    {

                        number mutation = this->global.init();

                        _block.weights[w].weight = best_weight+mutation;

                        _block.weights[w].reward = 0;
                    }

                    // if(this->Id == 1)
                    // {
                    //     std::cout<<"Sorted, first element reward: "<<_block.weights[0].reward<<std::endl;
                    // }

                    _block.swap_count = 0;

                    _block.id = best_population_count;

                    if( i != inputSize )
                    {
                        this->worker[i] = _block.weights[_block.id].weight;

                        this->curr_rewards[i] = _block.weights[_block.id].reward;
                    }

                }

                iter++;

            }

            if( bias.swap_count >= Populus*4 )
                {
                    
                    qsort(bias.weights,Populus,sizeof(weight_t),compare_weights);

                    // if( this->Id == 0 )
                    // {
                        
                    //     std::cout<<"Block: "<<i<<_block.weights[0].reward<<", "<<_block.weights[1].reward<<", "<<_block.weights[2].reward<<std::endl;
                    // }


                    // calculate weighted average of the weights, first half of the best population

                    number best_weight = 0;

                    const size_t best_population_count = Populus/2;

                    const float best_population_discount = 1.f/(static_cast<float>(best_population_count));

                    number sum = 0;

                    for(size_t w=0;w<best_population_count;++w)
                    {
                        number exped = 1/std::pow(2,w+1);

                        best_weight += ( bias.weights[w].weight * exped );

                        sum += exped;

                        bias.weights[w].reward = 0;
                    }

                    best_weight /= sum;


                    bias.weights[best_population_count].weight = best_weight;

                    bias.weights[best_population_count].reward = 0;


                    for(size_t w=best_population_count+1;w<Populus;++w)
                    {

                        number mutation = this->global.init();

                        bias.weights[w].weight = best_weight+mutation;

                        bias.weights[w].reward = 0;
                    }

                    // if(this->Id == 1)
                    // {
                    //     std::cout<<"Sorted, first element reward: "<<_block.weights[0].reward<<std::endl;
                    // }

                    bias.swap_count = 0;

                    bias.id = best_population_count;

                }

            this->reward = 0;
        }

        void giveReward(long double reward) 
        {          
            this->reward += reward;
            this->curr_rewards += 0.9f*(reward/Populus);
        }

        number fire(const SIMDVectorLite<inputSize>& input)
        {
            this->last_value = ( this->worker*input ).reduce() + this->block[inputSize].weights[this->block[inputSize].id].weight;
            return this->last_value;// + this->bias;
        }
        
        SIMDVectorLite<inputSize> mult(const SIMDVectorLite<inputSize>& input)
        {
            return this->worker*input;
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