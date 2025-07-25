#pragma once

#include <deque>
#include <cmath>
#include <iostream>
#include <cstdint>

#include "simd_vector_lite.hpp"
#include "layer.hpp"
#include "layer_kac.hpp"
#include "block_kac.hpp"

#include "initializers/hu.hpp"

#include "layer_counter.hpp"
#include "activation/exp.hpp"



namespace snn
{
    template<size_t InputSize,size_t ActionCount,size_t PopulationSize = 20>
    class Attention : public Layer
    {
        // it will be used to decide how much information to pass to hidden state
        snn::LayerKAC<InputSize*2,2,PopulationSize,Exp> conv;

        // add information about position
        snn::LayerKAC<InputSize*2 + ActionCount*2,1,PopulationSize> attention;

        std::deque<snn::SIMDVectorLite<InputSize>> last_actions;

        // mask that clear positions in vector
        snn::SIMDVectorLite<InputSize*2 + ActionCount*2> clear_mask;

        // mask that clear everything except first position
        snn::SIMDVectorLite<InputSize*2 + ActionCount*2> clear_mask_action_only;


        snn::SIMDVectorLite<InputSize> hidden_state;


        BlockKAC<InputSize,PopulationSize,HuInit<InputSize>> W1;

        BlockKAC<InputSize,PopulationSize,HuInit<InputSize>> W2;

        size_t id;

        public:

        Attention()
        {
            this->id = LayerCounter::LayerIDCounter++ ;
        }

        snn::SIMDVectorLite<InputSize> process(const snn::SIMDVectorLite<InputSize>& input)
        {   
            // push current input to buffer
            // this->last_actions.pop_back()

            this->last_actions.push_front(input);

            if(this->last_actions.size()>ActionCount)
            {
                this->last_actions.pop_back();
            }


            snn::SIMDVectorLite<InputSize*2 + ActionCount*2> pair(0);


            snn::SIMDVectorLite<InputSize> output(0);


            number score_sum = 0;

            size_t action_pos = 0;

            snn::SIMDVectorLite<InputSize> act_w1(0);

            // calculate attention for each pair
            for(const auto& action : this->last_actions)
            {
                for(size_t i=0;i<InputSize;++i)
                {
                    pair[i] = action[i];
                }

                pair = pair * this->clear_mask_action_only;

                pair[action_pos + InputSize] = 1;
                
                size_t action1_pos = 0;

                act_w1 = this->W1.mult(action);

                for(const auto& action1 : this->last_actions)
                {
                    for(size_t j=0;j<InputSize;++j)
                    {
                        pair[j+InputSize+ActionCount] = action1[j];
                    }

                    pair = pair * this->clear_mask;

                    pair[action1_pos + InputSize*2 + ActionCount] = 1;


                    number score_out = std::exp(this->attention.fire(pair)[0]);

                    output += score_out*( act_w1 + this->W2.mult(action1) );

                    score_sum += score_out;
                    
                    action1_pos++;
                }

                action_pos++;
            }

            // I have added it to give some long term recurency
            snn::SIMDVectorLite<InputSize> calculated_attention = output/score_sum;

            snn::SIMDVectorLite<InputSize*2> to_conv;

            for(size_t i=0;i<InputSize;++i)
            {
                to_conv[i] = calculated_attention[i];
                to_conv[InputSize+i] = hidden_state[i];
            }

            snn::SIMDVectorLite<2>  conv_output = this->conv.fire(to_conv); 

            number mean_conv_output = conv_output.reduce();

            conv_output /= mean_conv_output;

            hidden_state = hidden_state*conv_output[1] + calculated_attention*conv_output[0];

            return hidden_state;
        }

        void setup()
        {
            this->clear_mask = snn::SIMDVectorLite<InputSize*2 + ActionCount*2>(0);

            this->clear_mask_action_only = snn::SIMDVectorLite<InputSize*2 + ActionCount*2>(0);

            this->hidden_state = snn::SIMDVectorLite<InputSize>(0);

            // we are going to use that mask to clear information of position encoding
            for(size_t i=0;i<InputSize+ActionCount;++i)
            {
                this->clear_mask[i] = 1;
            }

            for(size_t i=0;i<InputSize;++i)
            {
                this->clear_mask[i+ActionCount+InputSize] = 1;
            }


            // that mask clear everything except first part with action
            for(size_t i=0;i<InputSize;++i)
            {
                this->clear_mask_action_only[i] = 1;
            }


            conv.setup();
            attention.setup();

            W1.setup();
            W2.setup();

        }

        void applyReward(long double reward)
        {
            conv.applyReward(reward);
            attention.applyReward(reward);

            W1.giveReward(reward);
            W2.giveReward(reward);
        }

        void shuttle()
        {
            conv.shuttle();
            attention.shuttle();

            W1.chooseWorkers();
            W2.chooseWorkers();
        }

        int8_t load()
        {
            int8_t ret = 0;

            ret = conv.load();

            if(ret!=0)
            {
                return ret;
            }

            ret = attention.load();

            if(ret!=0)
            {
                return ret;
            }

            std::string filename = "layer_vecs_"+std::to_string(this->id)+".layer";

            std::ifstream file;

            file.open(filename,std::ios::in);

            if(!file.good())
            {
                return -3;
            }

            W1.load(file);
            W2.load(file);

            file.close();


            return 0;

        }

        int8_t save() const
        {
            int8_t ret = 0;

            ret = conv.save();

            if(ret!=0)
            {
                return ret;
            }

            ret = attention.save();

            if(ret!=0)
            {
                return ret;
            }

            std::string filename = "layer_vecs_"+std::to_string(this->id)+".layer";

            std::ofstream file;

            file.open(filename,std::ios::out);

            if(!file.good())
            {
                return -3;
            }

            W1.dump(file);
            W2.dump(file);

            file.close();

            return 0;
        }

        int8_t load(std::istream& in)
        {
            int8_t ret = 0;

            ret = conv.load(in);

            if(ret!=0)
            {
                return ret;
            }

            ret = attention.load(in);

            if(ret!=0)
            {
                return ret;
            }

            W1.load(in);
            W2.load(in);

            return 0;
        }

        int8_t save(std::ostream& out) const
        {
            int8_t ret = 0;

            ret = conv.save(out);

            if(ret!=0)
            {
                return ret;
            }

            ret = attention.save(out);

            if(ret!=0)
            {
                return ret;
            }

            W1.dump(out);
            W2.dump(out);

            return 0;
        }

    };

}