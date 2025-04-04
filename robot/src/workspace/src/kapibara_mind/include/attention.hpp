#pragma once

#include <deque>
#include <cmath>
#include <iostream>
#include <cstdint>

#include "simd_vector_lite.hpp"
#include "layer.hpp"
#include "layer_kac.hpp"

namespace snn
{
    template<size_t InputSize,size_t OutputSize,size_t ActionCount,size_t PopulationSize = 20>
    class Attention : public Layer
    {

        snn::LayerKAC<InputSize*2,OutputSize,PopulationSize> conv;

        snn::LayerKAC<InputSize*2,1,PopulationSize> attention;

        std::deque<snn::SIMDVectorLite<InputSize>> last_actions;

        std::deque<snn::SIMDVectorLite<OutputSize>> cached_attention;

        std::deque<number> scores;

        public:

        snn::SIMDVectorLite<OutputSize> process(const snn::SIMDVectorLite<InputSize>& input)
        {   
            // push current input to buffer
            this->last_actions.pop_back();
            this->last_actions.push_front(input);

            this->cached_attention.pop_back();
            this->scores.pop_back();

            number score = 0;


            snn::SIMDVectorLite<InputSize*2> pair;

            for(size_t i=0;i<InputSize;++i)
            {
                pair[i] = input[i];
            }

            snn::SIMDVectorLite<OutputSize> output;

            // calculate attention for each pair
            for(const auto& action : this->last_actions)
            {
                for(size_t j=0;j<InputSize;++j)
                {
                    pair[j+InputSize] = action[j];
                }

                number assigned_score = std::exp(this->attention.fire(pair)[0]);

                output += conv.fire(pair)*assigned_score;

                score += assigned_score;
            }

            this->cached_attention.push_back(output);
            this->scores.push_back(score);

            snn::SIMDVectorLite<OutputSize> calculated_attention;

            number sumed_score = 0;
            
            for(size_t i=0;i<ActionCount;++i)
            {
                calculated_attention += this->cached_attention[i];
                sumed_score += this->scores[i];
            }

            calculated_attention = calculated_attention / sumed_score;

            return calculated_attention;
        }

        void setup()
        {
            conv.setup();
            attention.setup();


            for(size_t i=0;i<ActionCount;++i)
            {
                snn::SIMDVectorLite<582> action;

                this->last_actions.push_back(action);

                snn::SIMDVectorLite<256> cache;

                this->cached_attention.push_back(cache);

                this->scores.push_back(0);
            }

        }

        void applyReward(long double reward)
        {
            conv.applyReward(reward);
            attention.applyReward(reward);
        }

        void shuttle()
        {
            conv.shuttle();
            attention.shuttle();
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

            return 0;
        }

    };

}