#pragma once


#include <vector>
#include <memory>

#include "simd_vector.hpp"
#include "layer_proto.hpp"

#include "config.hpp"

namespace snn
{
    class Network
    {

        std::vector<std::shared_ptr<LayerProto>> layers;

        public:

        Network()
        {}

        const std::vector<std::shared_ptr<LayerProto>>& getLayers() const
        {
            return this->layers;
        }


        void addLayer(std::shared_ptr<LayerProto> layer)
        {
            this->layers.push_back(layer);
        }

        void clear()
        {
            this->layers.clear();
        }

        SIMDVector fire(SIMDVector input)
        {
            
            for(auto layer : layers)
            {
                layer->shuttle();
                input = layer->fire(input);
            }

            return input;
        }      

        void applyReward(number reward)
        {
            reward=reward/this->layers.size();

            for(auto layer : layers)
            {
                layer->applyReward(reward);
            }
        }

    };

};