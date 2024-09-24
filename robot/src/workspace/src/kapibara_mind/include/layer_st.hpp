#pragma once

#include <vector>
#include <functional>
#include <fstream>

#include "block.hpp"
#include "neuron.hpp"
#include "initializer.hpp"
#include "mutation.hpp"
#include "crossover.hpp"

#include "layer_proto.hpp"

#include "simd_vector.hpp"

#include "config.hpp"

#include "activation.hpp"
#include "activation/linear.hpp"

#include "layer_utils.hpp"

namespace snn
{
    #define LAYERST 2

    template<class NeuronT>
    class LayerST
    { 
        std::vector<NeuronT> neurons;
        std::shared_ptr<Initializer> init;
        std::shared_ptr<Activation> activation_func;

        public:

        LayerST()
        {
            this->activation_func=std::make_shared<Linear>();
        }

        LayerST(size_t N,std::shared_ptr<Initializer> init)
        {
            this->setup(N,init);
        }

        void setInitializer(std::shared_ptr<Initializer> init)
        {
            this->init=init;
        }

        void setActivationFunction(std::shared_ptr<Activation> active)
        {
            this->activation_func=active;
        }

        void setup(size_t N,std::shared_ptr<Initializer> init)
        {
            this->activation_func=std::make_shared<Linear>();
            neurons.clear();

            this->init=init;

            for(size_t i=0;i<N;++i)
            {
                neurons.push_back(NeuronT());
                neurons.back().setup(init);
            }
        }

        void updateWeights(const std::vector<SIMDVector>& weights,const std::vector<number>& biases)
        {
            for(size_t i=0;i<neurons.size();++i)
            {
                this->neurons[i].update(weights[i],biases[i]);
                i++;
            }
        }

        SIMDVector fire(const SIMDVector& input)
        {
            SIMDVector output;
            //output.reserve(this->neurons[0].outputSize());

            for(auto& neuron : this->neurons)
            {
                
                output.append(neuron.fire1(input));

            }

            this->activation_func->activate(output);

            return output;
        }

        size_t getTypeID()
        {
            return 2;
        };

        void generate_metadata(nlohmann::json& j) const
        {

        }

        int8_t load(std::ifstream& in)
        {
            return -1;
        }

        int8_t save(std::ofstream& out) const
        {
            return -1;
        }

    };  
    
} // namespace snn
