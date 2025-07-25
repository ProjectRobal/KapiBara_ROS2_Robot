#pragma once

#include <cstdint>

#include "simd_vector_lite.hpp"
#include "layer.hpp"
#include "layer_kac.hpp"

#include "activation/silu.hpp"
#include "activation/exp.hpp"
#include "activation/linear.hpp"
#include "activation/relu.hpp"

#include "initializers/uniform.hpp"

#include "config.hpp"

#include "misc.hpp"

/*
    An recurrent network layer. Inspired by S7 S-SSM model and ResNet.

    We have network that takes input, uses it for updating it's hidden state and use it 
    to appended additonal information to input and pass it as output.

    Hidden state:

    h_k = delta_k*h_k-1 + B_k*x_k

    y_k = C_k*h_k

    hidden_state is a matrix

    delta_k is a square matrix

    B_k*delta_k is a matrix of input_size x hidden_state_size

    Let's replace C matrix with feed forward network.


*/

namespace snn
{
    template<size_t InputSize,size_t HiddenStateSize,size_t Populus>
    class RResNet : public Layer
    {

        const uint32_t RRES_NET_ID = 2149;
        
        // generate a row of B matrix 
        LayerKAC<InputSize,InputSize,Populus> b_matrix;

        // generate delta used for discretization
        LayerKAC<InputSize,HiddenStateSize,Populus> delta;


        SIMDVectorLite<HiddenStateSize> hidden_state;

        // output layer
        LayerKAC<HiddenStateSize,InputSize,Populus,snn::ReLu> analyzer;

        size_t id;

        struct metadata
        {
            uint32_t id;
            size_t input_size;
            size_t hidden_state_size;
            size_t population_size;
        };

        public:

        RResNet()
        : hidden_state(0)
        {
            this->id = LayerCounter::LayerIDCounter++ ;
        }

        void setup()
        {
            this->analyzer.setup();
            this->delta.setup();
            this->b_matrix.setup();    
        }

        void reset()
        {
            this->hidden_state = SIMDVectorLite<HiddenStateSize>(0);
        }

        snn::SIMDVectorLite<InputSize> fire(const snn::SIMDVectorLite<InputSize>& input)
        {
            
            snn::SIMDVectorLite<InputSize> B = this->b_matrix.fire(input);

            snn::SIMDVectorLite<HiddenStateSize> delta_k = this->delta.fire(input);

            number B_u = (B*input).reduce();

            snn::SIMDVectorLite<HiddenStateSize>  dB_u = delta_k*B_u;


            snn::SIMDVectorLite<HiddenStateSize> A = 1.f - 1.f/(delta_k*delta_k + 0.5);

            this->hidden_state = A*this->hidden_state + dB_u;

            snn::SIMDVectorLite<InputSize> out = this->analyzer.fire(this->hidden_state);

            return out + input;
        }

        void applyReward(long double reward)
        {

            this->b_matrix.applyReward(reward);
            this->delta.applyReward(reward);
            this->analyzer.applyReward(reward);

        }

        void shuttle()
        {
            
            this->b_matrix.shuttle();
            this->delta.shuttle();
            this->analyzer.shuttle();

        }

        int8_t load()
        {

            std::string filename = "layer_"+std::to_string(this->id)+".layer";

            std::ifstream file;

            file.open(filename,std::ios::in);

            if( !file.good() )
            {
                return -2;
            }

            int8_t ret = this->load(file);

            file.close();

            return ret;
        }

        int8_t save() const
        {

            std::string filename = "layer_"+std::to_string(this->id)+".layer";

            std::ofstream file;

            file.open(filename,std::ios::out);

            if(!file.good())
            {
                return -2;
            }

            int8_t ret = this->save(file);

            file.close();

            return ret;
        }

        int8_t load(std::istream& in)
        {

            RResNet::metadata _metadata = {0};

            in.read((char*)&_metadata,sizeof(_metadata));

            if( (_metadata.id != RResNet::RRES_NET_ID) || (_metadata.input_size != InputSize) || (_metadata.population_size != Populus) )
            {
                return -2;
            }

            if( (this->b_matrix.load(in) != 0 ) || (this->analyzer.load(in) != 0) || (this->delta.load(in) != 0))
            {
                return -1;
            }

            char num_buff[SERIALIZED_NUMBER_SIZE] = {0};
            
            // save hidden state 
            for(size_t i=0;i<HiddenStateSize;++i)
            {
                in.read(num_buff,SERIALIZED_NUMBER_SIZE);
                
                number num = deserialize_number<number>(num_buff);

                this->hidden_state[i] = num;

            }

            return 0;
        }

        int8_t save(std::ostream& out) const
        {
            
            RResNet::metadata _metadata = {
                .id = RResNet::RRES_NET_ID,
                .input_size = InputSize,
                .hidden_state_size = HiddenStateSize,
                .population_size = Populus
            };

            out.write((char*)&_metadata,sizeof(_metadata));

            if( out.fail() )
            {
                return -3;
            }

            if( (this->b_matrix.save(out) != 0 ) || (this->analyzer.save(out) != 0) || (this->delta.save(out) != 0))
            {
                return -1;
            }

            // save hidden state 
            for(size_t i=0;i<HiddenStateSize;++i)
            {

                char* buff = serialize_number<number>(this->hidden_state[i]);

                out.write(buff,SERIALIZED_NUMBER_SIZE);

                delete [] buff;

            }


            return 0;
        }
    };

}