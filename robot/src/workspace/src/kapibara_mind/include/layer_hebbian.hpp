#pragma once

#include <array>
#include <functional>
#include <fstream>
#include <algorithm>
#include <string>
#include <thread>
#include <deque>

#include "block_kac.hpp"
#include "initializer.hpp"

#include "simd_vector.hpp"
#include "simd_vector_lite.hpp"

#include "config.hpp"

#include "activation/linear.hpp"

#include "layer.hpp"

#include <filesystem>

#include "nlohmann/json.hpp"

#include "layer_counter.hpp"

#include "initializers/hu.hpp"
#include "initializers/gauss.hpp"
#include "initializers/uniform.hpp"
/*


 A layer that use evolutionary algorithm for learing called CoSyne.

*/
namespace snn
{    
    template<size_t inputSize,size_t N,class Activation = Linear,class weight_initializer = UniformInit<(number)-0.01f,(number)0.01f>>
    class LayerHebbian : public Layer
    { 
        const uint32_t LAYER_HEBBIAN_ID = 2158;

        snn::SIMDVectorLite<inputSize> blocks[N];

        number biases[N];

        weight_initializer global;

        std::uniform_real_distribution<double> uniform;

        snn::SIMDVectorLite<inputSize> past_dw;

        size_t iter;

        size_t id;

        number learning_value;

        struct metadata
        {
            uint32_t id;
            size_t input_size;
            size_t node_size;
        };
        
        public:

        LayerHebbian()
        {
            this->id = LayerCounter::LayerIDCounter++ ;

            this->uniform=std::uniform_real_distribution<double>(0.f,1.f);

            this->learning_value = 0.01f;

            this->iter = 0;
        }

        void setup()
        {

            for(size_t i=0;i<N;++i)
            {
                // this->blocks[i]= BlockKAC<inputSize,Populus>();
                for(size_t j=0;j<inputSize;++j)
                {
                    this->blocks[i][j] = this->global.init();
                }

                this->biases[i] = this->global.init();

                // this->blocks.back().chooseWorkers();
            }
        }

        void applyReward(long double reward)
        {
            // if( reward < 0 )
            // {
            //     this->learning_value = -0.01f;
            // }
            // else
            // {
            //     this->learning_value = 0.01f;
            // }

            // this->learning_value = ( 1.f / ( std::exp(-reward) + 1.f ) - 0.5f )*0.1f;
            // reward/=this->blocks.size();
        }

        void applyLearning(const snn::SIMDVectorLite<N>& post_activations,const snn::SIMDVectorLite<inputSize>& pre_activations)
        {
            UniformInit<(number)0.f,(number)1.f> uniform;

            for(size_t i=0;i<N;++i)
            {

                if( uniform.init() > 0.2 )
                {
                    continue;
                }

                number p = post_activations[i];

                snn::SIMDVectorLite<inputSize> dw = this->learning_value*p*(pre_activations - this->blocks[i]*p);

                this->blocks[i] += dw;

                // this->biases[N] += this->learning_value*post_activations[i];

                // snn::SIMDVectorLite<inputSize> compare_high = this->blocks[i] > 10.f;
                // snn::SIMDVectorLite<inputSize> compare_low = this->blocks[i] < -10.f;

                // this->blocks[i] -= ( dw*compare_high + dw*compare_low );

                this->iter++;
            }
        }

        void shuttle()
        {
            
            
        }

        static void fire_parraler(snn::SIMDVectorLite<inputSize> * blocks,number *biases,const SIMDVectorLite<inputSize>& input,SIMDVectorLite<N> &output,size_t start,size_t end)
        {
            for(;start<end;++start)
            {
                output[start] = (blocks[start]*input).reduce();//+biases[start];
            }
        }

        SIMDVectorLite<N> fire(const SIMDVectorLite<inputSize>& input)
        {
            SIMDVectorLite<N> output(0);

            std::thread threads[USED_THREADS];

            size_t min = 0;

            size_t worker_count = USED_THREADS;

            if( worker_count > N )
            {
                worker_count = N;   
            }

            const size_t step = N/worker_count;

            size_t max = step; 


            for(size_t i=0;i<worker_count;++i)
            {
                threads[i] = std::thread(fire_parraler,this->blocks,this->biases,std::cref(input),std::ref(output),min,max);

                min = max;
                
                max = ( 1 - ( i/(worker_count-1) ) )*( max + step ) + ( i/(worker_count-1) )*N;
            }

            for(size_t i=0;i<worker_count;++i)
            {
                threads[i].join();
            }


            // for(size_t i=0;i<N;++i)
            // {
            //     number v = this->blocks[i].fire(input);
            //     output[i] = v;

            //     // if(used_thread<4)
            //     // {
            //     //     threads[ i%4 ] = std::thread(fire_parraler,this->blocks,std::cref(input),std::ref(output),i);
            //     //     used_thread++;
            //     // }
            //     // else
            //     // {
            //     //     for(auto& thread : threads)
            //     //     {
            //     //         thread.join();
            //     //     }

            //     //     used_thread = 0;
            //     // }

            // }

            // for(size_t i=0; i< 4; i++)
            // {
            //     if(threads[i].joinable())
            //     {
            //         threads[i].join();
            //     }
            // }

            // Activation::activate(output);

            return output;
        }

        int8_t load()
        {
            std::string filename = "layer_"+std::to_string(this->id)+".layer";

            std::ifstream file;

            file.open(filename,std::ios::in);

            if(!file.good())
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

            LayerHebbian::metadata meta={0};

            in.read((char*)&meta,sizeof(LayerHebbian::metadata));

            if( meta.id != LayerHebbian::LAYER_HEBBIAN_ID || meta.input_size != inputSize || meta.node_size != N )
            {
                return -3;
            }

            for(size_t i=0;i<N;++i)
            {
                if(in.good())
                {
                    // to do
                }
                else
                {
                    return -1;
                }
            }

            return 0;
        }

        int8_t save(std::ostream& out) const
        {

            LayerHebbian::metadata meta = {
                .id = LayerHebbian::LAYER_HEBBIAN_ID,
                .input_size = inputSize,
                .node_size = N
            };

            out.write((char*)&meta,sizeof(LayerHebbian::metadata));
            
            for(size_t i=0;i<N;++i)
            {
                if(out.good())
                {
                    // to do
                }
                else
                {
                    return -1;
                }
            }

            return 0;
        }        

        ~LayerHebbian()
        {

        }

    };  
    
} // namespace snn
