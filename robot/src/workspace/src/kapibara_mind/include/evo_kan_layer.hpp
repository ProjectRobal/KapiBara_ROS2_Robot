#pragma once

#include <vector>
#include <thread>

#include <evo_kan_block.hpp>

#include <simd_vector_lite.hpp>
#include <config.hpp>

namespace snn
{
    #define EVO_KAN_LAYER_HEADER "EKL100"
    
    template< size_t inputSize, size_t outputSize,class SplineClass = Spline >
    class EvoKanLayer
    {
        protected:

        EvoKan<inputSize,SplineClass> *blocks;

        SIMDVectorLite<outputSize> output;


        static void fire_thread(EvoKan<inputSize,SplineClass> *blocks,const SIMDVectorLite<inputSize>&input,SIMDVectorLite<outputSize>& output,size_t& current_id,std::mutex& mux);

        static void fit_thread(EvoKan<inputSize,SplineClass> *blocks,const SIMDVectorLite<inputSize>&input,const SIMDVectorLite<outputSize>& output,const SIMDVectorLite<outputSize>& target,size_t& current_id,std::mutex& mux);

        public:

        EvoKanLayer( size_t initial_spline_size = 0);

        SIMDVectorLite<outputSize> fire(const SIMDVectorLite<inputSize>& input);

        void fit(const SIMDVectorLite<inputSize>& input,const SIMDVectorLite<outputSize>& target);

        void save(std::ostream& out) const;

        void load(std::istream& in);

        ~EvoKanLayer();

    }; 


    template< size_t inputSize, size_t outputSize,class SplineClass >
    void EvoKanLayer<inputSize,outputSize,SplineClass>::fire_thread(EvoKan<inputSize,SplineClass> *blocks,const SIMDVectorLite<inputSize>&input,SIMDVectorLite<outputSize>& output,size_t& current_id,std::mutex& mux)
    {
        size_t id = 0;

        while( true )
        {
            {
                std::lock_guard guard(mux);

                id = current_id;

                current_id++;

                if( id >= outputSize )
                {
                    return;
                }
            }

            output[id] = blocks[id].fire(input);
        }
    }

    template< size_t inputSize, size_t outputSize,class SplineClass >
    void EvoKanLayer<inputSize,outputSize,SplineClass>::fit_thread(EvoKan<inputSize,SplineClass> *blocks,const SIMDVectorLite<inputSize>&input,const SIMDVectorLite<outputSize>& output,const SIMDVectorLite<outputSize>& target,size_t& current_id,std::mutex& mux)
    {
        size_t id = 0;

        number _target;
        number _output;

        while( true )
        {
            {
                std::lock_guard guard(mux);

                id = current_id;

                current_id++;

                if( id >= outputSize )
                {
                    return;
                }

                _target = target[id];
                _output = output[id];
            }

            blocks[id].fit(input,_output,_target);
        }
    }

    template< size_t inputSize, size_t outputSize,class SplineClass >
    EvoKanLayer<inputSize,outputSize,SplineClass>::EvoKanLayer( size_t initial_spline_size)
    {
        this->blocks = new EvoKan<inputSize,SplineClass>[outputSize](initial_spline_size);
    }

    template< size_t inputSize, size_t outputSize,class SplineClass >
    SIMDVectorLite<outputSize> EvoKanLayer<inputSize,outputSize,SplineClass>::fire(const SIMDVectorLite<inputSize>& input)
    {
        size_t current_id = 0;

        std::mutex id_lock;

        std::thread threads[THREAD_COUNT];

        for(size_t i=0;i<THREAD_COUNT;++i)
        {
            threads[i] = std::thread(EvoKanLayer::fire_thread,this->blocks,std::cref(input),std::ref(this->output),std::ref(current_id),std::ref(id_lock));
        }

        for(size_t i=0;i<THREAD_COUNT;++i)
        {
            if( threads[i].joinable() )
            {
                threads[i].join();
            }
        }

        return this->output;

    }

    template< size_t inputSize, size_t outputSize,class SplineClass >
    void EvoKanLayer<inputSize,outputSize,SplineClass>::fit(const SIMDVectorLite<inputSize>& input,const SIMDVectorLite<outputSize>& target)
    {
        size_t current_id = 0;

        std::mutex id_lock;

        std::thread threads[THREAD_COUNT];

        for(size_t i=0;i<THREAD_COUNT;++i)
        {
            threads[i] = std::thread(EvoKanLayer::fit_thread,this->blocks,std::cref(input),std::cref(this->output),std::cref(target),std::ref(current_id),std::ref(id_lock));
        }

        for(size_t i=0;i<THREAD_COUNT;++i)
        {
            if( threads[i].joinable() )
            {
                threads[i].join();
            }
        }
        
    }

    template< size_t inputSize, size_t outputSize,class SplineClass >
    void EvoKanLayer<inputSize,outputSize,SplineClass>::save(std::ostream& out) const
    {
        // save layer header
        out.write(EVO_KAN_LAYER_HEADER,strlen(EVO_KAN_LAYER_HEADER));

        for( size_t i=0; i<outputSize; ++i )
        {
            this->blocks[i].save(out);
        }

    }

    template< size_t inputSize, size_t outputSize,class SplineClass >
    void EvoKanLayer<inputSize,outputSize,SplineClass>::load(std::istream& in)
    {
        char header[strlen(EVO_KAN_LAYER_HEADER)];

        in.read(header,strlen(EVO_KAN_LAYER_HEADER));

        // check for header
        if( strncmp(header,EVO_KAN_LAYER_HEADER,strlen(EVO_KAN_LAYER_HEADER)) != 0)
        {   
            throw std::runtime_error("Header mismatch in byte stream!!!");
        }

        for( size_t i=0; i<outputSize; ++i )
        {
            this->blocks[i].load(in);
        }

    }

    template< size_t inputSize, size_t outputSize,class SplineClass >
    EvoKanLayer<inputSize,outputSize,SplineClass>::~EvoKanLayer()
    {
        delete [] this->blocks;
    }



}