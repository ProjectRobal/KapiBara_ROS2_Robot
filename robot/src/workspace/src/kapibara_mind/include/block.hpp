#pragma once

#include <algorithm>
#include <vector>
#include <deque>
#include <array>
#include <memory>
#include <random>
#include <fstream>


#include "simd_vector.hpp"
#include "neuron.hpp"
#include "initializer.hpp"
#include "crossover.hpp"
#include "mutation.hpp"

#include "config.hpp"

#include "misc.hpp"

namespace snn
{

    template<class NeuronT,size_t Populus>
    class Block
    {     

        struct BlockHeader {

            char block_type;
            size_t populus_size;
            size_t working_size;

        };   

        std::shared_ptr<Crossover> crossing;
        std::shared_ptr<Mutation> mutate;

        size_t mating_counter;

        std::uniform_int_distribution<size_t> uniform;

        std::array<std::shared_ptr<Neuron>,Populus> population;
        std::shared_ptr<Neuron> worker;

        std::deque<std::shared_ptr<Neuron>> past_neurons;

        std::shared_ptr<Neuron> best_worker;

        number swarming_speed;
        

        public:

        Block(std::shared_ptr<Crossover> _crossing,std::shared_ptr<Mutation> _mutate)
        : crossing(_crossing),
        mutate(_mutate),
        uniform(0,Populus-1),
        population({NULL}),
        worker(NULL),
        best_worker(NULL),
        mating_counter(0),
        swarming_speed(SWARMING_SPEED_DEFAULT)
        {
            
        }

        void setup(std::shared_ptr<Initializer> init)
        {
            for(auto& p : this->population)
            {
                p=std::make_shared<NeuronT>();
                p->setup(init);
            }
        }

        void setSwarmingSpeed(const number& swarming_speed)
        {
            this->swarming_speed=swarming_speed;
        }

        const number& getSwarmingSpeed()
        {
            return this->swarming_speed;
        }

        std::shared_ptr<NeuronT> getBestNeuron()
        {
            // sort array to get the neuron with bigger reward
            std::array<std::shared_ptr<Neuron>,Populus> to_sort=this->population;

            std::sort(to_sort.begin(),to_sort.end(),
            [](const std::shared_ptr<Neuron>& a,const std::shared_ptr<Neuron>& b)->bool
            {
                return a->reward()>b->reward();
            });

            return to_sort.front();
        }

        void neuronSwarming()
        {
            if(this->best_worker==NULL)
            {
                return;
            }

            std::random_device rd; 

            // Mersenne twister PRNG, initialized with seed from previous random device instance
            std::mt19937 gen(rd()); 

            std::uniform_int_distribution<size_t> swarm_chooser(0,9);

            for(auto neuron : this->population)
            {
                if(this->best_worker == neuron)
                {
                    continue;
                }

                if(swarm_chooser(gen)==5)
                {

                    snn::SIMDVector dweights = (this->best_worker->get_weights() - neuron->get_weights())/this->swarming_speed;

                    number dbias = ( this->best_worker->get_bias() - neuron->get_bias() )/this->swarming_speed;

                    neuron->update_weights(dweights);
                    neuron->update_bias(dbias);
                }
                
            }
        }

        void recordWorker(std::shared_ptr<Neuron> worker)
        {
            if(this->past_neurons.size()>=MAX_PAST_NEURONS)
            {
                this->past_neurons.pop_back();
            }
            
            this->past_neurons.push_front(worker);
        }

        void chooseWorkers()
        {
            std::random_device rd; 

            // Mersenne twister PRNG, initialized with seed from previous random device instance
            std::mt19937 gen(rd()); 

            size_t id=this->uniform(gen);

            // to make sure that we won't pick the same neuron as before
            if(this->worker == this->population[id])
            {
                id=(id+1+this->uniform(gen))%(Populus-1);
            }

            if(this->worker!=NULL)
            {
                this->recordWorker(this->worker);
            }

            this->worker=this->population[id];

        }


        void giveReward(long double reward)
        {          
            this->worker->giveReward(reward);

            long double past_reward=reward*DUMPING_FACTOR;

            for(auto neuron : this->past_neurons)
            {
                neuron->giveReward(past_reward);

                past_reward*=DUMPING_FACTOR;
            }
            
            if(this->worker->used()<USESES_TO_MAITING)
            {
                this->worker->use();
                if(this->worker->used()==USESES_TO_MAITING)
                {
                    ++this->mating_counter;
                }
            }

            if(this->best_worker==NULL)
            {
                this->best_worker=this->worker;
            }

            // pass the torch
            if(this->worker->reward()>this->best_worker->reward())
            {
                this->best_worker=this->worker;
            }
            
        }

        bool readyToMate()
        {
            return this->mating_counter >= MAITING_THRESHOLD*Populus;
        }

        void maiting(std::shared_ptr<Initializer> init)
        {
            this->best_worker=NULL;
            this->mating_counter=0;

            std::sort(this->population.begin(),this->population.end(),
            [](const std::shared_ptr<Neuron>& a,const std::shared_ptr<Neuron>& b)->bool
            {
                return a->reward()>b->reward();
            });

            size_t keep_of=Populus*AMOUNT_THAT_PASS;

            auto pivot=this->population.begin()+keep_of-1;
            auto iter2=pivot;
            auto iter=this->population.begin();

            for(auto it=this->population.begin();it!=pivot;++it)
            {
                (*it)->reset();
            }

            while(iter<pivot)
            {
                auto ite=iter2;
                (*(ite))=(*iter)->crossover(this->crossing,**(iter+1));
                (*(ite))->mutate(this->mutate);
                ite++;
                iter+=2;
            }

            while(pivot<this->population.end())
            {
                std::shared_ptr<NeuronT> n_neuron=std::make_shared<NeuronT>();
                n_neuron->setup(init);
                n_neuron->mutate(this->mutate);

                (*pivot)=n_neuron;

                pivot++;
            }

        }

        number fire(SIMDVector input)
        {
            // move population slowly towards best performing neuron
            this->neuronSwarming();

            return this->worker->fire1(input);

        }       

        size_t inputSize() const
        {
            return this->population[0]->input_size();
        }

        size_t outputSize() const
        {
            return 1;
        }


        void dump(std::ofstream& out) const
        {
            for(auto neuron : this->population)
            {
                neuron->save(out);
            }   

            uint64_t maiting = this->mating_counter;

            char maiting_data[sizeof(uint64_t)]={0};

            out.write(maiting_data,sizeof(uint64_t));
        }

        void load(std::ifstream& in)
        {
            for(auto neuron : this->population)
            {
                neuron->load(in);
            }

            uint64_t maiting = 0;

            char maiting_data[sizeof(uint64_t)]={0};

            in.read(maiting_data,sizeof(uint64_t));

            memmove(reinterpret_cast<char*>(&maiting),maiting_data,sizeof(uint64_t));

            this->mating_counter=maiting;
        }

        
    };
}