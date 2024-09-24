#pragma once

/*

    A class that implements replay buffer for short term memory in K.A.C algorithm.

    Use AutoEncoder to converts (inputs, outputs) pairs into single number. ( Not suitable for bigger range of inputs )
    Create AutoEncoder.


    Find values by sorting by each paramaters. Until the best match is found.
    -> Use mean square error to find pairs with smallest error.

    Maybe use something like divide and conquer.

*/


#include <vector>
#include <deque>
#include <utility>
#include <memory>

#include "config.hpp"

#include "simd_vector.hpp"


typedef std::pair<number,std::pair<snn::SIMDVector,number>> BufferElement;


class ShortTermMemory
{
    
    std::size_t MaxSize;

    // MSE threshold for chunk selection
    number Beta;
    // MSE threshold for selecting nearest neighbour
    number Alfa;    

    // holds (mean square values ( (inputs,outputs) , reward ) ) pairs.
    std::deque<std::shared_ptr<BufferElement>> buffer;

    // value - a value we are searching for, f - a index of first element of array, l - a index of last element of array
    std::shared_ptr<BufferElement> binary_search(const std::deque<std::shared_ptr<BufferElement>>& array,const snn::SIMDVector& vec, number& value,size_t f,size_t l)
    {

        if( f == l)
        {
            return NULL;
        }


        number MSE=((array[f]->first-value)*(array[f]->first-value)+(array[l]->first-value)*(array[l]->first-value))/2;

        if( MSE < this->Alfa )
        {
            number _mse=1<<32;

            int idx=-1;

            size_t count=0;

            for(size_t i=f;i<=l;++i)
            {
                count=0;
                /*number c_mse=((array[i]->second.first-vec)*(array[i]->second.first-vec)).dot_product()/vec.size();

                if( c_mse < _mse )
                {
                    _mse = c_mse;
                    idx=i;
                }*/

                for(size_t o=0;o<vec.size();++o)
                {
                    if((((vec[o]<0)==(array[i]->second.first[o]<0)) && (abs(vec[o]-array[i]->second.first[o]) <= this->Beta)))
                    {
                        count++;
                    }
                }   

                if( count == vec.size() )
                {
                    idx=i;
                }
            }

            if(idx>-1)
            {
                return array[idx];
            }
            else
            {
                return NULL;
            }
        }

        number center = array[floor((f+l)/2)]->first;

        if( value < center )
        {
            return binary_search(array,vec,value,f,floor((f+l)/2));
        }
        else if( value > center )
        {
            return binary_search(array,vec,value,floor((f+l)/2),l);
        }
        else
        {
            return array[floor((f+l)/2)];
        }

    }


    public:

    ShortTermMemory(std::size_t MaxSize,number Alfa,number Beta)
    {
        this->MaxSize=MaxSize;
        this->Alfa=Alfa;
        this->Beta=Beta;
    }

    void append(const snn::SIMDVector& inputs,const snn::SIMDVector& outputs,const number& reward)
    {
        if( this->buffer.size() >= this->MaxSize )
        {
            this->buffer.pop_front();
        }

        snn::SIMDVector input_outputs=inputs;

        input_outputs.extend(outputs);

        std::pair<snn::SIMDVector,number> pairs=std::pair<snn::SIMDVector,number>(input_outputs,reward);

        number mse=(input_outputs*input_outputs).dot_product();

        this->buffer.push_back(std::make_shared<BufferElement>(std::pair(mse,pairs)));
    }

    bool EstimateRewardFor(const snn::SIMDVector& inputs,const snn::SIMDVector& outputs,number& estimated)
    {
        snn::SIMDVector input_outputs=inputs;

        input_outputs.extend(outputs);

        number mse=(input_outputs*input_outputs).dot_product()/input_outputs.size();

        auto sorted=this->buffer;

        std::sort(sorted.begin(),sorted.end(),[](std::shared_ptr<BufferElement> a,std::shared_ptr<BufferElement> b)
        {
            return a->first>b->first;
        });

        auto ret=this->binary_search(sorted,input_outputs,mse,0,input_outputs.size()-1);

        // find element in buffer that is close to input_outputs

        if( ret == NULL)
        {
            return false;
        }

        std::cout<<"Estimated: "<<ret->second.first<<std::endl;

        estimated = ret->second.second;

        return true;

    }

    std::size_t getSize()
    {
        return this->buffer.size();        
    }

    const std::size_t& getMaxSize()
    {
        return this->MaxSize;
    }

};
