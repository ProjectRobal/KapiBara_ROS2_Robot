#pragma once

#include <random>
#include <cstdint>
#include <cmath>
#include <climits>
#include "simd_vector.hpp"
#include "simd_vector_lite.hpp"

#include "config.hpp"

#define SERIALIZED_NUMBER_SIZE (2*sizeof(int64_t))

namespace snn
{
    // serialize number of type T, and return it as serialized stream
    template<typename T>
    char* serialize_number(T num)
    {
        int exp=0;

        int64_t mant=std::numeric_limits<std::int64_t>::max()*std::frexp(num,&exp);

        int64_t _exp=exp;

        char* output = new char[2*sizeof(int64_t)];
        
        memmove(output,reinterpret_cast<char*>(&mant),sizeof(int64_t));
        memmove(output+sizeof(int64_t),reinterpret_cast<char*>(&_exp),sizeof(int64_t));

        return output;
    }

    template<typename T>
    // buffer size 2*sizeof(int64_t)
    void serialize_number(T num,char* buffer)
    {
        int exp=0;

        int64_t mant=std::numeric_limits<std::int64_t>::max()*std::frexp(num,&exp);

        int64_t _exp=exp;
        
        memmove(buffer,reinterpret_cast<char*>(&mant),sizeof(int64_t));
        memmove(buffer+sizeof(int64_t),reinterpret_cast<char*>(&_exp),sizeof(int64_t));

    }

    // deserialize number of type T
    template<typename T>
    number deserialize_number(char* data)
    {
        int64_t exp=0;
        int64_t mant=0;

        memmove(reinterpret_cast<char*>(&mant),data,sizeof(uint64_t));
        memmove(reinterpret_cast<char*>(&exp),data+sizeof(uint64_t),sizeof(uint64_t));

        return static_cast<T>(std::ldexp(static_cast<T>(mant) / std::numeric_limits<std::int64_t>::max() ,exp));
    }

    size_t get_action_id(const snn::SIMDVector& actions)
    {
        std::random_device rd; 

        // Mersenne twister PRNG, initialized with seed from previous random device instance
        std::mt19937 gen(rd()); 

        std::uniform_real_distribution<number> uniform_chooser(0.f,1.f);

        number shift = 0;

        number choose = uniform_chooser(gen);

        size_t action_id = 0;

        for(size_t i=0;i<actions.size();++i)
        {
            if( choose <= actions[i] + shift )
            {
                action_id = i;
                break;
            }

            shift += actions[i];
        }

        return action_id;
    }

    template<size_t Size>
    size_t get_action_id(const snn::SIMDVectorLite<Size>& actions)
    {
        std::random_device rd; 

        // Mersenne twister PRNG, initialized with seed from previous random device instance
        std::mt19937 gen(rd()); 

        std::uniform_real_distribution<number> uniform_chooser(0.f,1.f);

        number shift = 0;

        number choose = uniform_chooser(gen);

        size_t action_id = 0;

        for(size_t i=0;i<actions.size();++i)
        {
            if( choose <= actions[i] + shift )
            {
                action_id = i;
                break;
            }

            shift += actions[i];
        }

        return action_id;
    }

    template<size_t Size>
    void swap(size_t a, size_t b,SIMDVectorLite<Size>& vec) {
            number temp = vec[a];

            vec.set(vec[b],a);
            vec.set(temp,b);
        }

    void swap(size_t a, size_t b,SIMDVector& vec) {
            number temp = vec[a];

            vec.set(vec[b],a);
            vec.set(temp,b);
        }

    size_t partition(SIMDVector& arr, size_t low, size_t high) {
        number pivot = arr[high];  // Choosing the last element as the pivot
        size_t i = low - 1;        // Index of the smaller element

        for (size_t j = low; j <= high - 1; ++j) {
            // If the current element is smaller than or equal to the pivot
            if (arr[j] <= pivot) {
                ++i;    // Increment the index of the smaller element
                swap(i, j,arr);
            }
        }
        swap(i + 1, high,arr);
        return (i + 1);
    }

    template<size_t Size>
    size_t partition(SIMDVectorLite<Size>& arr, size_t low, size_t high) {
        number pivot = arr[high];  // Choosing the last element as the pivot
        size_t i = low - 1;        // Index of the smaller element

        for (size_t j = low; j <= high - 1; ++j) {
            // If the current element is smaller than or equal to the pivot
            if (arr[j] <= pivot) {
                ++i;    // Increment the index of the smaller element
                swap(i, j,arr);
            }
        }
        swap(i + 1, high,arr);
        return (i + 1);
    }

    void quicksort(SIMDVector &arr, int low, int high) {
        if (low < high) {
            size_t pi = partition(arr, low, high);

            // Separately sort elements before and after partition
            quicksort(arr, low, pi - 1);
            quicksort(arr, pi + 1, high);
        }
    }

    template<size_t Size>
    void quicksort(SIMDVectorLite<Size> &arr, int low, int high) {
        if (low < high) {
            size_t pi = partition(arr, low, high);

            // Separately sort elements before and after partition
            quicksort(arr, low, pi - 1);
            quicksort(arr, pi + 1, high);
        }
    }


    size_t partition_mask(SIMDVector& arr, size_t low, size_t high,SIMDVector& mask) {
        number pivot = mask[high];  // Choosing the last element as the pivot
        size_t i = low - 1;        // Index of the smaller element

        for (size_t j = low; j <= high - 1; ++j) {
            // If the current element is smaller than or equal to the pivot
            if (mask[j] <= pivot) {
                ++i;    // Increment the index of the smaller element
                swap(i, j,arr);
                swap(i,j,mask);
            }
        }
        swap(i + 1, high,arr);
        swap(i + 1, high,mask);
        
        return (i + 1);
    }

    template<size_t Size>
    size_t partition_mask(SIMDVectorLite<Size>& arr, size_t low, size_t high,SIMDVectorLite<Size>& mask) {
        number pivot = mask[high];  // Choosing the last element as the pivot
        size_t i = low - 1;        // Index of the smaller element

        for (size_t j = low; j <= high - 1; ++j) {
            // If the current element is smaller than or equal to the pivot
            if (mask[j] <= pivot) {
                ++i;    // Increment the index of the smaller element
                swap(i, j,arr);
                swap(i,j,mask);
            }
        }
        swap(i + 1, high,arr);
        swap(i + 1, high,mask);
        
        return (i + 1);
    }

    void quicksort_mask(SIMDVector &arr, int low, int high,SIMDVector& mask) {
        if (low < high) {
            size_t pi = partition_mask(arr, low, high,mask);

            // Separately sort elements before and after partition
            quicksort_mask(arr, low, pi - 1,mask);
            quicksort_mask(arr, pi + 1, high,mask);
        }
    }

    template<size_t Size>
    void quicksort_mask(SIMDVectorLite<Size> &arr, int low, int high,SIMDVectorLite<Size>& mask) {
        if (low < high) {
            size_t pi = partition_mask(arr, low, high,mask);

            // Separately sort elements before and after partition
            quicksort_mask(arr, low, pi - 1,mask);
            quicksort_mask(arr, pi + 1, high,mask);
        }
    }

    SIMDVector power(const SIMDVector& vec, size_t N)
        {
            SIMDVector out=vec;

            while(--N)
            {
                out=out*vec;
            }

            return out;
        }

    template<size_t Size>
    SIMDVectorLite<Size> power(const SIMDVectorLite<Size>& vec, size_t N)
        {
            SIMDVectorLite<Size> out=vec;

            while(--N)
            {
                out=out*vec;
            }

            return out;
        }

        SIMDVector exp(const SIMDVector& vec)
        {
            size_t n=1;

            SIMDVector x1=vec;

            SIMDVector sum=x1+1;

            while(n < 20)
            {
                x1=x1*(vec/(++n));
                sum+=x1;
            };

            return sum;
            
        }

        template<size_t Size>
        SIMDVectorLite<Size> exp(const SIMDVectorLite<Size>& vec)
        {
            size_t n=1;

            SIMDVectorLite<Size> x1=vec;

            SIMDVectorLite<Size> sum=x1+1;

            while(n < 20)
            {
                x1=x1*(vec/(++n));
                sum+=x1;
            };

            return sum;
            
        }

        SIMDVector pexp(const SIMDVector& vec)
        {

            SIMDVector xm = vec < 0.f;

            xm = xm*vec;

            xm = 1.f/((xm*-1)+1);

            return xm + (vec * (vec>=0.f));
            
        }

        template<size_t Size>
        SIMDVectorLite<Size> pexp(const SIMDVectorLite<Size>& vec)
        {

            SIMDVectorLite<Size> xm = vec < 0.f;

            xm = xm*vec;

            xm = 1.f/((xm*-1)+1);

            return xm + (vec * (vec>=0.f));
            
        }

        number pexp(number v)
        {
            return (v>0.f)*(v+1) + ((v<=0.f)/(-v+1));
        }

        SIMDVector simd_abs(const SIMDVector& vec)
        {
            SIMDVector check = ((vec<0)*-2)+1;

            return vec*check;     
            
        }

        template<size_t Size>
        SIMDVectorLite<Size> simd_abs(const SIMDVectorLite<Size>& vec)
        {
            SIMDVectorLite<Size> check = ((vec<0)*-2)+1;

            return vec*check;     
            
        }
};