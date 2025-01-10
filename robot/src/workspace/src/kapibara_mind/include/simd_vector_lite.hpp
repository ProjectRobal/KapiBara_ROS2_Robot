#pragma once

#include <cmath>
#include <cstdint>

#include "config.hpp"

namespace snn
{

#define VEC_COUNT (static_cast<size_t>(Size/MAX_SIMD_VECTOR_SIZE)+1)

template<size_t Size>
class SIMDVectorLite
{
private:
    /* data */
    SIMD _vec[VEC_COUNT];

    void zeros_last_element(size_t count)
    {
        for(size_t i=0;i<count;++i)
        {
            this->_vec[VEC_COUNT-1][MAX_SIMD_VECTOR_SIZE-i] = 0.f;
        }
    }

    SIMD mask_to_simd(const SIMD_MASK& mask) const
    {
        SIMD output(0);

        for(uint16_t i = 0; i< MAX_SIMD_VECTOR_SIZE; ++i)
        {
            output[i] = mask[i] ? 1.f : 0.f;

        }

        return output;
    }

public:
    SIMDVectorLite();

    SIMDVectorLite(number nm);

    number reduce();

    number operator[](size_t i) const;

    SIMD::reference operator[](size_t i);

    template<size_t OutputSize>
    SIMDVectorLite<OutputSize> split();

    size_t size() const
    {
        return Size;
    }

    void set(size_t i,number v);

    void set_block(size_t i,const SIMD& block)
    {
        if(i>=VEC_COUNT)
        {
            return;
        }

        this->_vec[i] = block;
    }

    SIMD& get_block(size_t i)
    {
        if(i>=VEC_COUNT)
        {
            i = 0;
        }

        return this->_vec[i];
    }

    void operator+=(number v);

    void operator-=(number v);

    void operator*=(number v);

    void operator/=(number v);

    SIMDVectorLite operator+(number v) const;

    SIMDVectorLite operator-(number v) const;

    SIMDVectorLite operator*(number v) const;

    SIMDVectorLite operator/(number v) const;


    void operator+=(const SIMDVectorLite<Size>& v);

    void operator-=(const SIMDVectorLite<Size>& v);

    void operator*=(const SIMDVectorLite<Size>& v);

    void operator/=(const SIMDVectorLite<Size>& v);

    SIMDVectorLite operator+(const SIMDVectorLite<Size>& v) const;

    SIMDVectorLite operator-(const SIMDVectorLite<Size>& v) const;

    SIMDVectorLite operator*(const SIMDVectorLite<Size>& v) const;

    SIMDVectorLite operator/(const SIMDVectorLite<Size>& v) const;



    SIMDVectorLite operator==(const SIMDVectorLite<Size>& v) const;

    SIMDVectorLite operator!=(const SIMDVectorLite<Size>& v) const;

    SIMDVectorLite operator>=(const SIMDVectorLite<Size>& v) const;

    SIMDVectorLite operator<=(const SIMDVectorLite<Size>& v) const;

    SIMDVectorLite operator>(const SIMDVectorLite<Size>& v) const;

    SIMDVectorLite operator<(const SIMDVectorLite<Size>& v) const;

    // template<size_t Size1>
    // friend std::ostream& operator<<(std::ostream& out,const snn::SIMDVectorLite<Size1>& vec);


    ~SIMDVectorLite();
};

template<size_t Size>
SIMDVectorLite<Size>::SIMDVectorLite()
{
    for(SIMD& simd : this->_vec)
    {
        simd = SIMD(0);
    }
}

template<size_t Size>
SIMDVectorLite<Size>::SIMDVectorLite(number num)
{
    size_t element_left = Size - static_cast<size_t>(Size/MAX_SIMD_VECTOR_SIZE)*MAX_SIMD_VECTOR_SIZE;

    size_t to_set = VEC_COUNT;

    
    for(size_t i=0;i<to_set;++i)
    {
        this->_vec[i] = SIMD(num);
    }

    to_set -=1 ;

    this->_vec[to_set] = SIMD(0);

    for(size_t i=0;i<element_left;++i)
    {
        this->_vec[to_set][i] = num;
    }

}

template<size_t Size>
template<size_t OutputSize>
SIMDVectorLite<OutputSize> SIMDVectorLite<Size>::split()
{
    SIMDVectorLite<OutputSize> output;

    size_t copy_from_last = OutputSize - (OutputSize/MAX_SIMD_VECTOR_SIZE) * MAX_SIMD_VECTOR_SIZE;

    size_t blocks_to_copy = (OutputSize/MAX_SIMD_VECTOR_SIZE)+1;

    if( copy_from_last > 0 )
    {
        blocks_to_copy--;
    }

    for(size_t i=0;i<blocks_to_copy;++i)
    {
        output.set_block(i,this->_vec[i]);
    }

    for(size_t i=0;i<copy_from_last;++i)
    {
        output.get_block(blocks_to_copy)[i] = this->_vec[blocks_to_copy][i];
    }

    return output;
}

template<size_t Size>
number SIMDVectorLite<Size>:: reduce()
{
    number output = 0;

    for(const SIMD& simd : this->_vec)
    {
        output+=std::experimental::reduce(simd);
    }

    return output;
}

template<size_t Size>
number SIMDVectorLite<Size>::operator[](size_t i) const
{
    if(i>=Size)
    {
        i = 0;
    }

    size_t simd_id = i/MAX_SIMD_VECTOR_SIZE;

    return _vec[simd_id][i - simd_id*MAX_SIMD_VECTOR_SIZE];
}

template<size_t Size>
SIMD::reference SIMDVectorLite<Size>::operator[](size_t i)
{
    if(i>=Size)
    {
        i = 0;
    }

    size_t simd_id = i/MAX_SIMD_VECTOR_SIZE;

    return _vec[simd_id][i - simd_id*MAX_SIMD_VECTOR_SIZE];
}

template<size_t Size>
void SIMDVectorLite<Size>::set(size_t i,number v)
{
    if(i>=Size)
    {
        return;
    }

    size_t simd_id = i/MAX_SIMD_VECTOR_SIZE;

    _vec[simd_id][i - simd_id*MAX_SIMD_VECTOR_SIZE] = v;
}

template<size_t Size>
void SIMDVectorLite<Size>::operator+=(number v)
{
    size_t element_left = Size - static_cast<size_t>(Size/MAX_SIMD_VECTOR_SIZE)*MAX_SIMD_VECTOR_SIZE;
    
    for(size_t i=0;i<VEC_COUNT;++i)
    {
        this->_vec[i] += v;
    }

    this->zeros_last_element(MAX_SIMD_VECTOR_SIZE-element_left);

}

template<size_t Size>
void SIMDVectorLite<Size>::operator-=(number v)
{
    size_t element_left = Size - static_cast<size_t>(Size/MAX_SIMD_VECTOR_SIZE)*MAX_SIMD_VECTOR_SIZE;
    
    for(size_t i=0;i<VEC_COUNT;++i)
    {
        this->_vec[i] -= v;
    }

    this->zeros_last_element(MAX_SIMD_VECTOR_SIZE-element_left);
}

template<size_t Size>
void SIMDVectorLite<Size>::operator*=(number v)
{
    for(SIMD& simd : this->_vec)
    {
        simd*=v;
    }
}

template<size_t Size>
void SIMDVectorLite<Size>::operator/=(number v)
{
    if( v == 0)
    {
        return;
    }

    for(SIMD& simd : this->_vec)
    {
        simd/=v;
    }
}

template<size_t Size>
SIMDVectorLite<Size> SIMDVectorLite<Size>::operator+(number v) const
{
    SIMDVectorLite<Size> output;

    size_t element_left = Size - static_cast<size_t>(Size/MAX_SIMD_VECTOR_SIZE)*MAX_SIMD_VECTOR_SIZE;
    
    for(size_t i=0;i<VEC_COUNT;++i)
    {
        output._vec[i] = this->_vec[i] + v;
    }

    output.zeros_last_element(MAX_SIMD_VECTOR_SIZE-element_left);

    return output;

}

template<size_t Size>
SIMDVectorLite<Size> SIMDVectorLite<Size>::operator-(number v) const
{
    SIMDVectorLite<Size> output;

    size_t element_left = Size - static_cast<size_t>(Size/MAX_SIMD_VECTOR_SIZE)*MAX_SIMD_VECTOR_SIZE;
    
    for(size_t i=0;i<VEC_COUNT;++i)
    {
        output._vec[i] = this->_vec[i] - v;
    }

    output.zeros_last_element(MAX_SIMD_VECTOR_SIZE-element_left);

    return output;
}

template<size_t Size>
SIMDVectorLite<Size> SIMDVectorLite<Size>::operator*(number v) const
{
    SIMDVectorLite<Size> output;
    
    for(size_t i=0;i<VEC_COUNT;++i)
    {
        output._vec[i] = this->_vec[i] * v;
    }

    return output;

}

template<size_t Size>
SIMDVectorLite<Size> SIMDVectorLite<Size>::operator/(number v) const
{
    if( v == 0)
    {
        v+= 0.000000000000001f;
    }

    SIMDVectorLite<Size> output;
    
    for(size_t i=0;i<VEC_COUNT;++i)
    {
        output._vec[i] = this->_vec[i] / v;
    }

    return output;

}

template<size_t Size>
void SIMDVectorLite<Size>::operator+=(const SIMDVectorLite<Size>& v)
{
    for(size_t i=0;i<VEC_COUNT;++i)
    {
        this->_vec[i] += v._vec[i];
    }
}

template<size_t Size>
void SIMDVectorLite<Size>::operator-=(const SIMDVectorLite<Size>& v)
{
    for(size_t i=0;i<VEC_COUNT;++i)
    {
        this->_vec[i] -= v._vec[i];
    }
}

template<size_t Size>
void SIMDVectorLite<Size>::operator*=(const SIMDVectorLite<Size>& v)
{
    for(size_t i=0;i<VEC_COUNT;++i)
    {
        this->_vec[i] *= v._vec[i];
    }
}

template<size_t Size>
void SIMDVectorLite<Size>::operator/=(const SIMDVectorLite<Size>& v)
{
    for(size_t i=0;i<VEC_COUNT;++i)
    {
        this->_vec[i] /= ( v._vec[i] + 0.000000000000000001f);
    }
}

template<size_t Size>
SIMDVectorLite<Size> SIMDVectorLite<Size>::operator+(const SIMDVectorLite<Size>& v) const
{
    SIMDVectorLite<Size> output;
    
    for(size_t i=0;i<VEC_COUNT;++i)
    {
        output._vec[i] = this->_vec[i] + v._vec[i];
    }

    return output;
}

template<size_t Size>
SIMDVectorLite<Size> SIMDVectorLite<Size>::operator-(const SIMDVectorLite<Size>& v) const
{
    SIMDVectorLite<Size> output;
    
    for(size_t i=0;i<VEC_COUNT;++i)
    {
        output._vec[i] = this->_vec[i] - v._vec[i];
    }

    return output;
}

template<size_t Size>
SIMDVectorLite<Size> SIMDVectorLite<Size>::operator*(const SIMDVectorLite<Size>& v) const
{
    SIMDVectorLite<Size> output;
    
    for(size_t i=0;i<VEC_COUNT;++i)
    {
        output._vec[i] = this->_vec[i] * v._vec[i];
    }

    return output;
}

template<size_t Size>
SIMDVectorLite<Size> SIMDVectorLite<Size>::operator/(const SIMDVectorLite<Size>& v) const
{
    SIMDVectorLite<Size> output;
    
    for(size_t i=0;i<VEC_COUNT;++i)
    {
        output._vec[i] = this->_vec[i] / ( v._vec[i] +  0.000000000000000001f);
    }

    return output;
}



template<size_t Size>
SIMDVectorLite<Size> SIMDVectorLite<Size>::operator==(const SIMDVectorLite<Size>& v) const
{
    SIMDVectorLite<Size> output;

    for(size_t i=0;i<VEC_COUNT;++i)
    {
        output._vec[i] = this->mask_to_simd(this->_vec[i] == v._vec[i]);
    }

    output.zeros_last_element(VEC_COUNT*MAX_SIMD_VECTOR_SIZE - Size);

    return output;
}

template<size_t Size>
SIMDVectorLite<Size> SIMDVectorLite<Size>::operator!=(const SIMDVectorLite<Size>& v) const
{
    SIMDVectorLite<Size> output;

    for(size_t i=0;i<VEC_COUNT;++i)
    {
        output._vec[i] = this->mask_to_simd(this->_vec[i] != v._vec[i]);
    }

    output.zeros_last_element(VEC_COUNT*MAX_SIMD_VECTOR_SIZE - Size);

    return output;

}

template<size_t Size>
SIMDVectorLite<Size> SIMDVectorLite<Size>::operator>=(const SIMDVectorLite<Size>& v) const
{
    SIMDVectorLite<Size> output;

    for(size_t i=0;i<VEC_COUNT;++i)
    {
        output._vec[i] = this->mask_to_simd(this->_vec[i] >= v._vec[i]);
    }

    output.zeros_last_element(VEC_COUNT*MAX_SIMD_VECTOR_SIZE - Size);

    return output;
}

template<size_t Size>
SIMDVectorLite<Size> SIMDVectorLite<Size>::operator<=(const SIMDVectorLite<Size>& v) const
{
    SIMDVectorLite<Size> output;

    for(size_t i=0;i<VEC_COUNT;++i)
    {
        output._vec[i] = this->mask_to_simd(this->_vec[i] <= v._vec[i]);
    }

    output.zeros_last_element(VEC_COUNT*MAX_SIMD_VECTOR_SIZE - Size);

    return output;
}

template<size_t Size>
SIMDVectorLite<Size> SIMDVectorLite<Size>::operator>(const SIMDVectorLite<Size>& v) const
{
    SIMDVectorLite<Size> output;

    for(size_t i=0;i<VEC_COUNT;++i)
    {
        output._vec[i] = this->mask_to_simd(this->_vec[i] > v._vec[i]);
    }

    output.zeros_last_element(VEC_COUNT*MAX_SIMD_VECTOR_SIZE - Size);

    return output;
}

template<size_t Size>
SIMDVectorLite<Size> SIMDVectorLite<Size>::operator<(const SIMDVectorLite<Size>& v) const
{
    SIMDVectorLite<Size> output;

    for(size_t i=0;i<VEC_COUNT;++i)
    {
        output._vec[i] = this->mask_to_simd(this->_vec[i] < v._vec[i]);
    }

    output.zeros_last_element(VEC_COUNT*MAX_SIMD_VECTOR_SIZE - Size);

    return output;
}

template<size_t Size>
SIMDVectorLite<Size>::~SIMDVectorLite()
{
}

}

template<size_t Size>
std::ostream& operator<<(std::ostream& out,const snn::SIMDVectorLite<Size>& vec)
{
    for(size_t i=0;i<Size;++i)
    {
        std::cout<<vec[i]<<" ";
    }

    return out;
}

template<size_t Size>
snn::SIMDVectorLite<Size> operator+(float num,const snn::SIMDVectorLite<Size>& vec)
{
    return vec+num;
}

template<size_t Size>
snn::SIMDVectorLite<Size> operator-(float num,const snn::SIMDVectorLite<Size>& vec)
{
    return (vec*-1.f)+num;
}

template<size_t Size>
snn::SIMDVectorLite<Size> operator*(float num,const snn::SIMDVectorLite<Size>& vec)
{
    return vec*num;
}

template<size_t Size>
snn::SIMDVectorLite<Size> operator/(float num,const snn::SIMDVectorLite<Size>& vec)
{
    return vec*num/(vec*vec);
}