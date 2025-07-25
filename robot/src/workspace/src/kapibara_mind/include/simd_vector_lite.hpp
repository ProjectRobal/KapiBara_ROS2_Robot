#pragma once

#include <cmath>
#include <cstdint>
#include <variant>
#include <functional>

#include "config.hpp"

namespace snn
{

#define VEC_COUNT (static_cast<size_t>(Size/MAX_SIMD_VECTOR_SIZE))

#define VEC_REMAINDER (static_cast<size_t>(Size - VEC_COUNT*MAX_SIMD_VECTOR_SIZE))

template<size_t Size>
class SIMDVectorLite
{
private:

    struct Empty
    {};

    /* data */
    SIMD _vec[VEC_COUNT];

    using remainder_simd = std::experimental::fixed_size_simd<number , VEC_REMAINDER >;

    using remainder_simd_mask = std::experimental::fixed_size_simd_mask<number , VEC_REMAINDER >;

    using remainder_type = std::conditional_t< VEC_REMAINDER != 0, remainder_simd ,Empty >;

    using remainder_type_mask =  std::conditional_t< VEC_REMAINDER != 0, remainder_simd_mask ,Empty >;

    using simd_ref_variant = std::variant<std::reference_wrapper<SIMD_ref>,std::reference_wrapper<typename remainder_simd::reference>>;

    using simd_variant = std::conditional_t<
        VEC_REMAINDER != 0,
        // The variant when there IS a remainder
        std::variant<
            std::reference_wrapper<SIMD>,
            std::reference_wrapper<remainder_simd>
        >,
        // The variant when there is NO remainder
        std::variant<
            std::reference_wrapper<SIMD>
        >
    >;

    using simd_variant_ref = std::conditional_t<
            VEC_REMAINDER != 0,
            // Variant for a vector with a remainder (two alternatives)
            std::variant<
                SIMD::reference,
                typename remainder_simd::reference
            >,
            // Variant for a vector with no remainder (one alternative)
            std::variant<
                SIMD::reference
            >
        >;


    remainder_type remainder;

    class SIMD_reference
    {
        private:

        size_t index;

        SIMD* _simd;
        remainder_type* _remainder;

        public:

        SIMD_reference(SIMD* _simd,size_t index)
        {
            this->index = index;

            this->_simd = _simd;
            this->_remainder = nullptr;
        }

        SIMD_reference(remainder_type* _remainder,size_t index)
        {
            this->index = index;

            this->_remainder = _remainder;
            this->_simd = nullptr;
        }

        operator number() const
        {
            if constexpr(VEC_REMAINDER != 0)
            {
                if( this->_simd )
                {
                    return (*this->_simd)[this->index];
                }
                else
                {
                    return (*this->_remainder)[this->index];   
                }
            }

            return (*this->_simd)[this->index];
        }

        number operator()() const
        {
            if constexpr(VEC_REMAINDER != 0)
            {
                if( this->_simd )
                {
                    return (*this->_simd)[this->index];
                }
                else
                {
                    return (*this->_remainder)[this->index];   
                }
            }

            return (*this->_simd)[this->index];
        }

        void operator=(number n)
        {
            if constexpr(VEC_REMAINDER != 0)
            {
                if( this->_simd )
                {
                    (*this->_simd)[this->index] = n;
                }
                else
                {
                    (*this->_remainder)[this->index] = n;   
                }
            }
            else
            {

                (*this->_simd)[this->index] = n;

            }
        }

    };

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

    remainder_type mask_to_remainder(const remainder_type_mask& mask) const
    {

        remainder_type output(0);

        for(uint16_t i = 0; i< MAX_SIMD_VECTOR_SIZE; ++i)
        {
            output[i] = mask[i] ? 1.f : 0.f;

        }

        return output;

    }


public:
    SIMDVectorLite();

    SIMDVectorLite(number nm);

    SIMDVectorLite(const std::array<number,Size>& arr);

    // SIMDVectorLite(const SIMDVectorLite& vec);

    // SIMDVectorLite(SIMDVectorLite&& vec);

    number reduce() const;

    number operator[](size_t i) const;

    SIMD_reference operator[](size_t i);

    template<size_t OutputSize>
    SIMDVectorLite<OutputSize> split();

    size_t size() const
    {
        return Size;
    }

    void set(size_t i,number v);

    void set_block(size_t i,simd_variant block)
    {
        if constexpr(VEC_REMAINDER != 0)
        {

            if( i == VEC_COUNT )
            {
                if( block.index() != 1 )
                {
                    return;
                }
                
                this->remainder = block;
                return;
            }

        }


        if(i>=VEC_COUNT)
        {
            return;
        }

        if( block.index() != 0 )
        {
            return;
        }

        this->_vec[i] = block;
    }

    simd_variant get_block(size_t i)
    {        
        if( i == VEC_COUNT )
        {
            return std::ref(this->remainder);
        }

        if(i>=VEC_COUNT)
        {
            i = 0;
        }
        
        return std::ref(this->_vec[i]);
    }

    // void operator=(const SIMDVectorLite& vec);

    // void operator=(SIMDVectorLite&& vec);

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

    snn::SIMDVectorLite<Size> exp();

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

    if constexpr(VEC_REMAINDER != 0)
    {
        this->remainder = remainder_type(0);
    }
}


// template<size_t Size>
// SIMDVectorLite<Size>::SIMDVectorLite(const SIMDVectorLite& vec)
// {
//     for(size_t i=0;i<VEC_COUNT;++i)
//     {
//         this->_vec[i] = vec._vec[i];
//     }

//     if constexpr(VEC_REMAINDER != 0)
//     {
//         this->remainder = vec.remainder;
//     }
// }


// template<size_t Size>
// SIMDVectorLite<Size>::SIMDVectorLite(SIMDVectorLite&& vec)
// {
//     for(size_t i=0;i<VEC_COUNT;++i)
//     {
//         this->_vec[i] = std::move(vec._vec[i]);
//     }

//     if constexpr(VEC_REMAINDER != 0)
//     {
//         this->remainder = std::move(vec.remainder);
//     }
// }

template<size_t Size>
SIMDVectorLite<Size>::SIMDVectorLite(number num)
{
    
    for(size_t i=0;i<VEC_COUNT;++i)
    {
        this->_vec[i] = SIMD(num);
    }

    if constexpr(VEC_REMAINDER != 0)
    {

        this->remainder = remainder_type(num);

    }

}

template<size_t Size>
SIMDVectorLite<Size>::SIMDVectorLite(const std::array<number,Size>& arr)
{
    for(size_t i=0;i<Size;++i)
    {
        this->set(i,arr[i]);
    }
}

// template<size_t Size>
// void SIMDVectorLite<Size>::operator=(const SIMDVectorLite& vec)
// {
//     for(size_t i=0;i<VEC_COUNT;++i)
//     {
//         this->_vec[i] = vec._vec[i];
//     }

//     if constexpr(VEC_REMAINDER != 0)
//     {
//         this->remainder = vec.remainder;
//     }
// }

// template<size_t Size>
// void SIMDVectorLite<Size>::operator=(SIMDVectorLite&& vec)
// {
//     for(size_t i=0;i<VEC_COUNT;++i)
//     {
//         this->_vec[i] = std::move(vec._vec[i]);
//     }

//     if constexpr(VEC_REMAINDER != 0)
//     {
//         this->remainder = std::move(vec.remainder);
//     }
// }

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
number SIMDVectorLite<Size>:: reduce() const
{
    number output = 0;

    for(const SIMD& simd : this->_vec)
    {
        output+=std::experimental::reduce(simd);
    }

    if constexpr( VEC_REMAINDER != 0 )
    {
        output += std::experimental::reduce(this->remainder);
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

    if constexpr( VEC_REMAINDER != 0 )
    {
        if( i >= VEC_COUNT*MAX_SIMD_VECTOR_SIZE )
        {
            i = i  - static_cast<size_t>(i/MAX_SIMD_VECTOR_SIZE)*MAX_SIMD_VECTOR_SIZE;

            return this->remainder[i];
        }
    }

    size_t simd_id = i/MAX_SIMD_VECTOR_SIZE;

    return _vec[simd_id][i - simd_id*MAX_SIMD_VECTOR_SIZE];
}

template<size_t Size>
SIMDVectorLite<Size>::SIMD_reference SIMDVectorLite<Size>::operator[](size_t i)
{
    if(i>=Size)
    {
        i = 0;
    }


    if constexpr( VEC_REMAINDER != 0 )
    {
        if( i >= VEC_COUNT*MAX_SIMD_VECTOR_SIZE )
        {
            i = i  - static_cast<size_t>(i/MAX_SIMD_VECTOR_SIZE)*MAX_SIMD_VECTOR_SIZE;

            return SIMD_reference(&this->remainder,i);
        }
    }

    size_t simd_id = i/MAX_SIMD_VECTOR_SIZE;

    // number& ref = _vec[simd_id][i - simd_id*MAX_SIMD_VECTOR_SIZE];

    // return _vec[simd_id][i - simd_id*MAX_SIMD_VECTOR_SIZE];

    return SIMD_reference(&_vec[simd_id],i - simd_id*MAX_SIMD_VECTOR_SIZE);
}

template<size_t Size>
void SIMDVectorLite<Size>::set(size_t i,number v)
{
    if(i>=Size)
    {
        return;
    }

    if constexpr( VEC_REMAINDER != 0 )
    {
        if( i > VEC_COUNT*MAX_SIMD_VECTOR_SIZE )
        {
            i = i  - static_cast<size_t>(i/MAX_SIMD_VECTOR_SIZE)*MAX_SIMD_VECTOR_SIZE;

            this->remainder[i] = v;
        }
    }

    size_t simd_id = i/MAX_SIMD_VECTOR_SIZE;

    _vec[simd_id][i - simd_id*MAX_SIMD_VECTOR_SIZE] = v;
}

template<size_t Size>
void SIMDVectorLite<Size>::operator+=(number v)
{
    
    for(size_t i=0;i<VEC_COUNT;++i)
    {
        this->_vec[i] += v;
    }

    if constexpr( VEC_REMAINDER != 0 )
    {
        this->remainder += v;
    }

}

template<size_t Size>
void SIMDVectorLite<Size>::operator-=(number v)
{
    
    for(size_t i=0;i<VEC_COUNT;++i)
    {
        this->_vec[i] -= v;
    }

    if constexpr( VEC_REMAINDER != 0 )
    {
        this->remainder -= v;
    }
    
}

template<size_t Size>
void SIMDVectorLite<Size>::operator*=(number v)
{
    for(SIMD& simd : this->_vec)
    {
        simd*=v;
    }

    if constexpr( VEC_REMAINDER != 0 )
    {
        this->remainder *= v;
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

    if constexpr( VEC_REMAINDER != 0 )
    {
        this->remainder /= v;
    }
}

template<size_t Size>
SIMDVectorLite<Size> SIMDVectorLite<Size>::operator+(number v) const
{
    SIMDVectorLite<Size> output;
    
    for(size_t i=0;i<VEC_COUNT;++i)
    {
        output._vec[i] = this->_vec[i] + v;
    }

    if constexpr( VEC_REMAINDER != 0 )
    {
        output.remainder = this->remainder + v;
    }

    return output;

}

template<size_t Size>
SIMDVectorLite<Size> SIMDVectorLite<Size>::operator-(number v) const
{
    SIMDVectorLite<Size> output;
    
    for(size_t i=0;i<VEC_COUNT;++i)
    {
        output._vec[i] = this->_vec[i] - v;
    }

    if constexpr( VEC_REMAINDER != 0 )
    {
        output.remainder = this->remainder - v;
    }

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

    if constexpr( VEC_REMAINDER != 0 )
    {
        output.remainder = this->remainder * v;
    }

    return output;

}

template<size_t Size>
SIMDVectorLite<Size> SIMDVectorLite<Size>::operator/(number v) const
{

    SIMDVectorLite<Size> output;
    
    for(size_t i=0;i<VEC_COUNT;++i)
    {
        output._vec[i] = this->_vec[i] / v;
    }

    if constexpr( VEC_REMAINDER != 0 )
    {
        output.remainder = this->remainder / v;
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

    if constexpr( VEC_REMAINDER != 0 )
    {
        this->remainder += v.remainder;
    }
}

template<size_t Size>
void SIMDVectorLite<Size>::operator-=(const SIMDVectorLite<Size>& v)
{
    for(size_t i=0;i<VEC_COUNT;++i)
    {
        this->_vec[i] -= v._vec[i];
    }

    if constexpr( VEC_REMAINDER != 0 )
    {
        this->remainder -= v.remainder;
    }
}

template<size_t Size>
void SIMDVectorLite<Size>::operator*=(const SIMDVectorLite<Size>& v)
{
    for(size_t i=0;i<VEC_COUNT;++i)
    {
        this->_vec[i] *= v._vec[i];
    }

    if constexpr( VEC_REMAINDER != 0 )
    {
        this->remainder *= v.remainder;
    }
}

template<size_t Size>
void SIMDVectorLite<Size>::operator/=(const SIMDVectorLite<Size>& v)
{
    for(size_t i=0;i<VEC_COUNT;++i)
    {
        this->_vec[i] /= v._vec[i];
    }

    if constexpr( VEC_REMAINDER != 0 )
    {
        this->remainder /= v.remainder;
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

    if constexpr( VEC_REMAINDER != 0 )
    {
        output.remainder = this->remainder + v.remainder;
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

    if constexpr( VEC_REMAINDER != 0 )
    {
        output.remainder = this->remainder - v.remainder;
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

    if constexpr( VEC_REMAINDER != 0 )
    {
        output.remainder = this->remainder * v.remainder;
    }

    return output;
}

template<size_t Size>
SIMDVectorLite<Size> SIMDVectorLite<Size>::operator/(const SIMDVectorLite<Size>& v) const
{
    SIMDVectorLite<Size> output;
    
    for(size_t i=0;i<VEC_COUNT;++i)
    {
        output._vec[i] = this->_vec[i] / v._vec[i];
    }

    if constexpr( VEC_REMAINDER != 0 )
    {
        output.remainder = this->remainder / v.remainder;
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

    if constexpr( VEC_REMAINDER != 0 )
    {

        output.remainder = this->mask_to_remainder(this->remainder == v.remainder);

    }

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

    if constexpr( VEC_REMAINDER != 0 )
    {

        output.remainder = this->mask_to_remainder(this->remainder != v.remainder);

    }

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

    if constexpr( VEC_REMAINDER != 0 )
    {

        output.remainder = this->mask_to_remainder(this->remainder >= v.remainder);

    }

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

    if constexpr( VEC_REMAINDER != 0 )
    {

        output.remainder = this->mask_to_remainder(this->remainder <= v.remainder);

    }

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

    if constexpr( VEC_REMAINDER != 0 )
    {

        output.remainder = this->mask_to_remainder(this->remainder > v.remainder);

    }

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

    if constexpr( VEC_REMAINDER != 0 )
    {

        output.remainder = this->mask_to_remainder(this->remainder < v.remainder);

    }

    return output;
}

template<size_t Size>
SIMDVectorLite<Size>::~SIMDVectorLite()
{
}

template<size_t Size>
snn::SIMDVectorLite<Size> snn::SIMDVectorLite<Size>::exp()
{

    snn::SIMDVectorLite<Size> output(0.f);

    for(size_t i=0;i<VEC_COUNT;++i)
    {
        output._vec[i] = std::experimental::exp(this->_vec[i]);
    }

    if constexpr( VEC_REMAINDER != 0 )
    {

        output.remainder = std::experimental::exp(this->remainder);

    }

    return output;
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

