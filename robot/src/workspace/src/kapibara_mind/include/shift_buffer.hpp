#pragma once

#include <cstdint>
#include <cstddef>


template<typename T,size_t N>
class ShiftBuffer
{
    protected:

    T buff[N];

    size_t iter;

    size_t size;

    public:

    ShiftBuffer();
    
    void push(const T& obj);

    size_t length() const;

    bool isFull() const;

    void reset();

    const T& get(size_t i) const;

};


template<typename T,size_t N>
ShiftBuffer<T,N>::ShiftBuffer()
{
    this->reset();
}

template<typename T,size_t N>
void ShiftBuffer<T,N>::push(const T& obj)
{
    this->buff[ this->iter ] = obj;

    this->iter ++;

    this->iter = this->iter - (this->iter/N)*N;

    if( this->size < N )
    {
        this->size ++;
    }
}

template<typename T,size_t N>
size_t ShiftBuffer<T,N>::length() const
{
    return this->size;
}

template<typename T,size_t N>
bool ShiftBuffer<T,N>::isFull() const
{
    return this->size == N;
}

template<typename T,size_t N>
void ShiftBuffer<T,N>::reset()
{
    this->size = 0;
    this->iter = 0;
}

template<typename T,size_t N>
const T& ShiftBuffer<T,N>::get(size_t i) const
{
    // this->iter is going to be 0 index

    if( i > this->size )
    {
        return T();
    }

    int32_t index = this->iter - 1 - i;

    if( index < 0)
    {
        index = N + index;
    }

    return this->buff[ index ];
}
