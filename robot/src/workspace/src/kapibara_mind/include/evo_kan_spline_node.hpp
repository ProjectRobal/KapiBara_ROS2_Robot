#pragma once

#include <config.hpp>

namespace snn
{
    /*!
        A struct that represents point in spline curve with x and y coordinats, it also
        support basic serialization and deserialization.
    */
    struct SplineNode
    {
        number x;
        number y;
        size_t index;

        SplineNode(){}

        SplineNode(number x,number y)
        {
            this->x = x;
            this->y = y;
        }

        static constexpr size_t size_for_serialization()
        {
            return 2*SERIALIZED_NUMBER_SIZE;
        }

        char* serialize() const
        {
            char* buffer = new char[this->size_for_serialization()];

            this->serialize(buffer);

            return buffer;
        }

        void serialize(char* buffer) const
        {
            serialize_number<number>(this->x,buffer);

            serialize_number<number>(this->y,buffer+SERIALIZED_NUMBER_SIZE);
        }

        void deserialize(char* buffer)
        {
            this->x = deserialize_number<number>(buffer);

            this->y = deserialize_number<number>(buffer+SERIALIZED_NUMBER_SIZE);
        }


    };
}