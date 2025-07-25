#pragma once

#include <vector>

#include <simd_vector_lite.hpp>
#include <misc.hpp>
#include <evo_kan_spline.hpp>

#include <config.hpp>

namespace snn
{
    
    template<size_t inputSize,class SplineClass = Spline>
    class EvoKan
    {
        protected:

        SplineClass* splines;

        SIMDVectorLite<inputSize> w;

        public:

        EvoKan( size_t initial_size = 0 );
        
        void fit(const SIMDVectorLite<inputSize>& input,number output,number target);

        number fire(const SIMDVectorLite<inputSize>& input);

        void simplify();

        void printInfo( std::ostream& out = std::cout );

        void save(std::ostream& out) const;

        void load(std::istream& in);

        ~EvoKan();

    };


    template<size_t inputSize,class SplineClass>
    EvoKan<inputSize,SplineClass>::EvoKan( size_t initial_size )
    {
        this->splines = new SplineClass[inputSize](initial_size);
    }

    /*!
        Function that update all splines based on input and porpagate target to all of
        them. 

        It also check whether the fit is even neccesary by comparing target with last output.
    */
    template<size_t inputSize,class SplineClass>
    void EvoKan<inputSize,SplineClass>::fit(const SIMDVectorLite<inputSize>& input,number output,number target)
    {

        if( abs(target - output) < ERROR_THRESHOLD_FOR_FIT )
        {
            return;
        }

        number tar = target/static_cast<number>(inputSize);

        for(size_t i=0;i<inputSize;++i)
        {
            this->splines[i].fit(input[i],tar);
        }

    }

    /*!
        Use splines to return activation for input. It use SIMD parralization to speed up a process.
    */
    template<size_t inputSize,class SplineClass>
    number EvoKan<inputSize,SplineClass>::fire(const SIMDVectorLite<inputSize>& input)
    {

        SIMDVectorLite<inputSize> x_left;
        SIMDVectorLite<inputSize> y_left;

        SIMDVectorLite<inputSize> x_right;
        SIMDVectorLite<inputSize> y_right;

        SIMDVectorLite<inputSize> a;

        for(size_t i=0;i<inputSize;++i)
        {
            number _x = input[i];

            std::pair<SplineNode*,SplineNode*> nodes = this->splines[i].search(_x);

            // when both nodes are valid

            SplineNode* left = nodes.first;
            SplineNode* right = nodes.second;

            if( !left && !right)
            {
                y_left[i] = 0.f;
                y_right[i] = 0.f;

                x_left[i] = 0.f;
                x_right[i] = 1.f;

                continue;
            }

            if( left && !right )
            {
                y_left[i] = 0.f;
                y_right[i] = left->x == input[i] ? left->y : 0;

                x_left[i] = 0.f;
                x_right[i] = 1.f;

                continue;
            }

            if( !left && right )
            {
                y_left[i] = 0.f;
                y_right[i] = right->x == input[i] ? right->y : 0;

                x_left[i] = 0.f;
                x_right[i] = 1.f;

                continue;
            }

            if( left == right )
            {

                y_left[i] = 0.f;
                y_right[i] = left->y;

                x_left[i] = 0.f;
                x_right[i] = 1.f;

                continue;
            }

            x_left[i] = left->x;
            y_left[i] = left->y;

            x_right[i] = right->x;
            y_right[i] = right->y;

        }

        SIMDVectorLite<inputSize> x = input - x_left;

        a = ( y_right - y_left )/(x_right - x_left);

        this->w = a*x + y_left;

        return w.reduce();
    }

    template<size_t inputSize,class SplineClass>
    void EvoKan<inputSize,SplineClass>::simplify()
    {
        for(size_t i=0;i<inputSize;++i)
        {
            this->splines[i].simplify();
        }
    }

    template<size_t inputSize,class SplineClass>
    void EvoKan<inputSize,SplineClass>::printInfo( std::ostream& out )
    {
        out<<"EvoKan stats:"<<std::endl;

        for(size_t i=0;i<inputSize;++i)
        {
            out<<"Spline id: "<<i<<std::endl;
            this->splines[i].printInfo(out);
            out<<std::endl;
        }
    }

    template<size_t inputSize,class SplineClass>
    void EvoKan<inputSize,SplineClass>::save(std::ostream& out) const
    {
        for(size_t i=0;i<inputSize;++i)
        {
            this->splines[i].save(out);
        }
    }

    template<size_t inputSize,class SplineClass>
    void EvoKan<inputSize,SplineClass>::load(std::istream& in)
    {
        for(size_t i=0;i<inputSize;++i)
        {
            this->splines[i].load(in);
        }
    }

    template<size_t inputSize,class SplineClass>
    EvoKan<inputSize,SplineClass>::~EvoKan()
    {
        delete [] this->splines;
    }

} // namespace snn
