#pragma once

#include <iostream>
#include <cstdint>
#include <vector>
#include <cmath>

#include <simd_vector_lite.hpp>

#include <config.hpp>

#include "initializers/uniform.hpp"
#include <ranges>

namespace snn
{

    template<size_t InputSize>
    class EvoKAN
    {
        // spline will be formed by using euqation:
        // y*exp(-(u-x)^2 * stretch)

        // y0*exp((x0-x)^2 * b)
        struct SplineNode
        {
            number x0;
            number y0;
            number a;

            SplineNode(number x,number y)
            {
                this->x0 = x;
                this->y0 = y;
            }

            SplineNode()
            {

            }

            void fit(number x,number y)
            {
                this->a = (y-this->y0)/((x-this->x0)*(x-this->x0));
            }

            static size_t required_size_for_serialization()
            {
                return 2*sizeof(number);
            }

            /*
                Funtction used for serialization.

                You need to provied buffer with size from required_size_for_serialization() function.
            */
            void serialize(char* buffer) const
            {
                const size_t buffer_size = 2*sizeof(number);

                memcpy(buffer,(char*)&this->x0,sizeof(number));

                memcpy(buffer+sizeof(number),(char*)&this->y0,sizeof(number));

            }

            /*
                Funtction used for deserialization.

                You need to provied buffer with size from required_size_for_serialization() function.
            */
            void deserialize(char* buffer) 
            {
                memcpy((char*)&this->x0,buffer,sizeof(number));
                memcpy((char*)&this->y0,buffer+sizeof(number),sizeof(number));
            }


        };

        typedef std::shared_ptr<SplineNode> NodeRef;

        


        struct Spline
        {
            private:

            std::vector<NodeRef> nodes;

            number biggest_y;

            number min_x;
            number max_x;

            std::pair<NodeRef,NodeRef> search( number x ) const
            {
                if( this->nodes.size() == 0 )
                {
                    return std::pair(nullptr,nullptr);
                }

                if( this->nodes.size() == 1 )
                {
                    return std::pair(this->nodes[0],this->nodes[0]);
                }

                auto left = this->nodes.begin();
                auto right = this->nodes.end()-1;

                if( (*left)->x0 > x )
                {
                    return std::pair(nullptr,*left);
                }

                if( (*right)->x0 < x )
                {
                    return std::pair(*right,nullptr);
                }

                size_t length = (right - left) + 1;

                while( length > 2 )
                {

                    auto center = left + (length/2);

                    NodeRef node = *center;

                    if( x == node->x0 )
                    {
                        left = center;
                        right = center+1;

                        break;
                    }

                    if( x < node->x0 )
                    {
                        right = center;
                    }
                    else
                    {
                        left = center;
                    }

                    length = (right - left)+1;

                }


                return std::pair(*left,*right);
            }

            public:

            static inline NodeRef make_node(number x,number y)
            {
                return std::make_shared<SplineNode>(x,y);
            }

            static inline NodeRef make_node()
            {
                return std::make_shared<SplineNode>();
            }


            Spline()
            {
                this->biggest_y = 0;
                this->min_x = 0;
                this->max_x = 0;
            }

            void fit(number x,number output,number target)
            {
                number error = abs(output-target);

                auto points = this->search(x); 
                
                // nudge closest points towards (x,target)
                if( error <= ERROR_THRESHOLD_KAN && this->nodes.size() > 0 )
                {

                    if( points.first != nullptr && points.second != nullptr )
                    {
                    
                        NodeRef left = points.first;
                        NodeRef right = points.second;

                        number dx_left = x - left->x0;
                        number dy_left = target - left->y0;

                        number dx_right = x - right->x0;
                        number dy_right = target - right->y0;

                        number coff = std::exp(-1.f*abs(dx_left))*0.25f;

                        left->x0 = left->x0 + dx_left*coff;

                        coff = std::exp(-1.f*abs(dy_left))*0.25f;

                        left->y0 = left->y0 + dy_left*coff;

                        coff = std::exp(-1.f*abs(dx_right))*0.25f;

                        right->x0 = right->x0 + dx_right*coff;

                        coff = std::exp(-1.f*abs(dy_right))*0.25f;

                        right->y0 = right->y0 + dy_right*coff;

                        return;
                    }

                }

                if( points.first != nullptr && points.second != nullptr )
                {
                    if( points.first->x0 == x )
                    {
                        // coverage based on error value
                        number coff = std::exp(-1.f*abs(points.first->y0-target))*0.5f;

                        points.first->y0 = ( points.first->y0 + target )*coff;
                        return;
                    }
                }


                // add new point

                NodeRef new_node = Spline::make_node(x,target);

                this->nodes.push_back(new_node);

                std::sort(this->nodes.begin(),this->nodes.end(),[](NodeRef a, NodeRef b)
                                  {
                                      return a->x0 < b->x0;
                                  });

            }

            NodeRef get_node(number x) const
            {
                
                auto nodes = this->search(x);

                if( nodes.first == nullptr || nodes.second == nullptr )
                {
                    return nullptr;
                }


                NodeRef node = nodes.first;
                NodeRef right = nodes.second;

                if( node->x0 != right->x0 )
                {

                    node->fit(right->x0,right->y0);

                }
                else
                {
                    node->a = 0;
                }

                return node;
            }

            size_t length() const
            {
                return this->nodes.size();
            }

            void save(std::ostream& out) const
            {
                const uint32_t length = this->nodes.size();

                // write total numbers of nodes
                out.write((char*)&length,sizeof(uint32_t));

                const size_t buffer_size = SplineNode::required_size_for_serialization();

                char buffer[buffer_size] = {0};

                for( const NodeRef& node : this->nodes )
                {
                    node->serialize(buffer);

                    out.write(buffer,buffer_size);
                }
                
            }

            void load(std::istream& in)
            {
                uint32_t length = 0;
                
                // read length of nodes
                in.read((char*)&length,sizeof(uint32_t));

                this->nodes.reserve(length);

                const size_t buffer_size = SplineNode::required_size_for_serialization();

                char buffer[buffer_size] = {0};

                for(size_t i=0;i<length;++i)
                {
                    in.read(buffer,buffer_size);

                    NodeRef node = make_node();

                    node->deserialize(buffer);

                    this->nodes.push_back(node);
                }

                // we need to keep the order of nodes the way as they were saved
                std::reverse(this->nodes.begin(),this->nodes.end());

            }

        };


        Spline splines[InputSize];

        // used in fiting process to distribute error
        snn::SIMDVectorLite<InputSize> active_values;

        snn::UniformInit<0.f,1.f> uniform_init;

        public:

        EvoKAN()
        {

            for(size_t i=0;i<InputSize;++i)
            {
                this->active_values[i] = 1.f;
            }

            number prob_mean = this->active_values.reduce();

            this->active_values /= prob_mean;

        }

        number fire(const snn::SIMDVectorLite<InputSize>& input)
        {
            number output = 0.f;

            snn::SIMDVectorLite<InputSize> x(0.f);
            snn::SIMDVectorLite<InputSize> y(0.f);
            snn::SIMDVectorLite<InputSize> a(0.f);

            size_t index = 0;
            

            for(const Spline& spline : this->splines)
            {
                auto node = spline.get_node(input[index]);

                if( node )
                {
                    x[index] = node->x0;
                    y[index] = node->y0;
                    a[index] = node->a;

                }

                index ++;
            }

            snn::SIMDVectorLite<InputSize> x_x = input - x;

            x_x = a*(x_x*x_x) + y;

            output += x_x.reduce();

            // generate fit select probablitiy for each node

            

            return output;
        }

        void fit(const snn::SIMDVectorLite<InputSize>& input,number output,number target)
        {

            number error = abs(output - target);

            // if error is too small, we do not do anything
            if( error < 0.01f )
            {
                return;
            }

            // select node to fit

            size_t node_index = 0;

            snn::SIMDVectorLite<InputSize> y_errors = this->active_values*target;
            snn::SIMDVectorLite<InputSize> output_errors = this->active_values*output;

            size_t index = 0;

            for(Spline& spline : this->splines)
            {
                spline.fit(input[index],output_errors[index],y_errors[index]);

                index ++;
            }

        }

        void printInfo(std::ostream& out = std::cout) const
        {
            out<<"EvoKAN Info:"<<std::endl;
            out<<"Input Size: "<<InputSize<<std::endl;

            for(size_t i=0;i<InputSize;++i)
            {
                out<<"Spline "<<i<<": "<<this->splines[i].length()<<" nodes"<<std::endl;
            }
        }

        void save(std::ostream& out) const
        {
            // save splines
            for(const Spline& spline : this->splines)
            {
                spline.save(out);
            }

        }

        void load(std::istream& in)
        {
            for(Spline& spline : this->splines)
            {
                spline.load(in);
            }
        }

    };

}