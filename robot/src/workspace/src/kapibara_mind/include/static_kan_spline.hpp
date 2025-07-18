#pragma once

#include <vector>

#include <simd_vector_lite.hpp>
#include <misc.hpp>

#include <evo_kan_spline_node.hpp>
#include <config.hpp>

namespace snn
{

    /*!
        A class that represents spline curve used by EVO KAN as activations function.
    */
    template<size_t Size>
    class SplineStatic
    {
        protected:

        std::array<SplineNode*,Size> nodes;

        void sort_nodes();

        public:

        SplineStatic( size_t initial_size = 8 );
        
        void fit(number x,number y);

        std::pair<SplineNode*,SplineNode*> search(number x);

        void remove_redudant_points();

        void smooth_the_spline(const size_t chunk_size);

        void linearization();

        void simplify();

        number fire(number x);

        void printInfo(std::ostream& out);

        void save(std::ostream& out) const;

        void load(std::istream& out);

        ~SplineStatic();

    };


    /*!
            In order for function to work, nodes has to be sorted in asceding order.
    */
    template<size_t Size>
    void SplineStatic<Size>::sort_nodes()
    {
        std::sort(this->nodes.begin(),this->nodes.end(),[](SplineNode* a, SplineNode* b)
                {
                    return a->x < b->x;
                });
    }

    template<size_t Size>
    SplineStatic<Size>::SplineStatic( size_t initial_size )
    {

        number min_x = DEF_X_LEFT;
        number max_x = DEF_X_RIGHT;

        const number step = ( max_x - min_x )/Size;

        DEF_Y_INIT init;

        size_t i = 0;
        
        while( min_x <= max_x )
        {
            number y = init.init();

            SplineNode *node = new SplineNode(min_x,y);

            this->nodes[i++] = node;

            min_x += step;
        }

    }
    
    /*!

        Update SplineStatic with new point.
    
    */
    template<size_t Size>
    void SplineStatic<Size>::fit(number x,number y)
    {
        // Check if point exists aleardy in SplineStatic
        std::pair<SplineNode*,SplineNode*> nodes = this->search(x);

        // Check if points swarming is possible
        if( nodes.first && nodes.second )
        {
            SplineNode* left = nodes.first;
            SplineNode* right = nodes.second;

            SIMDVectorLite<2> dx;

            dx[0] = left->x;
            dx[1] = right->x;

            dx -= x;

            dx*=dx;

            // if x is closer to left nudge left point
            if(dx[0] < dx[1])
            {
                left->y -= 0.1f*( left->y - y );
                
                left->x -= 0.1f*( left->x - x );

                // number a = ( right->y - y ) / ( right->x - x );

                // number new_y = a*( left->x - x ) + y;

                // left->y -= 0.1f*( left->y - new_y );

                return;
            }
            // if x is closer to right nudge right point
            else if(dx[1] < dx[0])
            {
                right->y -= 0.1f*( right->y - y );

                right->x -= 0.1f*( right->x - x );

                // number a = ( left->y - y ) / ( left->x - x );

                // number new_y = a*( right->x - x ) + y;

                // right->y -= 0.1f*( right->y - new_y );

                return;
            }

        }

        // if x is equal to one of the nodes x, nudge y
        if( nodes.first && nodes.first->x == x )
        {
            SplineNode* left = nodes.first;

            left->y -= 0.1f*( left->y - y );

            return;
        }
        // if x is equal to one of the nodes x, nudge y
        if( nodes.second && nodes.second->x == x )
        {
            SplineNode* right = nodes.second;

            right->y -= 0.1f*( right->y - y );

            return;
        }

    }

    /*!
        It use binary search to find pair of points with x between them.
    */
    template<size_t Size>
    std::pair<SplineNode*,SplineNode*> SplineStatic<Size>::search(number x)
    {

        if( this->nodes.size() == 0 )
        {
            return std::pair<SplineNode*,SplineNode*>(nullptr,nullptr);
        }

        if( this->nodes.size() == 1 )
        {
            this->nodes[0]->index = 0;

            return std::pair<SplineNode*,SplineNode*>(this->nodes[0],this->nodes[0]);
        }

        if( x < this->nodes[0]->x )
        {
            this->nodes[0]->index = 0;

            return std::pair<SplineNode*,SplineNode*>(nullptr,this->nodes[0]);
        }
        
        if( x > this->nodes[this->nodes.size()-1]->x )
        {
            this->nodes[this->nodes.size()-1]->index = this->nodes.size()-1;

            return std::pair<SplineNode*,SplineNode*>(this->nodes[this->nodes.size()-1],nullptr);
        }


        size_t p = 0;
        size_t q = this->nodes.size()-1;

        size_t center = (p+q)/2;

        while( (q-p) > 1 )
        {
            SplineNode* center_node = this->nodes[center];

            if( x > center_node->x )
            {
                p = center;
            }
            else if( x < center_node->x )
            {
                q = center;
            }
            else
            {
                center_node->index = center;
                this->nodes[center+1]->index = center+1;

                return std::pair<SplineNode*,SplineNode*>(center_node,this->nodes[center+1]);
            }

            center = (p+q)/2;
        }

        SplineNode* left = this->nodes[center];
        left->index = p;

        SplineNode* right = this->nodes[center+1];
        right->index = q;

        return std::pair<SplineNode*,SplineNode*>(left,right);

    }


    /*!
        Activation function.
    */
    template<size_t Size>
    number SplineStatic<Size>::fire(number x)
    {
        if( nodes.size() == 0 )
        {
            return 0.f;
        }

        std::pair<SplineNode*,SplineNode*> nodes = this->search(x);

        if( !nodes.first && !nodes.second )
        {
            return 0.f;
        }

        if( nodes.first && !nodes.second )
        {
            return nodes.first->x == x ? nodes.first->y : 0;
        }

        if( !nodes.first && nodes.second )
        {
            return nodes.second->x == x ? nodes.second->y : 0;
        }

        if( nodes.first->x == nodes.second->x )
        {
            return nodes.first->x == x ? nodes.first->y : 0; 
        }   

        // let's use linear approximation

        number _x = x - nodes.first->x;

        SplineNode* left = nodes.first;
        SplineNode* right = nodes.second;

        number a = ( right->y - left->y )/( right->x - left->x );

        return a*_x + left->y;

    }


    template<size_t Size>
    void SplineStatic<Size>::printInfo(std::ostream& out)
    {
        out<<"Node count: "<<this->nodes.size()<<std::endl;
    }

    template<size_t Size>
    void SplineStatic<Size>::save(std::ostream& out) const
    {
        uint32_t len = this->nodes.size();

        char len_buffer[4];

        memmove(len_buffer,(char*)&len,4);

        // save amount of nodes stored in spline
        out.write(len_buffer,4);

        constexpr size_t buffor_size = SplineNode::size_for_serialization();

        char buffer[buffor_size];

        for( SplineNode* node : this->nodes )
        {
            node->serialize(buffer);

            out.write(buffer,buffor_size);
        }
    }

    template<size_t Size>
    void SplineStatic<Size>::load(std::istream& in)
    {
        char len_buffer[4];

        in.read(len_buffer,4);

        uint32_t nodes_to_read;        

        memmove((char*)&nodes_to_read,len_buffer,4);

        constexpr size_t buffor_size = SplineNode::size_for_serialization();

        char buffer[buffor_size];

        for(uint32_t i=0;i<nodes_to_read;++i)
        {
            in.read(buffer,buffor_size);

            SplineNode* node = new SplineNode();

            node->deserialize(buffer);

            this->nodes.push_back(node);
        }

    }

    template<size_t Size>
    SplineStatic<Size>::~SplineStatic()
    {
        for( SplineNode* node : this->nodes )
        {
            delete node;
        }

    }

}