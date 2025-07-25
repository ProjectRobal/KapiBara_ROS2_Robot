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
    class Spline
    {
        protected:

        std::vector<SplineNode*> nodes;

        void sort_nodes();

        void add_node(SplineNode* node);

        public:

        Spline( size_t initial_size = 8 );
        
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

        ~Spline();

    };


    /*!
            In order for function to work, nodes has to be sorted in asceding order.
    */
    void Spline::sort_nodes()
    {
        std::sort(this->nodes.begin(),this->nodes.end(),[](SplineNode* a, SplineNode* b)
                {
                    return a->x < b->x;
                });
    }

    /*
        Use insertion sort to keep nodes sorted during node insertion.
    */
    void Spline::add_node(SplineNode* node)
    {
        auto loc = std::lower_bound(this->nodes.begin(),this->nodes.end(),node->x,
        [](SplineNode* a,number x){

            return a->x < x;
        });

        this->nodes.insert(loc,node);
    }

    Spline::Spline( size_t initial_size )
    {

        if( initial_size == 0 )
        {
            return;
        }

        this->nodes.reserve(initial_size);


        number min_x = DEF_X_LEFT;
        number max_x = DEF_X_RIGHT;

        const number step = ( max_x - min_x )/initial_size;

        DEF_Y_INIT init;
        
        while( min_x <= max_x )
        {
            number y = init.init();

            SplineNode *node = new SplineNode(min_x,y);

            this->nodes.push_back(node);

            min_x += step;
        }

    }
    
    /*!

        Update spline with new point.
    
    */
    void Spline::fit(number x,number y)
    {
        // Check if point exists aleardy in spline
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
            if(dx[0] < dx[1]  && dx[0] < ERROR_THRESHOLD_FOR_INSERTION)
            {
                left->y -= 0.1f*( left->y - y );
                
                left->x -= 0.01f*( left->x - x );

                // if points are very close to each other remove one of them
                if( abs( left->x - right->x ) < ERROR_THRESHOLD_FOR_POINT_REMOVAL)
                {
                    this->nodes.erase(this->nodes.begin()+left->index);
                }

                // this->sort_nodes();

                return;
            }
            // if x is closer to right nudge right point
            else if(dx[1] < dx[0] && dx[1] < ERROR_THRESHOLD_FOR_INSERTION)
            {
                right->y -= 0.1f*( right->y - y );

                right->x -= 0.01f*( right->x - x );

                // if points are very close to each other remove one of them
                if( abs( left->x - right->x ) < ERROR_THRESHOLD_FOR_POINT_REMOVAL)
                {
                    this->nodes.erase(this->nodes.begin()+right->index);
                }
                
                // this->sort_nodes();

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

        // // if there is no such point present insert it into spline.
        SplineNode* node = new SplineNode(x,y);

        // this->nodes.push_back(node);

        // this->sort_nodes();

        this->add_node(node);
    }

    /*!
        It use binary search to find pair of points with x between them.
    */
    std::pair<SplineNode*,SplineNode*> Spline::search(number x)
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

    void Spline::remove_redudant_points()
    {
        auto iter = this->nodes.begin();
        
        while( (iter+1) != this->nodes.end() )
        {
            auto next_iter = iter + 1;

            number dx = (*iter)->x-(*next_iter)->x;

            number dy = (*iter)->y-(*next_iter)->y;

            number distance = dx*dx + dy*dy;

            if( distance <= 0.000001f )
            {
                SplineNode* node = (*next_iter);

                delete node;

                this->nodes.erase(next_iter);
            }

            iter++;
        }
    }

    void Spline::smooth_the_spline(const size_t chunk_size)
    {
        auto iter = this->nodes.begin();

        std::vector<SplineNode*> new_nodes;
        
        size_t i = 0;

        number x = 0;
        number y = 0;

        while( iter!= this->nodes.end() )
        {
            SplineNode* node = (*iter);

            x += node->x;
            y += node->y;

            i++;

            iter++;

            if( i == chunk_size )
            {
                SplineNode* mean_node = new SplineNode(x/chunk_size,y/chunk_size);

                new_nodes.push_back(mean_node);

                i = 0;

                x = 0;
                y = 0;

            }

        }

        for(SplineNode* node : this->nodes)
        {
            delete node;
        }

        this->nodes.clear();

        this->nodes = std::move(new_nodes);

        this->sort_nodes();

    }

    void Spline::linearization()
    {
        auto iter = this->nodes.begin();
        auto first_iter = this->nodes.begin();

        std::vector<SplineNode*> new_nodes;


    }

    void Spline::simplify()
    {
        // remove close points
        this->remove_redudant_points();

        // it isn't ideal
        // this->smooth_the_spline(4);

        this->linearization();
    }

    /*!
        Activation function.
    */
    number Spline::fire(number x)
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


    void Spline::printInfo(std::ostream& out)
    {
        out<<"Node count: "<<this->nodes.size()<<std::endl;
    }

    void Spline::save(std::ostream& out) const
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

    void Spline::load(std::istream& in)
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

    Spline::~Spline()
    {
        for( SplineNode* node : this->nodes )
        {
            delete node;
        }

        this->nodes.clear();
    }

}