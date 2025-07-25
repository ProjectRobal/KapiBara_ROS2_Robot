#include "simd_vector.hpp"

namespace snn
{
    SIMD SIMDVector::get_partially_filled_simd(size_t N,number value,number else_number) const
    {
        return SIMD([N,value,else_number](size_t i){ 
            
            if(i<N)
            {
                return value;
            }

            return else_number;
            
            });
    }

    SIMDVector::SIMDVector()
    {
        this->ptr=0;
    }

    SIMDVector::SIMDVector(std::function<number(size_t)> init_func,size_t N)
    : SIMDVector()
    {
        for(size_t i=0;i<N;++i)
        {
            this->append(init_func(i));
        }
    }

    SIMDVector::SIMDVector(number v,size_t N)
    : SIMDVector()
    {
        for(size_t i=0;i<N;++i)
        {
            this->append(v);
        }
    }

    SIMDVector::SIMDVector(const std::initializer_list<number>& arr)
    : SIMDVector()
    {
        for(const auto& a : arr)
        {
            this->append(a);
        }
    }

    SIMDVector::SIMDVector(const SIMDVector& vec)
    {
        this->vec.clear();
        this->ptr=vec.ptr;
        std::copy(vec.vec.begin(),vec.vec.end(), std::back_inserter(this->vec));
    }

    SIMDVector::SIMDVector(SIMDVector&& vec)
    {
        this->vec.clear();
        this->ptr=vec.ptr;
        vec.ptr=0;

        this->vec=std::move(vec.vec);
    }

    void SIMDVector::reserve(size_t N)
    {
        this->ptr=(N%MAX_SIMD_VECTOR_SIZE)+1;

        for(size_t i=0;i<(N/MAX_SIMD_VECTOR_SIZE) + 1;++i)
        {
            this->append(SIMD(0));
        }
    }

    void SIMDVector::operator=(const SIMDVector& vec)
    {
        this->vec.clear();
        this->ptr=vec.ptr;
        std::copy(vec.vec.begin(),vec.vec.end(), std::back_inserter(this->vec));
    }

    void SIMDVector::operator=(SIMDVector&& vec)
    {
        this->vec.clear();
        this->ptr=vec.ptr;
        vec.ptr=0;

        this->vec=std::move(vec.vec);
    }

    void SIMDVector::extend(const SIMDVector& vec)
    {
        for(size_t i=0;i<vec.size();++i)
        {
            this->append(vec[i]);
        }
    }

    void SIMDVector::set(number n, size_t i)
    {
        if(i>=this->size())
        {
            return;
        }

        this->vec[i/MAX_SIMD_VECTOR_SIZE][i%MAX_SIMD_VECTOR_SIZE]=n;
    }

    number SIMDVector::get(const size_t& i) const
    {
        if(i>=this->size())
        {
            return 0;
        }

        return this->vec[i/MAX_SIMD_VECTOR_SIZE][i%MAX_SIMD_VECTOR_SIZE];
    }
    // it is buggy!!!!
    SIMDVector SIMDVector::extract(size_t a,size_t b) const
    {
        if(( b<a )||( a+b > this->size() ))
        {
            return *this;
        }

        SIMDVector output;

        for(size_t i=a;i<b;++i)
        {
            output.append(this->get(i));
        }


        return output;
    }

    const SIMD& SIMDVector::get_block(const size_t& i) const
    {
        if(i>=this->vec.size())
        {
            return this->vec[0];
        }

        return this->vec[i];
    }

    void SIMDVector::shift_block(size_t block_id,size_t from)
    {
        if( block_id >= this->vec.size() )
        {
            return;
        }

        SIMD& block = this->vec[block_id];

        for(size_t i=from;i<MAX_SIMD_VECTOR_SIZE-1;++i)
        {
            block[i] = block[i+1];
        }
    }

    number SIMDVector::pop(size_t i)
    {
        size_t block_id = i / MAX_SIMD_VECTOR_SIZE;

        number to_ret = 0;

        to_ret = this->vec[block_id][i - block_id*MAX_SIMD_VECTOR_SIZE];

        this->shift_block(block_id,i - block_id*MAX_SIMD_VECTOR_SIZE);

        this->vec[block_id][MAX_SIMD_VECTOR_SIZE-1] = this->get((block_id+1)*MAX_SIMD_VECTOR_SIZE);

        while( block_id < this->vec.size() )
        {
            this->shift_block(block_id);

            this->vec[block_id][MAX_SIMD_VECTOR_SIZE-1] = this->get((block_id+1)*MAX_SIMD_VECTOR_SIZE);

            block_id++;
        }

        return to_ret;

    }

    number SIMDVector::pop()
    {
        number ret=0;

        if(this->ptr==0)
        {
            ret=this->vec.back()[0];
            this->vec.pop_back();
            this->ptr=MAX_SIMD_VECTOR_SIZE-1;

            return ret;
        }

        this->ptr--;

        ret=this->vec.back()[this->ptr];
        this->vec.back()[this->ptr]=0;

        return ret;
    }

    number SIMDVector::append(number n)
    {

        if((this->ptr==MAX_SIMD_VECTOR_SIZE)||(this->vec.empty()))
        {
            this->append(SIMD(0));            
            this->ptr=0;
        }

        this->vec.back()[this->ptr]=n;
        this->ptr++;

        return n;
    }

    void SIMDVector::append(const SIMD& simd)
    {
        this->vec.push_back(simd);
    }

    void SIMDVector::append(const SIMD_MASK& mask)
    {
        SIMD dume(0);

        for(size_t i=0;i<mask.size();++i)
        {
            dume[i]=static_cast<number>(mask[i]);
        }

        this->vec.push_back(dume);
    }

    SIMDVector SIMDVector::operator+(const SIMDVector& v) const
    {
        SIMDVector sv;

        for(size_t i=0;i<std::min(this->vec.size(),v.vec.size());++i)
        {
            sv.append(this->vec[i]+v.vec[i]);
        }

        sv.ptr=std::min(this->ptr,v.ptr);

        return sv;
    }

    SIMDVector SIMDVector::operator-(const SIMDVector& v) const
    {
        SIMDVector sv;

        for(size_t i=0;i<std::min(this->vec.size(),v.vec.size());++i)
        {
            sv.append(this->vec[i]-v.vec[i]);
        }

        sv.ptr=std::min(this->ptr,v.ptr);

        return sv;
    }

    SIMDVector SIMDVector::operator*(const SIMDVector& v) const
    {
        SIMDVector sv;

        for(size_t i=0;i<std::min(this->vec.size(),v.vec.size());++i)
        {
            sv.append(this->vec[i]*v.vec[i]);
        }

        sv.ptr=std::min(this->ptr,v.ptr);

        return sv;
    }

    SIMDVector SIMDVector::operator/(const SIMDVector& v) const
    {
        SIMDVector sv;

        SIMDVector _v = v;

        for(size_t i=0;i<std::min(this->vec.size(),_v.vec.size());++i)
        {
            for(size_t o=0;o<MAX_SIMD_VECTOR_SIZE;++o)
            {
                if(_v.vec[i][o] == 0.f)
                {
                    _v.vec[i][o] = 1.f;
                }
            }
            sv.append(this->vec[i]/_v.vec[i]);
        }

        sv.ptr=std::min(this->ptr,v.ptr);

        return sv;
    }

    SIMDVector SIMDVector::operator*(number v) const
    {
        SIMDVector sv;

        for(size_t i=0;i<this->vec.size()-1;++i)
        {
            sv.append(this->vec[i]*v);
        }

        sv.append(this->vec.back()*this->get_partially_filled_simd(this->ptr,v));

        sv.ptr=this->ptr;

        return sv;   
    }

    SIMDVector SIMDVector::operator/(number v) const
    {
        SIMDVector sv;

        for(size_t i=0;i<this->vec.size()-1;++i)
        {
            sv.append(this->vec[i]/v);
        }

        sv.append(this->vec.back()/this->get_partially_filled_simd(this->ptr,v,1.f));

        sv.ptr=this->ptr;

        return sv;   
    }

    SIMDVector SIMDVector::operator-(number v) const
    {
        SIMDVector sv;

        for(size_t i=0;i<this->vec.size()-1;++i)
        {
            sv.append(this->vec[i]-v);
        }

        sv.append(this->vec.back()-this->get_partially_filled_simd(this->ptr,v));

        sv.ptr=this->ptr;

        return sv;   
    }

    SIMDVector SIMDVector::operator+(number v) const
    {
        SIMDVector sv;

        for(size_t i=0;i<this->vec.size()-1;++i)
        {
            sv.append(this->vec[i]+v);
        }

        sv.append(this->vec.back()+this->get_partially_filled_simd(this->ptr,v));

        sv.ptr=this->ptr;

        return sv;   
    }

    SIMDVector SIMDVector::operator==(const SIMDVector& v) const
    {
        SIMDVector sv;

        for(size_t i=0;i<this->vec.size();++i)
        {
            sv.append(this->vec[i]==v.vec[i]);
        }

        for(size_t i=this->ptr;i<MAX_SIMD_VECTOR_SIZE;++i)
        {
            sv.vec.back()[i]=0.f;
        }

        sv.ptr=this->ptr;

        return sv;
    }

    SIMDVector SIMDVector::operator!=(const SIMDVector& v) const
    {
        SIMDVector sv;

        for(size_t i=0;i<this->vec.size();++i)
        {
            sv.append(this->vec[i]!=v.vec[i]);
        }

        for(size_t i=this->ptr;i<MAX_SIMD_VECTOR_SIZE;++i)
        {
            sv.vec.back()[i]=0.f;
        }

        sv.ptr=this->ptr;

        return sv;
    }

    SIMDVector SIMDVector::operator>=(const SIMDVector& v) const
    {
        SIMDVector sv;

        for(size_t i=0;i<this->vec.size();++i)
        {
            sv.append(this->vec[i]>=v.vec[i]);
        }

        for(size_t i=this->ptr;i<MAX_SIMD_VECTOR_SIZE;++i)
        {
            sv.vec.back()[i]=0.f;
        }

        sv.ptr=this->ptr;

        return sv;
    }

    SIMDVector SIMDVector::operator<=(const SIMDVector& v) const
    {
        SIMDVector sv;

        for(size_t i=0;i<this->vec.size();++i)
        {
            sv.append(this->vec[i]<=v.vec[i]);
        }

        for(size_t i=this->ptr;i<MAX_SIMD_VECTOR_SIZE;++i)
        {
            sv.vec.back()[i]=0.f;
        }

        sv.ptr=this->ptr;

        return sv;
    }

    SIMDVector SIMDVector::operator>(const SIMDVector& v) const
    {
        SIMDVector sv;

        for(size_t i=0;i<this->vec.size();++i)
        {
            sv.append(this->vec[i]>v.vec[i]);
        }

        for(size_t i=this->ptr;i<MAX_SIMD_VECTOR_SIZE;++i)
        {
            sv.vec.back()[i]=0.f;
        }

        sv.ptr=this->ptr;

        return sv;
    }

    SIMDVector SIMDVector::operator<(const SIMDVector& v) const
    {
        SIMDVector sv;

        for(size_t i=0;i<this->vec.size();++i)
        {
            sv.append(this->vec[i]<v.vec[i]);
        }

        for(size_t i=this->ptr;i<MAX_SIMD_VECTOR_SIZE;++i)
        {
            sv.vec.back()[i]=0.f;
        }

        sv.ptr=this->ptr;

        return sv;
    }

    SIMDVector SIMDVector::operator==(number v) const
    {
        SIMD siv(v);

        SIMDVector sv;

        for(size_t i=0;i<this->vec.size()-1;++i)
        {
            sv.append(this->vec[i]==siv);
        }

        sv.append(this->vec.back()==this->get_partially_filled_simd(this->ptr,v));

        for(size_t i=this->ptr;i<MAX_SIMD_VECTOR_SIZE;++i)
        {
            sv.vec.back()[i]=0.f;
        }

        sv.ptr=this->ptr;

        return sv;
    }

    SIMDVector SIMDVector::operator!=(number v) const
    {
        SIMD siv(v);

        SIMDVector sv;

        for(size_t i=0;i<this->vec.size()-1;++i)
        {
            sv.append(this->vec[i]!=siv);
        }

        sv.append(this->vec.back()!=this->get_partially_filled_simd(this->ptr,v));

        for(size_t i=this->ptr;i<MAX_SIMD_VECTOR_SIZE;++i)
        {
            sv.vec.back()[i]=0.f;
        }

        sv.ptr=this->ptr;

        return sv;
    }

    SIMDVector SIMDVector::operator>=(number v) const
    {
        SIMD siv(v);

        SIMDVector sv;

        for(size_t i=0;i<this->vec.size()-1;++i)
        {
            sv.append(this->vec[i]>=siv);
        }

        sv.append(this->vec.back()>=this->get_partially_filled_simd(this->ptr,v));

        for(size_t i=this->ptr;i<MAX_SIMD_VECTOR_SIZE;++i)
        {
            sv.vec.back()[i]=0.f;
        }

        sv.ptr=this->ptr;

        return sv;
    }

    SIMDVector SIMDVector::operator<=(number v) const
    {
        SIMD siv(v);

        SIMDVector sv;

        for(size_t i=0;i<this->vec.size()-1;++i)
        {
            sv.append(this->vec[i]<=siv);
        }

        sv.append(this->vec.back()<=this->get_partially_filled_simd(this->ptr,v));

        for(size_t i=this->ptr;i<MAX_SIMD_VECTOR_SIZE;++i)
        {
            sv.vec.back()[i]=0.f;
        }

        sv.ptr=this->ptr;

        return sv;
    }

    SIMDVector SIMDVector::operator>(number v) const
    {
        SIMD siv(v);

        SIMDVector sv;

        for(size_t i=0;i<this->vec.size()-1;++i)
        {
            sv.append(this->vec[i]>siv);
        }

        sv.append(this->vec.back()>this->get_partially_filled_simd(this->ptr,v));

        for(size_t i=this->ptr;i<MAX_SIMD_VECTOR_SIZE;++i)
        {
            sv.vec.back()[i]=0.f;
        }

        sv.ptr=this->ptr;

        return sv;
    }

    SIMDVector SIMDVector::operator<(number v) const
    {
        SIMD siv(v);

        SIMDVector sv;

        for(size_t i=0;i<this->vec.size()-1;++i)
        {
            sv.append(this->vec[i]<siv);
        }

        sv.append(this->vec.back()<this->get_partially_filled_simd(this->ptr,v,v-1));

        for(size_t i=this->ptr;i<MAX_SIMD_VECTOR_SIZE;++i)
        {
            sv.vec.back()[i]=0.f;
        }

        sv.ptr=this->ptr;

        return sv;
    }

    void SIMDVector::operator+=(const SIMDVector& v)
    {

        for(size_t i=0;i<std::min(this->vec.size(),v.vec.size());++i)
        {
            this->vec[i]+=v.vec[i];
        }

    }

    void SIMDVector::operator-=(const SIMDVector& v)
    {
        for(size_t i=0;i<std::min(this->vec.size(),v.vec.size());++i)
        {
            this->vec[i]-=v.vec[i];
        }
    }

    void SIMDVector::operator*=(const SIMDVector& v)
    {
        for(size_t i=0;i<std::min(this->vec.size(),v.vec.size());++i)
        {
            this->vec[i]*=v.vec[i];
        }
    }

    void SIMDVector::operator/=(const SIMDVector& v)
    {
        SIMDVector _v = v;

        for(size_t i=0;i<std::min(this->vec.size(),v.vec.size());++i)
        {
            for(size_t o=0;o<MAX_SIMD_VECTOR_SIZE;++o)
            {
                if(_v.vec[i][o] == 0.f)
                {
                    _v.vec[i][o] = 1.f;
                }
            }
            this->vec[i]/=v.vec[i];
        }
    }

    void SIMDVector::operator*=(number v)
    {
        for(size_t i=0;i<this->vec.size()-1;++i)
        {
            this->vec[i]*=v;
        }

        this->vec.back()*=this->get_partially_filled_simd(this->ptr,v);
    }

    void SIMDVector::operator/=(number v)
    {
        for(size_t i=0;i<this->vec.size()-1;++i)
        {
            this->vec[i]/=v;
        }

        this->vec.back()/=this->get_partially_filled_simd(this->ptr,v,1.f);
    }

    void SIMDVector::operator-=(number v)
    {
        for(size_t i=0;i<this->vec.size()-1;++i)
        {
            this->vec[i]-=v;
        }

        this->vec.back()-=this->get_partially_filled_simd(this->ptr,v);
    }

    void SIMDVector::operator+=(number v)
    {
        for(size_t i=0;i<this->vec.size()-1;++i)
        {
            this->vec[i]+=v;
        }

        this->vec.back()+=this->get_partially_filled_simd(this->ptr,v);
    }

    number SIMDVector::reduce() const
    {
        number output=0;

        for(const auto& a : this->vec)
        {
            output+=std::experimental::reduce(a);
        }

        return output;
    }

    number SIMDVector::length() const
    {
        number output=0;

        snn::SIMDVector vec = (*this)*(*this);

        for(const auto& a : vec.vec)
        {
            output+=std::experimental::reduce(a);
        }

        return std::sqrt( output );
    }

    number SIMDVector::operator[](const size_t& i) const
    {
        return get(i);
    }

    SIMDVector::~SIMDVector()
    {
        this->vec.clear();
    }
}


std::ostream& operator<<(std::ostream& out,const snn::SIMDVector& vec)
{

    vec.print(out);

    return out;
}


snn::SIMDVector operator*(number v,const snn::SIMDVector& vec)
{
    return vec*v;
}

snn::SIMDVector operator/(number v,const snn::SIMDVector& vec)
{
    return (vec*v)/(vec*vec);
}

snn::SIMDVector operator-(number v,const snn::SIMDVector& vec)
{
    return (vec*-1)+v;
}

snn::SIMDVector operator+(number v,const snn::SIMDVector& vec)
{
    return vec+v;
}