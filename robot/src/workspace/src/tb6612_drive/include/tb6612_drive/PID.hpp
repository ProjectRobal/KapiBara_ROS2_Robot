#pragma once



template<typename T>
class PID
{
    private:

    T _p;
    T _i;
    T _d;

    T last_output;

    T max;
    T min;

    bool filtr;

    double _dt;

    public:

    PID()
    {
        filtr=false;
        _p=0;
        _i=0;
        _d=0;
        last_output=0;
        _dt=0.f;
    }

    PID(T p,T i,T d)
    : _p(p),_i(i),_d(d)
    {
        filtr=false;
        last_output=0;
        _dt=0.f;
    }

    void setParams(T p,T i=0,T d=0)
    {
        _p=p;
        if(i)
        {
            setI(i);
        }
        if(d)
        {
            setD(d);
        }
    }

    void setMax(T _max)
    {
        filtr=true;
        max=_max;
    }

    void setMin(T _min)
    {
        filtr=true;
        min=_min;
    }

    void setP(T p)
    {
        _p=p;
    }

    void setI(T i)
    {
        _i=i;
    }

    void setD(T d)
    {
        _d=d;
    }

    const T& P() const
    {
        return _p;
    }

    const T& I() const
    {
        return _i;
    }

    const T& D() const
    {
        return _d;
    }

    void setTimeStep(double dt)
    {
        _dt=dt;
    }

    const double& TimeStep() const
    {
        return _dt;
    }

    T step(T x,double dt)
    {

        last_output=filter(_p*x + _i*x*dt + _d*((x-last_output)/dt));

        return last_output;
    }

    T step(T x)
    {
        return step(x,_dt);
    }

    T filter(T x)
    {   
        if(!filtr)
        {
            return x;
        }

        if(x>max)
        {
            return max;
        }

        if(x<min)
        {
            return min;
        }

        return x;
    }

};

