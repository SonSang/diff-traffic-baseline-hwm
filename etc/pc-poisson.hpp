#ifndef _PC_POISSON_HPP_
#define _PC_POISSON_HPP_

#include "pc-integrate.hpp"

template <typename T>
inline T exp_rvar(const T r)
{
    return -std::log(r);
}

template <typename T>
struct inhomogeneous_poisson
{
    inhomogeneous_poisson(const T in_start, const pc_data<T> &in_pc) : t(in_start),
                                                                       integrator(&in_pc),
                                                                       arg(integrator.integrate(t))
    {
    }

    T next()
    {
        const T u = drand48();
        const T e = exp_rvar(u);
        arg += e;
        t = integrator.inv_integrate(arg);
        return t;
    }

    T next_trunc(const float trunc)
    {
        const T u = drand48();
        pc_integrator<pc_data<T> > i2(integrator);
        const T d = 1 - std::exp(-(i2.integrate(trunc) - arg));
        const T e = exp_rvar(1 - d*u);
        arg += e;
        t = integrator.inv_integrate(arg);
        return t;
    }

    T t;
    pc_integrator<pc_data<T> > integrator;
    T arg;

};

template <typename T>
std::vector<T> poisson_points(const T start, const T end, const size_t quota, const T sep, const pc_data<T> &pc)
{
    inhomogeneous_poisson<T> ipp(start, pc);

    std::vector<T> res;
    T candidate = ipp.next();
    while(res.size() < quota && candidate < end)
    {
        res.push_back(candidate);

        while(1)
        {
            inhomogeneous_poisson<T> ipp_copy(ipp);
            candidate = ipp.next();
            if(candidate - res.back() >= sep)
                break;
            ipp = ipp_copy;
        }
    }

    return res;
}
#endif
