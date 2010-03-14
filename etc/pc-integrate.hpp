#ifndef _PC_INTEGRATE_HPP_
#define _PC_INTEGRATE_HPP_

#include <cmath>
#include <cassert>
#include <vector>
#include <algorithm>
#include <boost/utility.hpp>

template <typename T>
struct pc_data
{
    pc_data(const T in_dx, const std::vector<T> &in_data, const T in_inf)
        : dx(in_dx), inf(in_inf)
    {
        build(in_data);
    }

    T operator[](const size_t i) const
    {
        return (integration[i+1]-integration[i])/dx;
    }

    void build(const std::vector<T> &data)
    {
        integration.resize(data.size()+1);
        integration[0] = 0.0f;
        for(size_t i = 1; i < integration.size(); ++i)
            integration[i] = integration[i-1] + data[i-1]*dx;
    }

    T integrate(const T x) const
    {
        if(x > (integration.size()-1)*dx)
        {
            const T local = x - (integration.size()-1)*dx;
            return integration.back() + local*inf;
        }

        const T      scaled = x/dx;
        const size_t cell   = std::floor(scaled);
        const T      local  = scaled - cell;
        return (1 - local)*integration[cell] + local*integration[cell+1];
    }

    T inv_integrate(const T v) const
    {
        typename std::vector<T>::const_iterator s = integration.begin();
        return inv_integrate(v, s);
    }

    T inv_integrate(const T v, typename std::vector<T>::const_iterator &start) const
    {
        assert(v >= integration.front());

        T x;
        if( v >= integration.back())
        {
            const T last_v = integration.back();
            x = (v - last_v)/inf + (integration.size()-1)*dx;
            start = integration.end();
        }
        else
        {
            assert(start < integration.end());
            typename std::vector<T>::const_iterator idx = boost::prior(std::upper_bound(start, integration.end(), v));
            const size_t idx_no = idx - integration.begin();
            x = (v - *idx)/(*this)[idx_no] + idx_no * dx;
            start = idx;
        }

        return x;
    }

    T                     dx;
    std::vector<T>        integration;
    T                     inf;
};
#endif
