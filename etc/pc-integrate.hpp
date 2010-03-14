#ifndef _PC_INTEGRATE_HPP_
#define _PC_INTEGRATE_HPP_

#include <cmath>
#include <cassert>
#include <vector>
#include <algorithm>
#include <boost/utility.hpp>
#include <boost/foreach.hpp>

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

template <typename F, typename T>
pc_data<T> pc_from_func(const F &func, const T dx, const size_t n)
{
    std::vector<T> data(n);
    T x = 0;
    for(size_t i = 0; i < n; ++i)
    {
        data[i] = 0.5*(func(x) + func(x+dx));
        x += dx;
    }
    return pc_data<T>(dx, data, n);
}

template <typename T>
pc_data<T> pc_from_avg(const std::vector<T> &obs, const T dx, const size_t n)
{
    const T inv_dx = 1/dx;
    std::vector<T> data(n, 0);
    BOOST_FOREACH(const T &o, obs)
    {
        if(o < 0.0)
            continue;
        const size_t idx = o*inv_dx;
        if(idx >= n)
            continue;
        data[idx] += inv_dx;
    }
    return pc_data<T>(dx, data, n);
}
#endif