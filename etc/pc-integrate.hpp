#ifndef _PC_INTEGRATE_HPP_
#define _PC_INTEGRATE_HPP_

#include <cmath>
#include <cassert>
#include <vector>
#include <algorithm>
#include <boost/foreach.hpp>

template <typename T>
struct pc_data
{
    typedef std::vector<T>                          arr_t;
    typedef typename std::vector<T>::const_iterator arr_citr_t;

    pc_data(const T in_dx, const arr_t &in_data, const T in_inf)
        : dx(in_dx), data(in_data), inf(in_inf), current_cell(0), current_sum(0)
    {
    }

    T operator[](const size_t i) const
    {
        return data[i];
    }

    void reset()
    {
        current_cell = 0;
        current_sum  = 0;
    }

    T integrate(const T x) const
    {
        assert(x >= current_cell*dx);

        while((current_cell+1)*dx < x)
        {
            if(current_cell >= data.size())
                break;

            current_sum += data[current_cell]*dx;
            ++current_cell;
        }

        /* i.e. if (current_cell < data.size())
         * const T local = x/dx - current_cell;
         * return current_sum + local*data[current_cell]*dx;
         * else
         * const T local = x - current_cell*dx;
         * return current_sum + local*inf;
         */

        const T last  = (current_cell < data.size()) ? data[current_cell] : inf;
        const T local = x - current_cell*dx;
        return current_sum + local*last;
    }

    T inv_integrate(const T v) const
    {
        assert(v >= current_sum);

        while(current_sum + data[current_cell]*dx < v)
        {
            if(current_cell >= data.size())
                break;

            current_sum += data[current_cell]*dx;
            ++current_cell;
        }

        const T denom = current_cell < data.size() ? data[current_cell] : inf;
        return (v - current_sum)/denom + current_cell*dx;
    }

    void write(std::ostream &o) const
    {
        o << data.size() << " " << dx << " ";
        for(size_t i = 0; i < data.size(); ++i)
            o << data[i] << " ";
        o << inf << std::endl;
    }

    T           dx;
    const arr_t data;
    T           inf;

    mutable size_t current_cell;
    mutable T      current_sum;
};

template <typename F, typename T>
pc_data<T> pc_from_func(const F &func, const T dx, const size_t n)
{
    typename pc_data<T>::arr_t data(n);
    T x = 0;
    BOOST_FOREACH(T &e, data)
    {
        e  = 0.5*(func(x) + func(x+dx));
        x += dx;
    }
    return pc_data<T>(dx, data, func(x));
}

template <typename T>
pc_data<T> pc_from_avg(const typename pc_data<T>::arr_t &obs, const T dx, const size_t n)
{
    const T inv_dx = 1/dx;
    typename pc_data<T>::arr_t data(n, 0);
    BOOST_FOREACH(const T &o, obs)
    {
        if(o < 0.0)
            continue;
        const size_t idx = o*inv_dx;
        if(idx >= n)
            continue;
        data[idx] += inv_dx;
    }
    return pc_data<T>(dx, data, 0);
}
#endif
