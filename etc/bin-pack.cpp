#include <iostream>
#include <vector>
#include <cmath>
#include <cassert>
#include <boost/foreach.hpp>
#include <boost/range/iterator_range.hpp>
#include <boost/assign/std/vector.hpp>
#include <boost/utility.hpp>

typedef std::vector<float>                      joblist;
typedef std::vector<float>::iterator            job_iter;
typedef std::vector<float>::const_iterator      const_job_iter;
typedef boost::iterator_range<job_iter>         job_range;
typedef boost::iterator_range<const_job_iter>   const_job_range;

typedef std::vector<joblist>                    binlist;

typedef std::vector<int>                   config;
typedef config::iterator                   config_iter;
typedef boost::iterator_range<config_iter> config_range;

using namespace boost::assign;

static float size(const joblist &jobs, int nprocs)
{
    float sum = 0.0f;
    float maxjob = 0.0f;
    BOOST_FOREACH(const float &j, jobs)
    {
        sum += j;
        maxjob = std::max(j, maxjob);
    }
    return std::max(sum/nprocs, maxjob);
}

// binlist eps_makespan(const std::vector<float> &jobs, int nprocs, float eps)
// {
//     float lower = size(jobs, nprocs);
//     float upper = 2.0f*lowers;
//     while(std::abs(upper-lower) > 1e-1)
//     {
//         float d = (upper + lower)*0.5f;
//         part = dual(jobs/d, eps);
//         if(part > nprocs)
//             lower = d;
//         else
//             upper = d;
//     }
//     part = dual(jobs/upper, eps);
//     return part;
// }

void eps_histogram(config &histogram, binlist &job_hist, float &h, const_job_range &jobs, float eps)
{
    int nbins = static_cast<int>(std::ceil(1.0f/(eps*eps)));
    h = (1.0f - eps)/nbins;
    float inv_h = 1.0f/h;

    histogram.resize(nbins);
    job_hist.resize(nbins);

    std::fill(histogram.begin(), histogram.end(), 0);

    BOOST_FOREACH(const float &job, jobs)
    {
        int slot = static_cast<int>(std::ceil( (job - eps)*inv_h)) - 1;

        ++histogram[slot];
        job_hist[slot].push_back(job);
    }
}

const_job_iter partition(const joblist &jobs, float split)
{
    return std::upper_bound(jobs.begin(), jobs.end(), split);
}

void pack_small(binlist &bins, const_job_range &small_range)
{
    binlist::iterator unfull_bin_start = bins.begin();
    binlist::iterator examine_start    = bins.begin();

    while(examine_start != bins.end())
    {
        float sum = 0.0f;
        BOOST_FOREACH(const float &j, *examine_start)
        {
            sum += j;
            if(sum > 1.0f)
                break;
        }
        if(sum > 1.0f)
        {
            ++examine_start;
            ++unfull_bin_start;
        }
        else
            break;
    }

    for(; examine_start != bins.end(); ++examine_start)
    {
        float sum = 0.0f;
        BOOST_FOREACH(const float &j, *examine_start)
        {
            sum += j;
            if(sum > 1.0f)
                break;
        }
        if(sum > 1.0f)
        {
            std::swap(*examine_start, *unfull_bin_start);
            ++unfull_bin_start;
        }
    }

    BOOST_FOREACH(const float &j, small_range)
    {
        if(unfull_bin_start == bins.end())
        {
            bins.push_back(joblist());
            bins.back().push_back(j);
        }
        else
            unfull_bin_start->push_back(j);

        float sum = 0.0f;
        BOOST_FOREACH(const float &j, *unfull_bin_start)
        {
            sum += j;
            if(sum > 1.0f)
                break;
        }
        if(sum > 1.0f)
            ++unfull_bin_start;
    }
}

struct feasible_iterator
{
    feasible_iterator(const config &maxconf_,
                      float base,
                      float h) :
        maxconf_(maxconf_),
        base_(base), h_(h), left_(1.0f)
    {
    }

    void reset()
    {
        left_ = 1.0f;
    }

    bool next(config &resconf)
    {
        int position = maxconf_.size()-1;
        ++resconf[position];
        left_ -= base_ + position*h_;
        while(resconf[position] > maxconf_[position] || left_ < 0.0f)
        {
            left_ += resconf[position]*(base_ + position*h_);
            resconf[position] = 0;
            --position;
            if(position < 0)
                return false;
            ++resconf[position];
            left_ -= base_ + position*h_;
        }
        return true;
    }

    const config &maxconf_;
    float base_;
    float h_;
    float left_;
};

struct bin_search
{
    bin_search()
        : bins(-1), parent(0)
    {}
    bin_search(int b)
        : bins(b), parent(0)
    {}
    bin_search(int b, bin_search *p)
        : bins(b), parent(p)
    {}

    int bins;
    bin_search *parent;
};

inline size_t index(const config &current, const config &dim)
{
    size_t sz = current.size();
    size_t res = 0;
    for(int i = 0; i < sz-1; ++i)
    {
        res = (res + current[i])*(dim[i+1]+1);
    }
    res += current[sz-1];

    return res;
}

inline void unindex(config &res, size_t idx, const config &cum_dims)
{
    size_t sz = res.size();

    for(int i = 0; i < sz-1; ++i)
    {
        res[i] = idx/cum_dims[i];
        idx -= res[i]*cum_dims[i];
    }

    res[sz-1] = idx;
}

std::vector<config> minbins(const config &conf, float eps, float h)
{
    size_t prod = 1;
    BOOST_FOREACH(const int &i, conf)
    {
        prod *= (i+1);
    }

    bin_search *search = new bin_search[prod];

    std::vector<config> todo;
    feasible_iterator fi(conf, eps, h);
    {
        config res(conf.size());
        std::fill(res.begin(), res.end(), 0);
        while(fi.next(res))
            todo.push_back(res);
    }

    BOOST_FOREACH(const config &c, todo)
    {
        search[index(c, conf)] = bin_search(1);
    }

    while(!todo.empty())
    {
        config t0    (todo.back());
        todo.pop_back();
        size_t t0_idx(index(t0, conf));

        config tf(conf);
        for(int i = 0; i < tf.size(); ++i)
            tf[i] -= t0[i];

        feasible_iterator fi(tf, eps, h);
        {
            config res(conf.size());
            std::fill(res.begin(), res.end(), 0);
            while(fi.next(res))
            {
                config res_total(res);
                for(int i = 0; i < conf.size(); ++i)
                    res_total[i] += t0[i];

                size_t t1_idx = index(res_total, conf);
                if( search[t1_idx].bins == -1 ||
                    search[t0_idx].bins + 1 < search[t1_idx].bins)
                {
                    search[t1_idx] = bin_search(1 + search[t0_idx].bins, search + t0_idx);
                    todo.push_back(res_total);
                }
            }
        }
    }

    config cum_dim(conf.size()-1);
    cum_dim.back() = conf[conf.size()-1]+1;
    for(int i = cum_dim.size()-2; i >= 0; --i)
        cum_dim[i] = cum_dim[i+1]*(conf[i+1]+1);

    std::vector<config> result;
    bin_search *last = search + index(conf, conf);
    config last_conf(conf);

    bin_search *current = last->parent;
    config current_conf(conf.size());
    unindex(current_conf, current - search, cum_dim);

    while(current)
    {
        result.push_back(config(conf.size()));
        for(int i = 0; i < conf.size(); ++i)
            result.back()[i] = last_conf[i] - current_conf[i];
        last = current;
        std::swap(last_conf, current_conf);
        current = last->parent;
        if(current)
            unindex(current_conf, current - search, cum_dim);
    }
    result.push_back(last_conf);

    delete search;

    return result;
}

binlist dual(const std::vector<float> &jobs, float eps)
{
    const_job_iter part(partition(jobs, eps));


    const_job_range small = boost::make_iterator_range(static_cast<const_job_iter>(jobs.begin()), part);
    const_job_range large = boost::make_iterator_range(part, static_cast<const_job_iter>(jobs.end()));

    binlist real_bins;
    if(!large.empty())
    {
        config histogram;
        binlist job_hist;
        float h;
        eps_histogram(histogram, job_hist, h,
                      large, eps);

        std::vector<config> hist_bins = minbins(histogram, eps, h);
        BOOST_FOREACH(const config &hb, hist_bins)
        {
            real_bins.push_back(joblist());
            for(int pos = 0; pos < hb.size(); ++pos)
            {
                for(int ct = 0; ct < hb[pos]; ++ct)
                {
                    real_bins.back().push_back(job_hist[pos].back());
                    job_hist[pos].back();
                }
            }
        }
    }

    pack_small(real_bins, small);

    return real_bins;
}

int main(int argc, char *argv[])
{
    float epsilon = 0.5f;
    joblist values;
    values += .14,.44,.93,.23,.56, 0.1, 0.1, 0.1, 0.1;
    std::sort(values.begin(), values.end());

    BOOST_FOREACH(const float &j, values)
    {
        std::cout << j << " ";
    }
    std::cout << std::endl;

    const_job_iter split = partition(values, epsilon);
    const_job_range large = boost::make_iterator_range(split, static_cast<const_job_iter>(values.end()));
    const_job_range small = boost::make_iterator_range(static_cast<const_job_iter>(values.begin()), split);
    std::cout << "Large: ";
    BOOST_FOREACH(const float &j, large)
    {
        std::cout << j << " ";
    }
    std::cout << std::endl;

    config  hist;
    binlist job_hist;
    float   h;
    eps_histogram(hist, job_hist, h, large, epsilon);

    std::cout << "Histogram: ";
    BOOST_FOREACH(const int &j, hist)
    {
        std::cout << j << " ";
    }
    std::cout << std::endl;

    std::cout << "job_hist: ";
    BOOST_FOREACH(const joblist &jl, job_hist)
    {
        std::cout << "[ ";
        BOOST_FOREACH(const float &j, jl)
        {
            std::cout << j << " ";
        }
        std::cout << "]";
    }
    std::cout << std::endl;

    pack_small(job_hist, small);

    std::cout << "job_hist: ";
    BOOST_FOREACH(const joblist &jl, job_hist)
    {
        std::cout << "[ ";
        BOOST_FOREACH(const float &j, jl)
        {
            std::cout << j << " ";
        }
        std::cout << "]";
    }
    std::cout << std::endl;

    config maxconf;
    maxconf += 2, 3, 5;

    std::vector<config> res = minbins(maxconf, 0.2, 0.2);
    BOOST_FOREACH(const config &cf, res)
    {
        std::cout << "[ ";
        BOOST_FOREACH(const float &c, cf)
        {
            std::cout << c << " ";
        }
        std::cout << "]";
    }
    std::cout << std::endl;

    return 0;
}
