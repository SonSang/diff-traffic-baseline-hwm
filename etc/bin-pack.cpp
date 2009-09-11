#include <iostream>
#include <fstream>
#include <vector>
#include <ext/hash_map>
#include <deque>
#include <cmath>
#include <cassert>
#include <boost/lexical_cast.hpp>
#include <boost/foreach.hpp>
#include <boost/range/iterator_range.hpp>
#include <boost/assign/std/vector.hpp>
#include <boost/assign/std/vector.hpp>
#include <boost/utility.hpp>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/vector_proxy.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <boost/algorithm/string/trim.hpp>

namespace ublas = boost::numeric::ublas;

typedef std::vector<float>                    joblist;
typedef joblist::iterator                     job_iter;
typedef joblist::const_iterator               const_job_iter;
typedef boost::iterator_range<job_iter>       job_range;
typedef boost::iterator_range<const_job_iter> const_job_range;

typedef std::vector<joblist>                  binlist;

typedef ublas::vector<int>                    config;
typedef config::iterator                      config_iter;
typedef boost::iterator_range<config_iter>    config_range;

inline size_t index(const config &current, const config &dim)
{
    size_t sz = current.size();
    size_t res = 0;
    for(int i = 0; i < sz-1; ++i)
        res = (res + current[i])*(dim[i+1]+1);
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

struct todo_queue : public std::deque<config>
{
    typedef std::deque<config> base;

    todo_queue(const config &maxc) : maxconf(maxc)
    {}

    config pop_front()
    {
        config res(base::front());
        base::pop_front();
        membership.erase(index(res, maxconf));
        return res;
    }

    void push_back(const config &res)
    {
        size_t idx(index(res, maxconf));
        __gnu_cxx::hash_map<size_t,bool>::iterator loc(membership.find(idx));
        if(loc == membership.end())
        {
            base::push_back(res);
            membership.insert(std::make_pair(idx,true));
        }
    }

    __gnu_cxx::hash_map<size_t,bool> membership;
    const config &maxconf;
};

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

template <class C>
typename C::value_type sum(const C &cont)
{
    typedef typename C::value_type value_type;
    value_type s = 0;
    BOOST_FOREACH(const value_type &i, cont)
    {
        s += i;
    }
    return s;
}

template <class C>
inline bool sum_over(const C &cont, typename C::value_type limit)
{
    typedef typename C::value_type value_type;
    value_type s = 0;
    BOOST_FOREACH(const value_type &i, cont)
    {
        s += i;
        if(s > limit)
            break;
    }
    return (s > limit);
}

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

struct feasible_iterator
{
    feasible_iterator(const config &maxconf_,
                      float base,
                      float h) :
        maxconf_(maxconf_),
        base_(base), h_(h), left_(1.0f)
    {
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

std::vector<config> minbins(const config &conf, float eps, float h)
{
    struct bin_search
    {
        bin_search()
            : bins(-1), parent(0) {}
        bin_search(int b)
            : bins(b), parent(0) {}

        bin_search(int b, bin_search *p)
            : bins(b), parent(p) {}

        int bins;
        bin_search *parent;
    };

    int s = conf.size();

    size_t prod = 1;
    BOOST_FOREACH(const int &i, conf)
    {
        prod *= (i+1);
    }

    bin_search *search = new bin_search[prod];

    todo_queue todo(conf);
    feasible_iterator fi(conf, eps, h);
    {
        config res((ublas::scalar_vector<int>(s, 0)));
        while(fi.next(res))
            todo.push_back(res);
    }

    BOOST_FOREACH(const config &c, todo)
    {
        search[index(c, conf)] = bin_search(1);
    }

    while(!todo.empty())
    {
        config t0    (todo.pop_front());
        size_t t0_idx(index(t0, conf));

        config tf = conf - t0;

        feasible_iterator fi(tf, eps, h);
        {
            config res((ublas::scalar_vector<int>(s, 0)));
            while(fi.next(res))
            {
                config res_total = res + t0;

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

    config cum_dim(s-1);
    cum_dim[s-2] = conf[s-1]+1;
    for(int i = s-3; i >= 0; --i)
        cum_dim[i] = cum_dim[i+1]*(conf[i+1]+1);

    std::vector<config> result;
    bin_search *last = search + index(conf, conf);
    config last_conf(conf);

    bin_search *current = last->parent;
    config current_conf(s);
    if(current)
        unindex(current_conf, current - search, cum_dim);

    while(current)
    {
        result.push_back(last_conf - current_conf);
        last = current;
        std::swap(last_conf, current_conf);
        current = last->parent;
        if(current)
            unindex(current_conf, current - search, cum_dim);
    }
    result.push_back(last_conf);

    delete[] search;

    return result;
}

void pack_small(binlist &bins, const_job_range &small_range)
{
    binlist::iterator unfull_bin_start = bins.begin();
    binlist::iterator examine_start    = bins.begin();

    while(examine_start != bins.end())
        if(sum_over(*examine_start, 1.0f))
        {
            ++examine_start;
            ++unfull_bin_start;
        }
        else
            break;

    for(; examine_start != bins.end(); ++examine_start)
        if(sum_over(*examine_start, 1.0f))
        {
            std::swap(*examine_start, *unfull_bin_start);
            ++unfull_bin_start;
        }

    BOOST_FOREACH(const float &j, small_range)
    {
        if(unfull_bin_start == bins.end())
        {
            bins.push_back(joblist());
            bins.back().push_back(j);
            unfull_bin_start = boost::prior(bins.end());
        }
        else
            unfull_bin_start->push_back(j);

        if(sum_over(*unfull_bin_start, 1.0f))
            ++unfull_bin_start;
    }
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
        real_bins.reserve(hist_bins.size());
        BOOST_FOREACH(const config &hb, hist_bins)
        {
            real_bins.push_back(joblist());
            for(int pos = 0; pos < hb.size(); ++pos)
                for(int ct = 0; ct < hb[pos]; ++ct)
                {
                    real_bins.back().push_back(job_hist[pos].back());
                    job_hist[pos].pop_back();
                }
        }
    }

    pack_small(real_bins, small);

    return real_bins;
}

bool pack_small_in_nbins(binlist &bins, const_job_range &small_range)
{
    binlist::iterator unfull_bin_start = bins.begin();
    binlist::iterator examine_start    = bins.begin();

    while(examine_start != bins.end())
    {
        if(sum_over(*examine_start, 1.0f))
        {
            ++examine_start;
            ++unfull_bin_start;
        }
        else
            break;
    }

    for(; examine_start != bins.end(); ++examine_start)
        if(sum_over(*examine_start, 1.0f))
        {
            std::swap(*examine_start, *unfull_bin_start);
            ++unfull_bin_start;
        }

    BOOST_FOREACH(const float &j, small_range)
    {
        if(unfull_bin_start == bins.end())
            return false;
        else
            unfull_bin_start->push_back(j);

        if(sum_over(*unfull_bin_start, 1.0f))
            ++unfull_bin_start;
    }

    return true;
}

bool dual_in_nbins(const std::vector<float> &jobs, float eps, int nbins)
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
        if(hist_bins.size() > nbins)
            return false;
        real_bins.reserve(hist_bins.size());
        BOOST_FOREACH(const config &hb, hist_bins)
        {
            real_bins.push_back(joblist());
            for(int pos = 0; pos < hb.size(); ++pos)
                for(int ct = 0; ct < hb[pos]; ++ct)
                {
                    real_bins.back().push_back(job_hist[pos].back());
                    job_hist[pos].pop_back();
                }
        }
    }

    return pack_small_in_nbins(real_bins, small);
}

binlist eps_makespan(const joblist &jobs, int nprocs, float eps)
{
    float lower = size(jobs, nprocs);
    float upper = 2.0f*lower;
    while(std::abs(upper-lower) > 1e-1)
    {
        float d = (upper + lower)*0.5f;
        float inv_d = 1.0f/d;
        joblist scaled(jobs);
        BOOST_FOREACH(float &j, scaled)
        {
            j *= inv_d;
        }
        if(dual_in_nbins(scaled, eps, nprocs))
            lower = d;
        else
            upper = d;
    }
    float inv_upper = 1.0f/upper;
    joblist scaled(jobs);
    BOOST_FOREACH(float &j, scaled)
    {
        j *= inv_upper;
    }

    binlist part(dual(scaled, eps));
    BOOST_FOREACH(joblist &jl, part)
    {
        BOOST_FOREACH(float &j, jl)
        {
            j *= upper;
        }
    }
    return part;
}



int main(int argc, char *argv[])
{
    if(argc < 4)
    {
        std::cerr << "Usage: " << argv[0] << " <epilson> <nbins> <jobfile>|-" << std::endl;
        exit(1);
    }

    float epsilon = boost::lexical_cast<float>(argv[1]);
    if(epsilon <= 0.0f || epsilon > 1.0f)
    {
        std::cerr << "Epsilon must be in (0, 1]" << std::endl;
        exit(1);
    }

    int   nbins   = boost::lexical_cast<int>(argv[2]);
    if(nbins <= 0)
    {
        std::cerr << "nbins must be > 0" << std::endl;
        exit(1);
    }

    std::istream *input_stream;
    if(*argv[3] == '-')
        input_stream = &(std::cin);
    else
        input_stream = new std::ifstream(argv[3]);

    joblist values;

    while(!input_stream->eof())
    {
        std::string str;
        std::getline(*input_stream, str);
        boost::algorithm::trim_left(str);
        if(!str.empty())
            values.push_back(boost::lexical_cast<float>(str));
    }

    std::sort(values.begin(), values.end());

    binlist bl(eps_makespan(values, nbins, epsilon));

    BOOST_FOREACH(const joblist &jl, bl)
    {
        std::cout << "[ ";
        float sum = 0.0f;
        BOOST_FOREACH(const float &f, jl)
        {
            std::cout << f << " ";
            sum += f;
        }
        std::cout << "]: " << sum << " ";
    }
    std::cout << std::endl;

    return 0;
}
