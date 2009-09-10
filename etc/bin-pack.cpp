#include <iostream>
#include <vector>
#include <cmath>
#include <cassert>
#include <boost/foreach.hpp>
#include <boost/range/iterator_range.hpp>
#include <boost/assign/std/vector.hpp>

typedef std::vector<float>                    joblist;
typedef std::vector<float>::iterator          job_iter;
typedef std::vector<float>::const_iterator    const_job_iter;
typedef boost::iterator_range<job_iter>       job_range;
typedef boost::iterator_range<const_job_iter> const_job_range;

typedef std::vector<joblist>               binlist;

typedef std::vector<int>                   config;

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
    std::vector<joblist>::iterator unfull_bin_start = bins.begin();
    std::vector<joblist>::iterator examine_start    = bins.begin();

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

void minbins(const config &conf, float eps, float h)
{
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

    return 0;
}
