#ifndef _NETWORK_HPP_
#define _NETWORK_HPP_

#include <vector>
#include <map>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctype.h>
#include "boost/foreach.hpp"

#define foreach BOOST_FOREACH

struct ltstr
{
    inline bool operator()(char* s1, char* s2) const
    {
        return strcmp(s1, s2) < 0;
    }
};

template <typename T>
union id
{
    T    *dp;
    char *sp;

    inline bool retrieve_ptr(std::vector<T> &v, std::map<char*, int, ltstr> &m)
    {
        std::map<char*, int, ltstr>::iterator res = m.find(sp);
        if(res == m.end())
            return false;

        free(sp);
        dp = &(v[res->second]);
        return true;
    }
};

struct road;
struct lane;
struct intersection;

typedef id<road> road_id;
typedef id<lane> lane_id;
typedef id<intersection> intersection_id;

#include "xml-util.hpp"
#include "intervals.hpp"
#include "arz.hpp"
#include "road.hpp"
#include "lane.hpp"
#include "intersection.hpp"

struct network
{
    bool load_from_xml(const char * filename);
    bool xml_read(xmlTextReaderPtr reader);

    ~network();

    void prepare(float h);

    float sim_step();

    float gamma_c;
    float inv_gamma;
    float min_h;

    char * name;
    std::vector<road> roads;
    std::vector<lane> lanes;
    std::vector<intersection> intersections;
};
#endif
