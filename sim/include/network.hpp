#ifndef _NETWORK_HPP_
#define _NETWORK_HPP_

#include <vector>
#include <map>
#include <cassert>
#include <cmath>
#include <cfloat>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <algorithm>
#include <ctype.h>
#include "boost/foreach.hpp"

#define foreach BOOST_FOREACH

#define LANE_WIDTH 2.5f
#define CAR_LENGTH 4.5f
//* This is the position of the car's axle from the FRONT bumper of the car
#define CAR_REAR_AXLE 3.5f

typedef enum {DATA, CELLS, MERGES} draw_type;
#define H (2.1f*CAR_LENGTH)

const char* hwm_version_string();

struct carticle
{
    carticle() {}
    carticle(float ix, float iu) : x(ix), theta(0.0f), u(iu), y(0.0f), lc_state(0.0) {}

    int id;
    float x; //< Parametric position of carticle's front bumper axle along current lane.
    float theta; //< Orientation of carticle w.r.t. lane

    float u; //< Velocity of carticle.
    float y; //< Lane change position.

    int lc_state;
};

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
struct road_membership;
struct intersection;

typedef id<road> road_id;
typedef id<lane> lane_id;
typedef id<intersection> intersection_id;

#include "xml-util.hpp"
#include "car-models.hpp"
#include "timer.hpp"
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

    void fill_from_carticles();

    float sim_step();

    void calc_bounding_box();

    int add_carticle(int lane, float pos, float u);

    void dump_carticles(FILE *fp) const;

    float gamma_c;
    float inv_gamma;
    float min_h;

    float global_time;

    int carticle_ids;

    float bb[4]; //< Minx, maxx, miny, maxy

    char * name;
    std::vector<road> roads;
    std::vector<lane> lanes;
    std::vector<intersection> intersections;
};
#endif
