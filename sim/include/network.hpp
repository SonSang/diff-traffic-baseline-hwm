#ifndef _NETWORK_HPP_
#define _NETWORK_HPP_

#include <vector>
#include <deque>
#include <map>
#include <cassert>
#include <cmath>
#include <cfloat>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <algorithm>
#include <ctype.h>
#include <boost/foreach.hpp>
#include <boost/random/linear_congruential.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/variate_generator.hpp>

#define foreach BOOST_FOREACH

#define LANE_WIDTH 3.62f
#define CAR_LENGTH 4.5f
//* This is the position of the car's axle from the FRONT bumper of the car
#define CAR_REAR_AXLE 3.5f

typedef enum {DATA, CELLS, MERGES} draw_type;
#define H (12.1f*CAR_LENGTH)

const char* hwm_version_string();

struct source_sink;
struct lane;

struct carticle
{
    carticle() {}
    carticle(float ix, float iu) : x(ix), theta(0.0f), u(iu), y(0.0f), motion_state(0), lastlane(0) {}

    int id;
    float x; //< Parametric position of carticle's rear axle along current lane.
    float theta; //< Orientation of carticle w.r.t. lane

    float u; //< Velocity of carticle.
    float y; //< Lane change/source-sink position.

    inline bool free_motion() const { return motion_state == 0; }

    inline bool in_lane_change() const { return std::abs(motion_state) == 1; }
    inline int lane_change_dir() const
    {
        assert(in_lane_change());
        return motion_state;
    }

    inline void start_lane_change(int dir)
    {
        assert(free_motion());
        assert(std::abs(dir) == 1);
        motion_state = dir;
    }

    inline void end_lane_change()
    {
        assert(in_lane_change());
        motion_state = 0;
    }

    inline bool in_turn() const { return std::abs(motion_state) == 2; }
    inline int turn_dir() const
    {
        assert(in_turn());
        return (motion_state < 0) ? -1 : 1;
    }

    inline void start_turn(int dir, source_sink *target)
    {
        assert(free_motion());
        assert(std::abs(dir) == 1);
        motion_state = (dir < 0) ? -2 : 2;
        turn_target = target;
    }

    inline void end_turn()
    {
        assert(in_turn());
        motion_state = 0;
        turn_target = 0;
    }

    int motion_state; //< Lane-change/source-sink state - -1 is left change, 1 right, 0 none, -2 is left turn,  2 right turn
    const lane *lastlane;
    source_sink *turn_target;
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
#include "car-stream.hpp"
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

    void translate(const point &tr);

    int add_carticle(int lane, float pos, float u);

    void dump_carticles(FILE *fp) const;

    float gamma_c;
    float inv_gamma;
    float min_h;

    float global_time;

    int carticle_ids;

    float bb[4]; //< Minx, maxx, miny, maxy

    typedef boost::minstd_rand base_generator_type;
    base_generator_type *generator;
    boost::uniform_real<> *uni_dist;

    char * name;
    std::vector<road> roads;
    std::vector<lane> lanes;
    std::vector<intersection> intersections;
};
#endif
