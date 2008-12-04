#ifndef _LANE_HPP_
#define _LANE_HPP_

#include "arz.hpp"
#include "road.hpp"
#include "intersection.hpp"
#include "intervals.hpp"

#include <vector>

struct lane_end
{
    intersection::id intersection;
};

struct road_membership
{
    road::id parent_road;
    float interval[2];
    float lane_position;
};

typedef intervals<road_membership> road_intervals;

struct adjacency_pair;

#define NO_ADJACENCY 0
typedef intervals<adjacency_pair*> adjacency_intervals;

struct lane
{
    typedef int id;

    road_intervals road_memberships;

    adjacency_intervals left;
    adjacency_intervals right;

    lane_end start;
    lane_end end;

    float speedlimit;

    float h;
    unsigned int ncells;
    q *data;
};

struct lane_adjacency
{
    lane::id neighbor;
    adjacency_intervals::entry_id interval_entry;
};

struct adjacency_pair
{
    lane_adjacency left;
    lane_adjacency right;
};

#endif
