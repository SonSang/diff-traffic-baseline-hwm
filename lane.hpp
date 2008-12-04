#ifndef _LANE_HPP_
#define _LANE_HPP_

#include "intersection.hpp"
#include <vector>

struct q
{
    float rho;
    float y;
};

struct road
{
    const char * name;
};

typedef int lane_id;
#define NO_LANE -1

typedef int entry_id;

struct lane_adjacency
{
    lane_id neighbor;
    entry_id interval_entry;
};

typedef int intersection_id;
#define TAPER    -2
#define DEAD_END -1

struct lane_end
{
    intersection_id intersection;
};

typedef intervals<lane_adjacency> adjacency_intervals;

struct lane
{
    adjacency_intervals left;
    adjacency_intervals right;

    lane_end start;
    lane_end end;

    float speedlimit;

    float h;
    unsigned int ncells;
    q *data;
};
#endif
