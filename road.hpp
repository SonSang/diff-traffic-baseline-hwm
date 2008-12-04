#ifndef _ROAD_HPP_
#define _ROAD_HPP_

#include <vector>

struct point
{
    float x;
    float y;
};

struct line_rep
{
    void locate(point *pt, float x, float offset) const;

    void draw(float start, float stop, float offset) const;

    void populate_lengths();

    float lane_width;

    std::vector<float> lengths;
    std::vector<point> points;
};

struct road
{
    typedef int id;

    const char * name;

    line_rep rep;

    std::vector<int> member_lanes;
};

#endif
