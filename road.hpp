#ifndef _ROAD_HPP_
#define _ROAD_HPP_

#include <vector>
#include <cassert>
#include <cmath>
#include <cstdio>

//! A simple 2d point.
struct point
{
    point()
    {}

    point(float inx, float iny)
        : x(inx), y(iny)
    {}

    float x;
    float y;
};

//! A representation of a road's shape.
struct line_rep
{
    void locate(point *pt, float x, float offset) const;

    void draw(float start, float stop, float offset) const;

    int find_segment(float x) const;

    void calc_lengths();

    int to_string(char buff[], int len) const;

    float lane_width; //!< Width of lanes.

    std::vector<float> lengths; //!< Cumulative lengths of segments.
    std::vector<point> points;  //!< Points defining a polyline.
};

//! Road data structure.
/*
  Groups bundles of lanes together, giving them a name and with
  line_rep, a shape
*/
struct road
{
    typedef int id;

    const char * name; //!< Road name

    line_rep rep; //!< A description of the road's shape

    std::vector<int> member_lanes; //<! Lanes in the road
};

#endif
