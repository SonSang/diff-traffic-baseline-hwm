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

    //! Find the segment containing x.
    /*
      \param x A float in [0.0, 1.0].
      \returns The segment containing x
    */
    int find_segment(float x) const;

    //! Builds lengths, normals, and mitres from points
    /*!
      This assumes that points has at least 2 elements in it;
      we overwrite any existing quantities with new calculations.
    */
    void calc_rep();

    //! Create a string representation of line
    /*!
      Fills buff with the first len characters of rep
      \param buff An array of at least len characters.
      \param len  The max. number of characters to write in buff (including trailing \0).
      \returns The number of written characters (not inluding trailing \0).
    */
    int to_string(char buff[], int len) const;

    float lane_width; //!< Width of lanes.

    std::vector<point> points;  //!< Points defining a polyline.

    std::vector<float> lengths; //!< Cumulative lengths of segments.
    std::vector<point> normals; //!< (#points-1) normalized vectors describing segment direction
    std::vector<float> mitres;  //!< (#points-2) quantities describing mitre joints at interior points
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
