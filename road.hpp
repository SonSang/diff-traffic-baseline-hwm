#ifndef _ROAD_HPP_
#define _ROAD_HPP_

#include "xml-util.hpp"
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

//! A four-sided simple polygon.
struct quad
{
    int v[4];
};

//! A representation of a road's shape.
struct line_rep
{
    //! Compute length of line displaced from polyline
    /*!
      \param offset An offset from the polyline along which to place the point.
                    The convention that the parameterization increases with x,
                    and the offset increaseses with y
    */
    float offset_length(float offset) const;

    //! Determine where a point along the parameterization is.
    /*!
      \param pt A pointer to a point where the results will be stored.
      \param x  A float in [0, 1] specifying the parameter to extract
      \param offset An offset from the polyline along which to place the point.
                    The convention that the parameterization increases with x,
                    and the offset increaseses with y
    */
    void locate(point *pt, float x, float offset) const;

    //! Extract a strip of quads.
    /*!
      \param range A pair of ordered (<) floats in [0, 1] representing the parameteric range of the road to extract.
      \param center_offset The offset from the center of the road along which to take the paramterization.
      \param offsets A pair of offsets defining the extents of the sweep in 'y'.
      This function isn't air-tight and can generate some bogus meshes, but it's a start
    */
    void lane_mesh(float range[2], float center_offset, float offsets[2]) const;

    void draw(float start, float stop, float offset) const;

    //! Find the segment containing x.
    /*!
      \param x A float in [0.0, 1.0].
      \param offset A float representing offset from centerline (+ is 'left')
      \returns The segment containing x
    */
    int find_segment(float x, float offset) const;

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

    bool xml_read(xmlTextReaderPtr reader);

    float lane_width; //!< Width of lanes.

    std::vector<point> points;  //!< Points defining a polyline.

    std::vector<float> clengths; //!< Cumulative lengths of segments. (#points)
    std::vector<point> normals; //!< (#points-1) normalized vectors describing segment direction.
    std::vector<float> cmitres;  //!< Cumulative quantities describing mitre joints at each point. (#points)
};

//! Road data structure.
/*!
  Groups bundles of lanes together, giving them a name and with
  line_rep, a shape
*/
struct road
{
    typedef int id;

    ~road();

    bool xml_read(xmlTextReaderPtr reader);

    char * name;             //!< Road name
    line_rep rep;                  //!< A description of the road's shape
    std::vector<int> member_lanes; //<! Lanes in the road
};

#endif
