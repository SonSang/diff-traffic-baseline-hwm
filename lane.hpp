#ifndef _LANE_HPP_
#define _LANE_HPP_

#include "arz.hpp"
#include "road.hpp"
#include "intersection.hpp"
#include "intervals.hpp"

#include <vector>

//! A container for information about lane teminini and junctions.
struct lane_end
{
    intersection::id intersection; //< Reference to the intersection this lane is incident to.
};

//! Describes the lanes relationship to a parent road.
/*!
  A limited description of how a lane follows its parent road
  Most notiably, we have a constant lane_position with no facility for
  parametrically-varying offsets.
 */
struct road_membership
{
    road::id parent_road; //< Specifies parent road.
    float interval[2];    //< The interval of the parent road's parametrization this structure describes.
    float lane_position;  //< This lane's offset from the centerline of the road. See line_rep for one
                          //< interpretation of this.
};

typedef intervals<road_membership> road_intervals;

struct adjacency_pair;

#define NO_ADJACENCY 0
typedef intervals<adjacency_pair*> adjacency_intervals;

//! A single continuous road lane
/*!
  This structure is the basic computational domain of our network.
  It can belong to several roads, and owns an array of solutions.
*/
struct lane
{
    typedef int id;

    road_intervals road_memberships; //< Helps describe spatial configuration of lane.

    adjacency_intervals left;  //< Lane's left neighbors.
    adjacency_intervals right; //< Lane's right neighbors.

    lane_end start; //< 'Source' terminus information.
    lane_end end;   //< 'Sink' terminus information.

    float speedlimit; //< The speedlimit along the lane.
                      //< Constant for the moment, but conceivably could vary
                      //< with parametrization.

    float h;             //< The simulation grid cell spacing. Assumed constant for now.
    unsigned int ncells; //< The number of simulation grid cells.
                         //< ncells*h = length.
    q *data;             //< The simulation data.
};

//! A structure describing a single lane's adjacency info for one entry.
struct lane_adjacency
{
    lane::id neighbor; //< The incident lane.
    adjacency_intervals::entry_id interval_entry; //< Reference to interval in one of the
                                                  //< incident lane's adjacency_intervals.
};

//! Structure  describing an interval where two lanes are adjacent.
struct adjacency_pair
{
    lane_adjacency left;  //< Reference to left lane's adjacency information.
    lane_adjacency right; //< Reference to right lane's adjacency information.
};

#endif
