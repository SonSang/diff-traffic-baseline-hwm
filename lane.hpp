#ifndef _LANE_HPP_
#define _LANE_HPP_

//! Information about a lane's outgoing BC
struct lane_end
{
    typedef enum {TAPER, DEAD_END, INTERSECTION} type; //< Types of ends.

    type end_type;        //< The type of this BC.
    intersection_id inters; //< Reference to the intersection this lane is incident to.
    int intersect_in_ref; //< Reference to this lane's local id in inters.
};

//! Describes the lanes relationship to a parent road.
/*!
  A limited description of how a lane follows its parent road
  Most notiably, we have a constant lane_position with no facility for
  parametrically-varying offsets.
 */
struct road_membership
{
    bool xml_read(xmlTextReaderPtr reader);

    road_id parent_road; //< Specifies parent road.
    float interval[2];    //< The interval of the parent road's parametrization this structure describes.
    float lane_position;  //< This lane's offset from the centerline of the road. See line_rep for one
                          //< interpretation of this.
};

typedef intervals<road_membership> road_intervals;

struct adjacency_pair;

#define NO_ADJACENCY 0
typedef intervals<adjacency_pair*> adjacency_intervals;

//! Structure  describing an interval where two lanes are adjacent.
struct adjacency_pair
{
    lane_id left_neighbor; //< The left incident lane.
    lane_id right_neighbor; //< The right incident lane.

    adjacency_intervals::entry_id left_interval; //< Reference to interval in the
                                                 //< left lane's adjacency_intervals.
    adjacency_intervals::entry_id right_interval; //< Reference to interval in the
                                                  //< right lane's adjacency_intervals.
};

//! A single continuous road lane
/*!
  This structure is the basic computational domain of our network.
  It can belong to several roads, and owns an array of solutions.
*/
struct lane
{
    bool xml_read(xmlTextReaderPtr reader);

    float calc_length() const;

    void draw_data(float gamma_c) const;

    float collect_riemann(float gamma_c, float inv_gamma);

    void update(float maxspeed);

    road_intervals road_memberships; //< Helps describe spatial configuration of lane.

    adjacency_intervals left;  //< Lane's left neighbors.
    adjacency_intervals right; //< Lane's right neighbors.

    lane_end start; //< 'Source' terminus information.
    lane_end   end;   //< 'Sink' terminus information.

    float speedlimit; //< The speedlimit along the lane.
                      //< Constant for the moment, but conceivably could vary
                      //< with parametrization.

    float h;             //< The simulation grid cell spacing. Assumed constant for now.
    unsigned int ncells; //< The number of simulation grid cells.
                         //< ncells*h = length.
    q *data;             //< The simulation data.
    riemann_solution *rs;//< Saved Riemann solutions for this lane.
};
#endif
