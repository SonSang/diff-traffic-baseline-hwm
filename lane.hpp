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

struct adjacency;

typedef intervals<adjacency> adjacency_intervals;

//! Structure lane adjacency
struct adjacency
{
    bool xml_read(xmlTextReaderPtr reader);

    static const int NO_ADJACENCY = 0;
    lane_id neighbor; //< The right incident lane.

    float neighbor_interval[2]; //< The parametric interval
                               //< in the neighbor corresponding
                               //< to this adjacency
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

    lane* left_adjacency(float &t) const;
    lane* right_adjacency(float &t) const;

    void get_matrix(const float &x, float mat[16]) const;
    void get_point_and_normal(const float &x, point &pt, point &no) const;
    void get_point(const float &x, point &pt) const;

    void draw_data(float gamma_c) const;
    void draw_carticles() const;

    float collect_riemann(float gamma_c, float inv_gamma);

    void update(float maxspeed);

    void advance_carticles(float dt, float gamma_c);

    void swap_carticles();

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

    std::vector<carticle> carticles[2];
};
#endif
