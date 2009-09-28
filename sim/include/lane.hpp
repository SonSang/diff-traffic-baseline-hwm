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

struct source_sink
{
    source_sink() {}
    source_sink(float p, int c) : pos(p), capacity(c) {}

    bool full() const
    {
        return capacity == 0;
    }

    void reserve_space()
    {
        --capacity;
    }

    void add_carticle(carticle &c)
    {
        carticles.push_back(c);
    }

    float pos;
    int capacity;
    std::vector<carticle> carticles;
};

//! Structure lane adjacency
struct adjacency
{
    bool xml_read(xmlTextReaderPtr reader);

    static const int NO_ADJACENCY = 0;
    lane_id neighbor; //< The right incident lane.

    float neighbor_interval[2]; //< The parametric interval
                               //< in the neighbor corresponding
                               //< to this adjacency

    int find_source(float x) const;

    std::vector<source_sink> source_sinks;
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
    void  auto_scale_memberships();
    void scale_offsets(float f);

    lane* left_adjacency(float &t) const;
    lane* right_adjacency(float &t) const;

    lane* upstream_lane() const;
    lane* downstream_lane() const;

    void get_matrix(const float &x, float mat[16]) const;
    int  get_point_and_normal(const float &x, point &pt, point &no) const;
    void get_point(const float &x, point &pt) const;

    float velocity(float x, float gamma_c) const;

    void draw_data(draw_type dtype, float gamma_c) const;
    void draw_carticles() const;
    void draw_source_sinks(float scale) const;

    void fill_from_carticles();
    void reset_data();
    void fill_y(float gamma_c);

    float collect_riemann(float gamma_c, float inv_gamma);

    void update(float maxspeed);

    int merge_intent(float t, float gamma_c) const;
    bool merge_possible(carticle &c, int intent, float gamma_c) const;

    void find_ss(float x);

    void advance_carticles(float dt, float gamma_c);

    void swap_carticles();

    void clear_merges();
    void apply_merges(float dt, float gamma_c);

    void dump_carticles(FILE *fp, const float origin[2]) const;

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
    merge_state *merge_states;

    std::vector<carticle> carticles[2];
};

struct ss_stack
{
    ss_stack(float x, intervals<adjacency> *ia) : int_adj(ia)
    {
        const float t = x;
        adjacency_no = int_adj->find(t);

        if(adjacency_no == -1)
            ss_idx = (*int_adj).base_data.find_source(t);
        else
            ss_idx = (*int_adj).entries[adjacency_no].data.find_source(t);
    }

    source_sink* front()
    {
        if(ss_idx == -1)
            return 0;

        if(adjacency_no == -1)
            return (&(*int_adj).base_data.source_sinks[ss_idx]);
        else
            return (&(*int_adj).entries[adjacency_no].data.source_sinks[ss_idx]);
    }

    float front_value()
    {
        const source_sink *s = front();

        return s ? s->pos : FLT_MAX;
    }

    void advance()
    {
        ++ss_idx;

        int sslen;
        if(adjacency_no == -1)
            sslen = static_cast<int>((*int_adj).base_data.source_sinks.size());
        else
            sslen = static_cast<int>((*int_adj).entries[adjacency_no].data.source_sinks.size());

        while(ss_idx >= sslen)
        {
            ++adjacency_no;
            if(adjacency_no >= static_cast<int>(int_adj->entries.size()))
            {
                ss_idx = -1;
                break;
            }

            sslen = static_cast<int>((*int_adj).entries[adjacency_no].data.source_sinks.size());
        }
    }

    bool end() const
    {
        return ss_idx == -1;
    }

    intervals<adjacency> *int_adj;
    int adjacency_no;
    int ss_idx;
};

struct ss_walker
{
    ss_walker(lane &in_la) : la(in_la),
        left_s(0.0f, &(la.left)),
        right_s(0.0f, &(la.right))
    {
        new_pick();
    }

    source_sink* front()
    {
        if(end())
            return 0;
        else
            return pick->front();
    }

    void advance()
    {
        if(!end())
        {
            pick->advance();
            new_pick();
        }
    }

    void advance(float x)
    {
        while(!end() && pick->front()->pos < x)
        {
            pick->advance();
            new_pick();
        }
    }

    void new_pick()
    {
        if(left_s.end() && right_s.end())
            pick = 0;
        else if(left_s.front_value() < right_s.front_value())
        {
            pick = &left_s;
            side = -1;
        }
        else
        {
            pick = &right_s;
            side = 1;
        }
    }

    bool end() const
    {
        return !pick;
    }

    lane &la;

    ss_stack left_s;
    ss_stack right_s;

    ss_stack *pick;
    int side;
};

struct const_ss_stack
{
    const_ss_stack(float x, const intervals<adjacency> *ia) : int_adj(ia)
    {
        const float t = x;
        adjacency_no = int_adj->find(t);

        if(adjacency_no == -1)
            ss_idx = (*int_adj).base_data.find_source(t);
        else
            ss_idx = (*int_adj).entries[adjacency_no].data.find_source(t);
    }

    const source_sink* front() const
    {
        if(ss_idx == -1)
            return 0;

        if(adjacency_no == -1)
            return (&(*int_adj).base_data.source_sinks[ss_idx]);
        else
            return (&(*int_adj).entries[adjacency_no].data.source_sinks[ss_idx]);
    }

    float front_value() const
    {
        const source_sink *s = front();

        return s ? s->pos : FLT_MAX;
    }

    void advance()
    {
        ++ss_idx;

        int sslen;
        if(adjacency_no == -1)
            sslen = static_cast<int>((*int_adj).base_data.source_sinks.size());
        else
            sslen = static_cast<int>((*int_adj).entries[adjacency_no].data.source_sinks.size());

        while(ss_idx >= sslen)
        {
            ++adjacency_no;
            if(adjacency_no >= static_cast<int>(int_adj->entries.size()))
            {
                ss_idx = -1;
                break;
            }

            sslen = static_cast<int>((*int_adj).entries[adjacency_no].data.source_sinks.size());
        }
    }

    bool end() const
    {
        return ss_idx == -1;
    }

    const intervals<adjacency> *int_adj;
    int adjacency_no;
    int ss_idx;
};

struct const_ss_walker
{
    const_ss_walker(const lane &in_la) : la(in_la),
                                         left_s(0.0f, &(la.left)),
                                         right_s(0.0f, &(la.right))
    {
        new_pick();
    }

    const source_sink* front() const
    {
        if(end())
            return 0;
        else
            return pick->front();
    }

    void advance()
    {
        if(!end())
        {
            pick->advance();
            new_pick();
        }
    }

    void advance(float x)
    {
        while(!end() && pick->front()->pos < x)
        {
            pick->advance();
            new_pick();
        }
    }

    void new_pick()
    {
        if(left_s.end() && right_s.end())
            pick = 0;
        else if(left_s.front_value() < right_s.front_value())
        {
            pick = &left_s;
            side = -1;
        }
        else
        {
            pick = &right_s;
            side = 1;
        }
    }

    bool end() const
    {
        return !pick;
    }

    const lane &la;

    const_ss_stack left_s;
    const_ss_stack right_s;

    const_ss_stack *pick;
    int side;
};
#endif
