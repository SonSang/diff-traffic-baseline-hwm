#include "network.hpp"


static inline bool lt(float l, float r)
{
    return (l - r) < -1e-6;
}

static inline bool eq(float l, float r)
{
    return std::abs(r - l) < 1e-6;
}

struct lexicographic
{
    inline bool operator()(const point &pt0, const point &pt1) const
    {
        return lt(pt0.x, pt1.x) || (eq(pt0.x, pt1.x) && lt(pt0.y, pt1.y));
    }
};

static inline bool rightturn(const point &pt0, const point &pt1, const point &pt2)
{
    float rtval = ((pt1.x-pt0.x)*(pt2.y-pt1.y) - (pt2.x-pt1.x)*(pt1.y-pt0.y));
    return rtval < -1e-6;
}

static inline void convex_hull(std::vector<point> & pts)
{
    lexicographic lx;
    std::sort(pts.begin(), pts.end(), lx);
    std::vector<point> newpts;
    newpts.push_back(pts[0]);
    int count = 1;
    while(count < static_cast<int>(pts.size()))
    {
        const point &bpt = newpts.back();
        if(!(eq(bpt.x, pts[count].x) && eq(bpt.y, pts[count].y)))
            newpts.push_back(pts[count]);
        ++count;
    }
    pts.clear();

    pts.push_back(newpts[0]);
    pts.push_back(newpts[1]);

    for(count = 2; count < static_cast<int>(newpts.size()); ++count)
    {
        pts.push_back(newpts[count]);

        int back = static_cast<int>(pts.size())-1;
        while(back > 1 && !rightturn(pts[back-2], pts[back-1], pts[back]))
        {
            std::swap(pts[back], pts[back-1]);
            pts.pop_back();
            --back;
        }
    }

    int upper_back = static_cast<int>(pts.size());
    for(count = static_cast<int>(newpts.size())-2; count >= 0; --count)
    {
        pts.push_back(newpts[count]);

        int back = static_cast<int>(pts.size())-1;
        while(back > upper_back && !rightturn(pts[back-2], pts[back-1], pts[back]))
        {
            std::swap(pts[back], pts[back-1]);
            pts.pop_back();
            --back;
        }
    }
    pts.pop_back();
}

static bool read_lane_ref(void *item, xmlTextReaderPtr reader)
{
    std::vector<lane_id> *lanev = reinterpret_cast<std::vector<lane_id>*>(item);

    char *ref = 0;
    int local_id;

    boost::fusion::vector<list_matcher<char*>,
        list_matcher<int> > vl(lm("ref", &ref),
                               lm("local_id", &local_id));

    if(!read_attributes(vl, reader))
       return false;

    if(local_id >= static_cast<int>(lanev->size()))
        lanev->resize(local_id+1);

    (*lanev)[local_id].sp = ref;

    return true;
}

static bool read_incoming(void *item, xmlTextReaderPtr reader)
{
    intersection *is = reinterpret_cast<intersection*>(item);
    xml_elt read[] =
        {{0,
          BAD_CAST "lane_ref",
          &(is->incoming),
          read_lane_ref}};

    if(!read_elements(reader, sizeof(read)/sizeof(read[0]), read, BAD_CAST "incoming"))
        return false;

    return true;;
}

static bool read_outgoing(void *item, xmlTextReaderPtr reader)
{
    intersection *is = reinterpret_cast<intersection*>(item);
    xml_elt read[] =
        {{0,
          BAD_CAST "lane_ref",
          &(is->outgoing),
          read_lane_ref}};

    if(!read_elements(reader, sizeof(read)/sizeof(read[0]), read, BAD_CAST "outgoing"))
        return false;

    return true;
}

static bool read_incident(void *item, xmlTextReaderPtr reader)
{
    intersection *is = reinterpret_cast<intersection*>(item);

    xml_elt read[] =
        {{0,
          BAD_CAST "incoming",
          item,
          read_incoming},
         {0,
          BAD_CAST "outgoing",
          item,
          read_outgoing}};

    bool status = read_elements(reader, sizeof(read)/sizeof(read[0]), read, BAD_CAST "incident");
    if(!status)
        return false;

    return read[0].count == 1 && read[1].count == 1 && (is->incoming.size() + is->outgoing.size()) > 0;
}

static bool read_state(void *item, xmlTextReaderPtr reader)
{
    intersection *is = reinterpret_cast<intersection*>(item);

    is->states.push_back(intersection::state());

    intersection::state & new_state = is->states.back();

    new_state.in_states.resize(is->incoming.size());
    foreach(intersection::state::out_id &oid, new_state.in_states)
    {
        oid.out_ref = intersection::state::STOP;
        oid.fict_lane = 0;
    }

    new_state.out_states.resize(is->outgoing.size());
    foreach(intersection::state::in_id &iid, new_state.out_states)
    {
        iid.in_ref = intersection::state::STARVATION;
        iid.fict_lane = 0;
    }

    return new_state.xml_read(reader);
}

static bool read_states(void *item, xmlTextReaderPtr reader)
{
    intersection *is = reinterpret_cast<intersection*>(item);

    if((is->incoming.size() + is->outgoing.size()) == 0)
        return false;

    xml_elt read[] =
        {{0,
          BAD_CAST "state",
          item,
          read_state}};

    if(!read_elements(reader, sizeof(read)/sizeof(read[0]), read, BAD_CAST "states"))
        return false;

    foreach(const intersection::state &st, is->states)
        assert(st.isvalid(is->incoming.size(), is->outgoing.size()));

    return read[0].count > 0;
}


static bool read_lane_pair(void *item, xmlTextReaderPtr reader)
{
    intersection::state *s = reinterpret_cast<intersection::state*>(item);

    int in, out;
    boost::fusion::vector<list_matcher<int>,
        list_matcher<int> > vl(lm("in_id",  &in),
                               lm("out_id", &out));

    if(!read_attributes(vl, reader))
       return false;

    s->in_states[in].out_ref = out;
    s->out_states[out].in_ref = in;

    return true;
}

bool intersection::state::xml_read(xmlTextReaderPtr reader)
{
    xml_elt read[] =
        {{0,
          BAD_CAST "lane_pair",
          this,
          read_lane_pair}};

    boost::fusion::vector<list_matcher<int>,
        list_matcher<float> > vl(lm("id",       &id),
                                 lm("duration", &duration));

    if(!read_attributes(vl, reader))
       return false;

    if(!read_elements(reader, sizeof(read)/sizeof(read[0]), read, BAD_CAST "state"))
        return false;

    return true;
}

bool intersection::state::isvalid(int nincoming, int noutgoing) const
{
    foreach(const out_id &oid, in_states)
        if((oid.out_ref != STOP && oid.out_ref < 0) || oid.out_ref >= noutgoing)
            return false;

    foreach(const in_id &iid, out_states)
        if((iid.in_ref != STARVATION && iid.in_ref < 0)  || iid.in_ref >= nincoming)
            return false;

    return true;
}

bool intersection::xml_read(xmlTextReaderPtr reader)
{
    xml_elt read[] =
        {{0,
          BAD_CAST "incident",
          this,
          read_incident},
         {0,
          BAD_CAST "states",
          this,
          read_states}};

    bool status = read_elements(reader, sizeof(read)/sizeof(read[0]), read, BAD_CAST "intersection");
    if(!status)
        return false;

    return read[0].count == 1 && read[1].count == 1;
}

int intersection::next_state()
{
    ++current_state;
    if(current_state >= static_cast<int>(states.size()))
        current_state = 0;

    return current_state;
}

lane* intersection::incoming_state(int intern_ref) const
{
    const state &cstate = states[current_state];

    if(intern_ref >= 0)
        return cstate.in_states[intern_ref].fict_lane;
    else
    {
        assert(-intern_ref-1 < static_cast<int>(outgoing.size()));
        return outgoing[-intern_ref-1].dp;
    }
}

lane* intersection::outgoing_state(int intern_ref) const
{
    const state &cstate = states[current_state];

    if(intern_ref >= 0)
        return cstate.out_states[intern_ref].fict_lane;
    else
    {
        assert(-intern_ref-1 < static_cast<int>(incoming.size()));
        return incoming[-intern_ref-1].dp;
    }
}

void intersection::build_shape(float lane_width)
{
    shape.resize(2*(outgoing.size()+incoming.size()));

    int i = 0;
    foreach(const lane_id &lid, incoming)
    {
        const road_membership *rom;
        if(lid.dp->road_memberships.entries.empty())
            rom = &(lid.dp->road_memberships.base_data);
        else
            rom = &(lid.dp->road_memberships.entries.back().data);

        rom->parent_road.dp->rep.locate(&(shape[2*i]), rom->interval[1], rom->lane_position - 0.5*lane_width);
        rom->parent_road.dp->rep.locate(&(shape[2*i+1]), rom->interval[1], rom->lane_position + 0.5*lane_width);
        ++i;
    }

    int incount = static_cast<int>(incoming.size());
    i = 0;
    foreach(const lane_id &lid, outgoing)
    {
        const road_membership *rom = &(lid.dp->road_memberships.base_data);

        rom->parent_road.dp->rep.locate(&(shape[2*(i + incount)]), rom->interval[0], rom->lane_position - 0.5*lane_width);
        rom->parent_road.dp->rep.locate(&(shape[2*(i + incount)+1]), rom->interval[0], rom->lane_position + 0.5*lane_width);
        ++i;
    }

    convex_hull(shape);

    center.x = center.y = 0.0f;

    foreach(const point &pt, shape)
    {
        center.x += pt.x;
        center.y += pt.y;
    }
    center.x /= shape.size();
    center.y /= shape.size();
}

float intersection::collect_riemann(float gamma_c, float inv_gamma)
{
    float maxspeed = FLT_MIN;
    const state &cstate = states[current_state];

    int i = 0;
    foreach(const state::out_id &oid, cstate.in_states)
    {
        lane *la = oid.fict_lane;
        if(la)
        {
            lane *start = incoming[i].dp;
            lane *end   = la;

            full_q q_l;
            q_l.from_q(start->data + start->ncells - 1,
                       start->speedlimit,
                       gamma_c);
            full_q q_r;
            q_r.from_q(end->data,
                       end->speedlimit,
                       gamma_c);

            lebacque_inhomogeneous_riemann(start->rs + start->ncells,
                                           &q_l,
                                           &q_r,
                                           start->speedlimit,
                                           end->speedlimit,
                                           gamma_c,
                                           inv_gamma);

            memcpy(end->rs,
                   start->rs + start->ncells,
                   sizeof(riemann_solution));

            assert(std::isfinite(end->rs->speeds[0]));
            assert(std::isfinite(end->rs->speeds[1]));

            maxspeed = std::max(maxspeed, std::max(std::abs(end->rs->speeds[0]), std::abs(end->rs->speeds[1])));
        }
        ++i;
    }

    i = 0;
    foreach(const state::in_id &iid, cstate.out_states)
    {
        lane *la = iid.fict_lane;
        if(la)
        {
            lane *start = la;
            lane *end   = outgoing[i].dp;

            full_q q_l;
            q_l.from_q(start->data + start->ncells - 1,
                       start->speedlimit,
                       gamma_c);
            full_q q_r;
            q_r.from_q(end->data,
                       end->speedlimit,
                       gamma_c);

            lebacque_inhomogeneous_riemann(start->rs + start->ncells,
                                           &q_l,
                                           &q_r,
                                           start->speedlimit,
                                           end->speedlimit,
                                           gamma_c,
                                           inv_gamma);

            memcpy(end->rs,
                   start->rs + start->ncells,
                   sizeof(riemann_solution));

            assert(std::isfinite(end->rs->speeds[0]));
            assert(std::isfinite(end->rs->speeds[1]));

            maxspeed = std::max(maxspeed, std::max(std::abs(end->rs->speeds[0]), std::abs(end->rs->speeds[1])));
        }
        ++i;
    }

    return maxspeed;
}

static const char fict_name[] = "Fictitious intersection road";

void intersection::initialize_state_lanes()
{
    foreach(state &st, states)
    {
        int count = 0;
        for(size_t is = 0; is < st.in_states.size(); ++is)
        {
            if(st.in_states[is].out_ref < 0)
                continue;

            ++count;
        }

        st.fict_roads.reserve(count);
        st.fict_lanes.reserve(count);

        count = 0;
        for(size_t is = 0; is < st.in_states.size(); ++is)
        {
            if(st.in_states[is].out_ref < 0)
                continue;

            const lane *in_la = incoming[is].dp;
            point in_vec;
            point in_pt;
            in_la->get_point_and_normal(1.0, in_pt, in_vec);

            const lane *out_la = outgoing[st.in_states[is].out_ref].dp;
            point out_vec;
            point out_pt;
            out_la->get_point_and_normal(0.0, out_pt, out_vec);

            point middle;

            if(std::abs(out_vec.x*in_vec.x + out_vec.y*in_vec.y) > 0.9f)
            {
                middle.x = (in_pt.x + out_pt.x)*0.5;
                middle.y = (in_pt.y + out_pt.y)*0.5;
            }
            else
                intersect_lines(middle,
                                in_pt, in_vec,
                                out_pt, out_vec);
            middle.z = (in_pt.z + out_pt.z)*0.5f;

            st.fict_roads.push_back(road());
            road &r = st.fict_roads.back();
            r.name = strdup(fict_name);
            r.rep.points.push_back(in_pt);
            r.rep.points.push_back(middle);
            r.rep.points.push_back(out_pt);
            r.rep.calc_rep();

            st.fict_lanes.push_back(lane());
            lane &la = st.fict_lanes.back();
            la.road_memberships.base_data.parent_road.dp = &r;
            la.road_memberships.base_data.interval[0] = 0.0f;
            la.road_memberships.base_data.interval[1] = 1.0f;
            la.left.base_data.neighbor.dp  = 0;
            la.right.base_data.neighbor.dp = 0;

            la.start.end_type  = lane_end::INTERSECTION;
            la.start.inters.dp = this;
            la.start.intersect_in_ref = -(is+1);
            la.end.end_type   = lane_end::INTERSECTION;
            la.end.inters.dp = this;
            la.end.intersect_in_ref = -(st.in_states[is].out_ref+1);
            la.speedlimit     = out_la->speedlimit;

            st.in_states[is].fict_lane = &la;

            st.out_states[st.in_states[is].out_ref].fict_lane = &la;
            ++count;
        }
    }
}

void intersection::translate(const point &tr)
{
    foreach(point &pt, shape)
    {
        pt.x += tr.x;
        pt.y += tr.y;
        pt.z += tr.z;
    }

    foreach(state &s, states)
    {
        foreach(road &rd, s.fict_roads)
        {
            foreach(point &pt, rd.rep.points)
            {
                pt.x += tr.x;
                pt.y += tr.y;
                pt.z += tr.z;
            }
        }
    }

}
