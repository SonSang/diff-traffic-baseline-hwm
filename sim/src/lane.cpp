#include "network.hpp"
#include <sstream>

static inline float rk4(float x, float dtinvlen, const lane *la, float gamma_c)
{
    float k[4];
    k[0] = la->velocity(x,                      gamma_c);
    k[1] = la->velocity(x + 0.5f*dtinvlen*k[0], gamma_c);
    k[2] = la->velocity(x + 0.5f*dtinvlen*k[1], gamma_c);
    k[3] = la->velocity(x +      dtinvlen*k[2], gamma_c);

    return 1.0f/6.0f*dtinvlen*(k[0] + 2.0f*k[1] + 2.0f*k[2] + k[3]);
}

template <>
bool interval_xml_read<road_membership>::xml_read(road_membership & item, xmlTextReaderPtr reader)
{
    return item.xml_read(reader);
}

template <>
bool interval_xml_read<adjacency>::xml_read(adjacency &item, xmlTextReaderPtr reader)
{
    return item.xml_read(reader);
}

static bool read_dead_end(void *item, xmlTextReaderPtr reader)
{
    lane_end *le = reinterpret_cast<lane_end*>(item);
    le->end_type = lane_end::DEAD_END;
    return true;
}

static bool read_int_ref(void *item, xmlTextReaderPtr reader)
{
    lane_end *le = reinterpret_cast<lane_end*>(item);
    le->end_type = lane_end::INTERSECTION;

    boost::fusion::vector<list_matcher<char*> > vl(lm("ref", &(le->inters.sp)));

    return read_attributes(vl, reader);
}

static bool read_lane_ref(void *item, xmlTextReaderPtr reader)
{
    lane_end *le = reinterpret_cast<lane_end*>(item);
    le->end_type = lane_end::LANE;

    boost::fusion::vector<list_matcher<char*> > vl(lm("ref", &(le->lane.sp)));

    return read_attributes(vl, reader);
}

static bool read_taper(void *item, xmlTextReaderPtr reader)
{
    lane_end *le = reinterpret_cast<lane_end*>(item);
    le->end_type = lane_end::TAPER;
    return true;
}

static bool read_start(void *item, xmlTextReaderPtr reader)
{
    const xmlChar *name = xmlTextReaderConstName(reader);

    lane *l = reinterpret_cast<lane*>(item);

    xml_elt read[] =
        {{0,
          BAD_CAST "dead_end",
          &(l->start),
          read_dead_end},
         {0,
          BAD_CAST "intersection_ref",
          &(l->start),
          read_int_ref},
         {0,
          BAD_CAST "lane_ref",
          &(l->start),
          read_lane_ref},
         {0,
          BAD_CAST "taper",
          &(l->start),
          read_taper}};

    if(!read_elements(reader, sizeof(read)/sizeof(read[0]), read, name))
        return false;

    return (read[0].count + read[1].count + read[2].count) == 1;
}

static bool read_end(void *item, xmlTextReaderPtr reader)
{
    const xmlChar *name = xmlTextReaderConstName(reader);

    lane *l = reinterpret_cast<lane*>(item);

    xml_elt read[] =
        {{0,
          BAD_CAST "dead_end",
          &(l->end),
          read_dead_end},
         {0,
          BAD_CAST "lane_ref",
          &(l->end),
          read_lane_ref},
         {0,
          BAD_CAST "intersection_ref",
          &(l->end),
          read_int_ref},
         {0,
          BAD_CAST "taper",
          &(l->end),
          read_taper}};

    if(!read_elements(reader, sizeof(read)/sizeof(read[0]), read, name))
        return false;

    return (read[0].count + read[1].count + read[2].count) == 1;
}

static bool read_road_membership_interval(void *item, xmlTextReaderPtr reader)
{
    lane *l = reinterpret_cast<lane*>(item);
    return l->road_memberships.xml_read(reader, BAD_CAST "road_membership");
}

static bool read_road_intervals(void *item, xmlTextReaderPtr reader)
{
    xml_elt read[] =
        {{0,
          BAD_CAST "interval",
          item,
          read_road_membership_interval}};

    if(!read_elements(reader, sizeof(read)/sizeof(read[0]), read, BAD_CAST "road_intervals"))
        return false;

    return read[0].count == 1;
}

static bool read_left_interval(void *item, xmlTextReaderPtr reader)
{
    lane *l = reinterpret_cast<lane*>(item);
    return l->left.xml_read(reader, BAD_CAST "lane_adjacency");
}

static bool read_left_lane(void *item, xmlTextReaderPtr reader)
{
    xml_elt read[] =
        {{0,
          BAD_CAST "interval",
          item,
          read_left_interval}};

    if(!read_elements(reader, sizeof(read)/sizeof(read[0]), read, BAD_CAST "left"))
        return false;

    return read[0].count == 1;
}

static bool read_right_interval(void *item, xmlTextReaderPtr reader)
{
    lane *l = reinterpret_cast<lane*>(item);
    return l->right.xml_read(reader, BAD_CAST "lane_adjacency");
}

static bool read_right_lane(void *item, xmlTextReaderPtr reader)
{
    xml_elt read[] =
        {{0,
          BAD_CAST "interval",
          item,
          read_right_interval}};

    if(!read_elements(reader, sizeof(read)/sizeof(read[0]), read, BAD_CAST "right"))
        return false;

    return read[0].count == 1;
}

static bool read_adjacency(void *item, xmlTextReaderPtr reader)
{
    xml_elt read[] =
        {{0,
          BAD_CAST "left",
          item,
          read_left_lane},
         {0,
          BAD_CAST "right",
          item,
          read_right_lane}};

    if(!read_elements(reader, sizeof(read)/sizeof(read[0]), read, BAD_CAST "adjacency_intervals"))
        return false;

    return read[0].count == 1 && read[1].count == 1;
}

bool road_membership::xml_read(xmlTextReaderPtr reader)
{
    boost::fusion::vector<list_matcher<char*>,
        list_matcher<float>,
        list_matcher<float>,
        list_matcher<float> > vl(lm("parent_road_ref", &(parent_road.sp)),
                                 lm("interval_start", interval),
                                 lm("interval_end", interval+1),
                                 lm("lane_position", &lane_position));

    return read_attributes(vl, reader);
}

bool adjacency::xml_read(xmlTextReaderPtr reader)
{
    boost::fusion::vector<list_matcher<char*>,
        list_matcher<float>,
        list_matcher<float> > vl(lm("lane_ref", &(neighbor.sp)),
                                 lm("interval_start", neighbor_interval),
                                 lm("interval_end", neighbor_interval+1));

    merge_weight = 0;
    if(!read_attributes(vl, reader))
    {
        neighbor.sp = 0;

        if(!xmlTextReaderIsEmptyElement(reader))
        {
            std::string pts;
            read_leaf_text(pts, reader, "lane_adjacency");

            std::stringstream ss(pts);
            float temp;
            ss >> temp;
            while(ss.good())
            {
                source_sinks.push_back(source_sink(temp, 2));
                ss >> temp;
            }
        }
    }
    else
        get_attribute(merge_weight, reader, BAD_CAST "merge_weight");

    return true;
};

int adjacency::find_source(float x) const
{
    int start = 0;
    int end = source_sinks.size();
    if(!end || x > source_sinks[end-1].pos)
        return -1;
    if(x < source_sinks[start].pos)
        return 0;

    while(start + 1 < end)
    {
        int mid = (start+end)/2;
        if(x < source_sinks[mid].pos)
            end = mid;
        else
            start = mid;
    }
    return start+1;
}

typedef boost::minstd_rand base_generator_type;
boost::variate_generator<base_generator_type&, boost::uniform_real<> > *adjacency::uni;

bool lane::xml_read(xmlTextReaderPtr reader)
{
    boost::fusion::vector<list_matcher<float> > vl(lm("speedlimit", &speedlimit));

    if(!read_attributes(vl, reader))
        return false;

    xml_elt read[] =
        {{0,
          BAD_CAST "start",
          this,
          read_start},
         {0,
          BAD_CAST "end",
          this,
          read_end},
         {0,
          BAD_CAST "road_intervals",
          this,
          read_road_intervals},
         {0,
          BAD_CAST "adjacency_intervals",
          this,
          read_adjacency}};

    bool status = read_elements(reader, sizeof(read)/sizeof(read[0]), read, BAD_CAST "lane");
    if(!status)
        return false;

    return read[0].count == 1 && read[1].count == 1 && read[2].count == 1 && read[3].count == 1;
}

float lane::calc_length() const
{
    float length = 0.0f;

    const road_membership *rom = &(road_memberships.base_data);
    int p = -1;
    while(1)
    {
        length += std::abs((rom->interval[1] - rom->interval[0]))*rom->parent_road.dp->rep.offset_length(rom->lane_position);

        ++p;
        if(p >= static_cast<int>(road_memberships.entries.size()))
            break;
        rom = &(road_memberships.entries[p].data);
    }

    return length;
}

void lane::auto_scale_memberships()
{
    std::vector<float> mlengths;
    mlengths.push_back(0);

    {
        const road_membership *rom = &(road_memberships.base_data);
        int p = -1;
        while(1)
        {
            float length = std::abs((rom->interval[1] - rom->interval[0]))*rom->parent_road.dp->rep.offset_length(rom->lane_position) + mlengths.back();

            mlengths.push_back(length);

            ++p;
            if(p >= static_cast<int>(road_memberships.entries.size()))
                break;
            rom = &(road_memberships.entries[p].data);
        }
    }

    float inv_total_len = 1.0f/mlengths.back();

    for(size_t i = 0; i < road_memberships.entries.size(); ++i)
        road_memberships.entries[i].divider = mlengths[i+1]*inv_total_len;

    return;
}


void lane::scale_offsets(float f)
{
    road_membership *rom = &(road_memberships.base_data);
    int p = -1;
    while(1)
    {
        if(rom->lane_position < 0)
            rom->lane_position +=  0.5f;
        else if(rom->lane_position > 0)
            rom->lane_position += -0.5f;
        rom->lane_position *= f;

        ++p;
        if(p >= static_cast<int>(road_memberships.entries.size()))
            break;
        rom = &(road_memberships.entries[p].data);
    }
}

lane* lane::left_lane(float &t) const
{
    const adjacency &adj = left.get_rescale(t);
    if(adj.neighbor.dp)
        t = t*(adj.neighbor_interval[1]-adj.neighbor_interval[0]) + adj.neighbor_interval[0];

    return adj.neighbor.dp;
}

lane* lane::right_lane(float &t) const
{
    const adjacency &adj = right.get_rescale(t);
    if(adj.neighbor.dp)
        t = t*(adj.neighbor_interval[1]-adj.neighbor_interval[0]) + adj.neighbor_interval[0];

    return adj.neighbor.dp;
}

const adjacency &lane::left_adjacency(float &t) const
{
    const adjacency &adj = left.get_rescale(t);
    if(adj.neighbor.dp)
        t = t*(adj.neighbor_interval[1]-adj.neighbor_interval[0]) + adj.neighbor_interval[0];

    return adj;
}

const adjacency &lane::right_adjacency(float &t) const
{
    const adjacency &adj = right.get_rescale(t);
    if(adj.neighbor.dp)
        t = t*(adj.neighbor_interval[1]-adj.neighbor_interval[0]) + adj.neighbor_interval[0];

    return adj;
}

const adjacency &lane::left_adjacency(float &t, float &myend) const
{
    const intervals<adjacency>::entry_id loc = left.find(t);

    myend = left.entries.empty() ? 1.0f : left.entries[loc+1].divider;
    const adjacency &adj = (loc == -1) ?  left.base_data : left.entries[loc].data;
    if(adj.neighbor.dp)
        t = t*(adj.neighbor_interval[1]-adj.neighbor_interval[0]) + adj.neighbor_interval[0];

    return adj;
}

const adjacency &lane::right_adjacency(float &t, float &myend) const
{
    const intervals<adjacency>::entry_id loc = right.find(t);

    myend = right.entries.empty() ? 1.0f : right.entries[loc+1].divider;
    const adjacency &adj = (loc == -1) ?  right.base_data : right.entries[loc].data;
    if(adj.neighbor.dp)
        t = t*(adj.neighbor_interval[1]-adj.neighbor_interval[0]) + adj.neighbor_interval[0];

    return adj;
}

lane* lane::upstream_lane() const
{
    switch(start.end_type)
    {
    case lane_end::DEAD_END:
    case lane_end::TAPER:
        return 0;
    case lane_end::LANE:
        return start.lane.dp;
    case lane_end::INTERSECTION:
        return start.inters.dp->outgoing_state(start.intersect_in_ref);
    default:
        assert(0);
        return 0;
    };
}

lane* lane::downstream_lane() const
{
    switch(end.end_type)
    {
    case lane_end::DEAD_END:
    case lane_end::TAPER:
        return 0;
    case lane_end::LANE:
        return end.lane.dp;
    case lane_end::INTERSECTION:
        return end.inters.dp->incoming_state(end.intersect_in_ref);
    default:
        assert(0);
        return 0;
    };
}

void lane::get_matrix(const float &t, float mat[16]) const
{
    point p, n;
    get_point_and_normal(t, p, n);

    mat[ 0]=n.x;  mat[ 1]= n.y; mat[ 2]=0.0f;mat[ 3]=0.0f;
    mat[ 4]=-n.y; mat[ 5]= n.x; mat[ 6]=0.0f;mat[ 7]=0.0f;
    mat[ 8]=0.0f; mat[ 9]= 0.0f;mat[10]=1.0f;mat[11]=0.0f;
    mat[12]=p.x;  mat[13]= p.y; mat[14]=p.z ;mat[15]=1.0f;
}

int lane::get_point_and_normal(const float &t, point &p, point &n) const
{
    float x = t;
    const road_membership *rom = &(road_memberships.get_rescale(x));

    rom->parent_road.dp->rep.locate_vec(&p, &n, x*(rom->interval[1]-rom->interval[0])+rom->interval[0], rom->lane_position);
    if(rom->interval[0] > rom->interval[1])
    {
        n.x *= -1.0f;
        n.y *= -1.0f;

        return -1;
    }

    return 1;
}

void lane::get_point(const float &t, point &pt) const
{
    float x = t;
    const road_membership *rom = &(road_memberships.get_rescale(x));
    x = x*(rom->interval[1]-rom->interval[0])+rom->interval[0];
    rom->parent_road.dp->rep.locate(&pt, x, rom->lane_position);
}

float lane::velocity(float t, float gamma_c) const
{
    float pos = t*ncells - 0.5f;
    int cell = static_cast<int>(std::floor(pos));
    float u[2];
    float rho[2];

    if(cell < -1)
        cell = -1;
    else if(cell > static_cast<int>(ncells+1))
        cell = static_cast<int>(ncells+1);

    if(cell < 0)
    {
        const lane *prev = upstream_lane();
        if(prev)
        {
            //* not strictly right, because of difference in 'h'
            //* between lanes
            //* but close enough
            u[0] = to_u(prev->data[prev->ncells-1].rho,
                        prev->data[prev->ncells-1].y,
                        prev->speedlimit, gamma_c);
            rho[0] = prev->data[prev->ncells-1].rho;
        }
        else
        {
            u[0] = 0.0f;
            rho[0] = 1.0f;
        }
    }
    else
    {
        u[0] = to_u(data[cell].rho, data[cell].y, speedlimit, gamma_c);
        rho[0] = data[cell].rho;
    }

    if(cell + 1 >= static_cast<int>(ncells))
    {
        const lane *next = downstream_lane();
        if(next)
        {
            //* not strictly right, because of difference in 'h'
            //* between lanes
            //* but close enough
            u[1] = to_u(next->data[0].rho,
                        next->data[0].y,
                        next->speedlimit, gamma_c);
            rho[1] = next->data[0].rho;
        }
        else
        {
            u[1] = 0.0f;
            rho[1] = 1.0f;
        }
    }
    else
    {
        u[1] = to_u(data[cell+1].rho, data[cell+1].y, speedlimit, gamma_c);
        rho[1] = data[cell+1].rho;
    }

    float f0 = rho[0]*(1.0f - (pos-cell));
    float f1 = rho[1]*(pos-cell);
    float denom = (f0+f1);
    if(denom < 1e-4)
        return speedlimit;
    return (f0*u[0] + f1*u[1])/denom;
}

void lane::fill_from_carticles()
{
    float inv_h = 1.0f/h;

    foreach(carticle &cart, carticles[0])
    {
        float front_pos = cart.x*ncells + CAR_REAR_AXLE*inv_h;
        float back_pos  = front_pos - CAR_LENGTH*inv_h;

        int front_cell = static_cast<int>(std::floor(front_pos));
        int back_cell  = static_cast<int>(std::floor(back_pos));

        assert(front_cell - back_cell <= 1);
        if(front_cell < static_cast<int>(ncells))
        {
            float portion = front_pos-front_cell;
            data[front_cell].rho += portion;
            data[front_cell].y   += portion*cart.u;
        }

        if(back_cell < 0)
        {
            lane* up = upstream_lane();
            if(up)
            {
                float portion = (front_cell-back_pos)*inv_h*up->h;
                up->data[up->ncells-1].rho += portion;
                up->data[up->ncells-1].y   += portion*cart.u;
            }
        }
        else
        {
            float portion = front_cell-back_pos;
            data[back_cell].rho += portion;
            data[back_cell].y   += portion*cart.u;
        }
    }
}

void lane::reset_data()
{
    memset(data, 0, sizeof(q)*ncells);
    memset(merge_states, 0, sizeof(merge_state)*ncells);
}

void lane::fill_y(float gamma)
{
    for(size_t i = 0; i < ncells; ++i)
    {
        if(data[i].rho > 0.0)
            data[i].y /= data[i].rho;

        data[i].y = to_y(data[i].rho, data[i].y, speedlimit, gamma);
        data[i].fix();
    }
}

float lane::collect_riemann(float gamma_c, float inv_gamma)
{
    full_q fq_buff[2];
    full_q * fq[2] = {fq_buff, fq_buff + 1};

    float maxspeed = 0.0f;

    fq[0]->from_q(data,
                  speedlimit, gamma_c);

    float inv_speedlimit = 1.0f/speedlimit;

    if(upstream_lane() == 0)
    {
        starvation_riemann(rs,
                           fq[0],
                           speedlimit,
                           inv_speedlimit,
                           gamma_c,
                           inv_gamma);

        assert(rs[0].check());

        // we know that maxspeed previously was 0, and
        // that starvation_riemann has a nonnegative speed in speeds[1]
        // and nothing in speeds[0]
        maxspeed = rs[0].speeds[1];
    }
    else
        memset(rs, 0, sizeof(riemann_solution));

    for(size_t i = 1; i < ncells; ++i)
    {
        fq[1]->from_q(data + i,
                      speedlimit, gamma_c);

        riemann(rs + i,
                fq[0],
                fq[1],
                speedlimit,
                inv_speedlimit,
                gamma_c,
                inv_gamma);

        assert(rs[i].check());
        maxspeed = std::max(maxspeed, std::max(std::abs(rs[i].speeds[0]), std::abs(rs[i].speeds[1])));

        std::swap(fq[0], fq[1]);
    }

    // if(downstream_lane() == 0)
    // {
    //     stop_riemann(rs+ncells,
    //                  fq[0],
    //                  speedlimit,
    //                  inv_speedlimit,
    //                  gamma_c,
    //                  inv_gamma);

    //     assert(rs[ncells].check());

    //     // we know that stop_riemann has a speed in speeds[0]
    //     // and nothing in speeds[1]
    //     maxspeed = std::max(maxspeed, std::abs(rs[ncells].speeds[0]));

    // }
    // else
        memset(rs+ncells, 0, sizeof(riemann_solution));

    return maxspeed;
}

void lane::update(float dt)
{
    const float relaxation_fac = 0.4;

    float coeff = dt/h;

    if(ncells == 1)
    {
        int i = 0;
        data[i].rho -= coeff*(rs[i].fluct_r.rho + rs[i+1].fluct_l.rho);
        data[i].y   -= coeff*(rs[i].fluct_r.y   + rs[i+1].fluct_l.y);

        data[i].y   -= data[i].y*coeff*relaxation_fac;

        data[i].fix();
        return;
    }
    size_t i = 0;

    data[i].rho -= coeff*(rs[i].fluct_r.rho + rs[i+1].fluct_l.rho);
    data[i].y   -= coeff*(rs[i].fluct_r.y   + rs[i+1].fluct_l.y);

    data[i].y   -= data[i].y*coeff*relaxation_fac;

    data[i].fix();

    for(i = 1; i < ncells-1; ++i)
    {
        data[i].rho -= coeff*(rs[i].fluct_r.rho + rs[i+1].fluct_l.rho);
        data[i].y   -= coeff*(rs[i].fluct_r.y   + rs[i+1].fluct_l.y);

        data[i].y   -= data[i].y*coeff*relaxation_fac;

        data[i].fix();
    }

    data[i].rho -= coeff*(rs[i].fluct_r.rho + rs[i+1].fluct_l.rho);
    data[i].y   -= coeff*(rs[i].fluct_r.y   + rs[i+1].fluct_l.y);

    data[i].y   -= data[i].y*coeff*relaxation_fac;

    data[i].fix();
}

struct lc_curve
{
    lc_curve(float target_y) : y_(target_y) {}

    inline float operator()(float t) const
    {
        return y(t) - y_;
    }

    static inline float y(float t)
    {
        return ((((8.7025182813*t+
                   -21.7562980512*t) +
                  15.5458393998*t) +
                 -1.56245957436*t) +
                0.0709664576132*t) +
        -0.000283090356282;
    }

    static inline float end(float speed)
    {
        return 11.50731f*std::pow(speed, -0.5f);
    }

    float y_;
};

int lane::merge_intent(float local_t, float gamma_c) const
{
    float            left_t   = local_t;
    const adjacency &l_adj    = left_adjacency(left_t);
    const lane      *left_la  = l_adj.neighbor.dp;
    float            right_t  = local_t;
    const adjacency &r_adj    = right_adjacency(right_t);
    const lane      *right_la = r_adj.neighbor.dp;

    if(!(left_la || right_la))
        return 0;

    const float left_weight  = l_adj.weighted_merge();
    const float right_weight = r_adj.weighted_merge();

    if(left_weight > right_weight && left_weight > 0.0f)
        return -1;
    else if(right_weight > left_weight && right_weight > 0.0f)
        return 1;

    int mycell = static_cast<int>(std::floor(local_t*ncells));
    if(mycell < 0)
        mycell = 0;
    else if(mycell >= ncells)
        mycell = ncells-1;
    if(data[mycell].rho <= 1e-4)
        return 0;

    float u = to_u(data[mycell].rho, data[mycell].y, speedlimit, gamma_c);

    float ahead_u;
    if(mycell + 1 < static_cast<int>(ncells))
        ahead_u = to_u(data[mycell+1].rho, data[mycell+1].y, speedlimit, gamma_c);
    else
        return 0;

    if(ahead_u >= 0.8*u)
        return 0;

    float left_factor = 0.0f;
    float right_factor = 0.0f;

    if(left_la)
    {
        int othercell = static_cast<int>(std::floor(left_t*left_la->ncells));
        if(othercell < 0)
            othercell = 0;
        else if(othercell >= ncells)
            othercell = ncells-1;
        float other_u = to_u(left_la->data[othercell].rho, left_la->data[othercell].y, left_la->speedlimit, gamma_c);
        left_factor   = (other_u > ahead_u) ? (other_u - ahead_u)/speedlimit : 0.0f;
    }

    if(right_la)
    {
        int othercell = static_cast<int>(std::floor(right_t*right_la->ncells));
        if(othercell < 0)
            othercell = 0;
        else if(othercell >= ncells)
            othercell = ncells-1;
        float other_u = to_u(right_la->data[othercell].rho, right_la->data[othercell].y, right_la->speedlimit, gamma_c);
        right_factor  = (other_u > ahead_u) ? (other_u - ahead_u)/speedlimit : 0.0f;
    }

    if(right_factor > left_factor)
        return (right_factor > 0.5f) ? 1 : 0;
    else
        return (left_factor  > 0.5f) ? -1 : 0;
}

bool lane::merge_possible(carticle &c, int dir, float gamma_c) const
{
    assert(dir);

    if(c.u < 1.0f)
        return false;

    float other_t = c.x;
    float end_int;
    const adjacency &other_adj = (dir == -1) ? left_adjacency(other_t, end_int) : right_adjacency(other_t, end_int);
    const lane *other_la = other_adj.neighbor.dp;
    assert(other_la);

    if(other_la == c.lastlane)
        return false;

    float end = lc_curve::end(c.u);
    if(c.x + end/(h*ncells) > end_int)
        return false;

    int lastcell = static_cast<int>(std::ceil(other_t*other_la->ncells + end*c.u/other_la->h));

    if(lastcell >= static_cast<int>(other_la->ncells))
        return false;

    int loc = static_cast<int>(std::floor(other_t*other_la->ncells));
    for(; loc < lastcell; ++loc)
        if(other_la->data[loc].rho > 0.1f)
            return false;

    return true;
}

struct turn_curve
{
    turn_curve(float in_x) : x_(in_x) {}

    float operator()(float t)
    {
        return x(t) - x_;
    }

    float x_;

    static float x(float t)
    {
        return (((0.952735372896*t+
                  -0.999804060107*t) +
                 -1.72981115096*t) +
                2.77923310042*t) +
            0.000979602079628;
    }

    static float y(float t)
    {
        return ((((4.95688785514*t+
                   -12.4993312165*t) +
                  9.28482561644*t) +
                 -0.8082796784*t) +
                0.0691053639752*t) +
            -0.00106975213459;
    }

    static float theta(float t)
    {
        const float orientation = M_PI/2.0f;
        const float kmax = 1.1172f;
        const float m = 2.0f*orientation/kmax;
        const float inv_m = 1.0f/m;

        t *= m;

        if(t <= m/2)
            return kmax*t*t*inv_m;
        else if(t <= m)
            return -kmax*(t*t*inv_m - 2*t + m/2);
        else
            return kmax*m/2;
    }

    static float x_end(float speed)
    {
        return  0.843112757647*speed+
            2.45445917647;
    }

    static float y_end(float speed)
    {
        return 0.244331745882*speed+
            4.11774005882;
    }
};

void lane::find_ss(float x)
{
    ss_walker ssw(*this);

    while(!ssw.end())
    {
        printf("res %f, side: %d\n", ssw.front()->pos, ssw.side);
        ssw.advance();
    }
}

void lane::advance_carticles(float dt, float gamma_c)
{
    float inv_len = 1.0f/(ncells*h);


    int count =0;
    foreach(carticle &cart, carticles[0])
    {
        // advance vehicle
        float param_u = rk4(cart.x, dt*inv_len, this, gamma_c);
        cart.x += param_u*std::cos(cart.theta);
        cart.u = param_u*(ncells*h)/dt;

        if(cart.free_motion())
        {
            // check for intent to merge
            int intent = merge_intent(cart.x, gamma_c);

            // check for possibility of merge
            if(intent && merge_possible(cart, intent, gamma_c)
               && count+1 < carticles[0].size() && carticles[0][count+1].free_motion())
                cart.start_lane_change(intent);
        }

        ++count;

        if(cart.in_lane_change())
        {
            assert(cart.x < 1.0);
            float end = lc_curve::end(cart.u);

            float y_lookup = std::abs(cart.y);
            if(cart.y*cart.lane_change_dir() < 0.0f)
                y_lookup = 1.0 - y_lookup;

            lc_curve t_solve(y_lookup);
            float t = secant<lc_curve>(0.1f, 0.5f, 0.0f, 1.0f, 1e-4f, 100, t_solve);

            float prev_y = cart.y;
            float new_t = t + dt/end;
            float new_y = (new_t < 1.0f) ? lc_curve::y(new_t) : 1.0f;

            cart.y = new_y;
            if(prev_y*cart.lane_change_dir() < 0.0f)
                cart.y -= 1.0f;

            cart.y *= cart.lane_change_dir();

            float del_y = cart.y - prev_y;
            cart.theta = std::atan2(del_y*LANE_WIDTH, cart.u*dt);

            float back_of_carticle  = cart.x - (CAR_LENGTH-CAR_REAR_AXLE)*inv_len;
            float front_of_carticle = cart.x +  CAR_REAR_AXLE*inv_len;

            int first_cell = static_cast<int>(std::floor(back_of_carticle*ncells));
            int last_cell  = static_cast<int>(std::floor(front_of_carticle*ncells));
            if(first_cell < 0)
                first_cell = 0;
            if(last_cell >= ncells)
                last_cell = ncells-1;

            for(int i = first_cell; i < last_cell; ++i)
                merge_states[i].transition = del_y;

            if(cart.y < -0.5f)
            {
                assert(cart.lane_change_dir() == -1);
                cart.y += 1.0f;

                lane *llane = lane::left_lane(cart.x);
                if(llane)
                {
                    cart.lastlane = this;
                    llane->carticles[1].push_back(cart);
                }

                continue;
            }
            else if(cart.y > 0.5f)
            {
                assert(cart.lane_change_dir() == 1);
                cart.y -= 1.0f;

                lane *rlane = lane::right_lane(cart.x);
                if(rlane)
                {
                    cart.lastlane = this;
                    rlane->carticles[1].push_back(cart);
                }

                continue;
            }
            else if(cart.y*cart.lane_change_dir() <= 0.0f && std::abs(cart.y) < 1e-2f)
            {
                cart.y = 0.0f;
                cart.theta = 0.0f;
                cart.end_lane_change();
            }
        }

        if(cart.x > 1.0 - CAR_REAR_AXLE * inv_len)
        {
            lane *next;
            if(end.end_type == lane_end::LANE ||
               end.end_type == lane_end::INTERSECTION)
            {
                if((next = downstream_lane()))
                {
                    cart.x = (cart.x - (1.0 - CAR_REAR_AXLE * inv_len)) * ncells*h/(next->ncells*next->h);
                    cart.lastlane = this;
                    next->carticles[1].push_back(cart);
                }
                else
                {
                    cart.x = 1.0f - CAR_REAR_AXLE * inv_len;
                    cart.lastlane = this;
                    carticles[1].push_back(cart);
                }
            }
            continue;
        }
        carticles[1].push_back(cart);
    }
}

struct carticle_sort
{
    inline bool operator()(const carticle &l, const carticle &r)
    {
        return l.x < r.x;
    }
};

void lane::swap_carticles()
{
    carticles[0].swap(carticles[1]);
    carticle_sort cs;
    std::sort(carticles[0].begin(), carticles[0].end(), cs);

    if(carticles[0].size() > 1)
        for(int i = static_cast<int>(carticles[0].size())-2; i >= 0; --i)
        {
            assert(i >= 0);
            assert(i+1 < static_cast<int>(carticles[0].size()));
            if(carticles[0][i].x > carticles[0][i+1].x - (CAR_LENGTH*2.5f)/(ncells*h))
                carticles[0][i].x = carticles[0][i+1].x - (CAR_LENGTH*2.5f)/(ncells*h);
        }
    carticles[1].clear();
}

void lane::clear_merges()
{
    memset(merge_states, 0, sizeof(merge_state)*ncells);
}

void lane::apply_merges(float dt, float gamma_c)
{
    float inv_ncells = 1.0f/ncells;
    for(size_t i = 0; i < ncells; ++i)
    {
        if(merge_states[i].transition < 0.0f)
        {
            merge_states[i].transition *= -1;

            float param = i*inv_ncells;
            lane *la = left_lane(param);
            if(la)
            {
                int neighbor_cell = static_cast<int>(std::floor(param*la->ncells));
                if(merge_states[i].transition > data[i].rho)
                    merge_states[i].transition = data[i].rho;
                if(merge_states[i].transition + la->data[neighbor_cell].rho > 1.0)
                    merge_states[i].transition = 1.0f - la->data[neighbor_cell].rho;

                float my_u = to_u(data[i].rho, data[i].y, speedlimit, gamma_c);
                float neighbor_u = to_u(la->data[neighbor_cell].rho,
                                        la->data[neighbor_cell].y,
                                        la->speedlimit, gamma_c);

                data[i].rho -= merge_states[i].transition;
                data[i].y    = to_y(data[i].rho, my_u, speedlimit, gamma_c);

                la->data[neighbor_cell].rho += merge_states[i].transition;
                la->data[neighbor_cell].y    = to_y(la->data[neighbor_cell].rho, (neighbor_u+my_u)*0.5, la->speedlimit, gamma_c);

                data[i].fix();
                la->data[neighbor_cell].fix();
            }
        }
        else if(merge_states[i].transition > 0.0f)
        {
            float param = i*inv_ncells;
            lane *la = right_lane(param);
            if(la)
            {
                int neighbor_cell = static_cast<int>(std::floor(param*la->ncells));
                if(merge_states[i].transition > data[i].rho)
                    merge_states[i].transition = data[i].rho;
                if(merge_states[i].transition + la->data[neighbor_cell].rho > 1.0)
                    merge_states[i].transition = 1.0f - la->data[neighbor_cell].rho;

                float my_u = to_u(data[i].rho, data[i].y, speedlimit, gamma_c);
                float neighbor_u = to_u(la->data[neighbor_cell].rho,
                                        la->data[neighbor_cell].y,
                                        la->speedlimit, gamma_c);

                data[i].rho                    -= merge_states[i].transition;
                data[i].y    = to_y(data[i].rho, my_u, speedlimit, gamma_c);

                la->data[neighbor_cell].rho += merge_states[i].transition;
                la->data[neighbor_cell].y    = to_y(la->data[neighbor_cell].rho, (neighbor_u+my_u)*0.5, la->speedlimit, gamma_c);

                data[i].fix();
                la->data[neighbor_cell].fix();
            }
        }
    }
}

void lane::dump_carticles(FILE *fp, const float origin[2]) const
{
    foreach(const carticle &ca, carticles[0])
    {
        point pt, n, nf;
        float param = ca.x;
        get_point_and_normal(param, pt, n);

        pt.x -= origin[0];
        pt.y -= origin[1];

        float s = -std::sin(ca.theta);
        float c =  std::cos(ca.theta);
        nf.x = c*n.x + -s*n.y;
        nf.y = s*n.x +  c*n.y;

        fprintf(fp, "%d %f %f %f %f %f %f %d\n", ca.id, pt.x+LANE_WIDTH*ca.y*n.y, pt.y-LANE_WIDTH*ca.y*n.x, pt.z, nf.x, nf.y, ca.u, ca.motion_state);
    }
}
