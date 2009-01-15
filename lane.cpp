#include "network.hpp"

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

static bool read_left_adjacency(void *item, xmlTextReaderPtr reader)
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

static bool read_right_adjacency(void *item, xmlTextReaderPtr reader)
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
          read_left_adjacency},
         {0,
          BAD_CAST "right",
          item,
          read_right_adjacency}};

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

    if(!read_attributes(vl, reader))
        neighbor.sp = 0;

    return true;
};

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

void lane::scale_offsets(float f)
{
    road_membership *rom = &(road_memberships.base_data);
    int p = -1;
    while(1)
    {
        rom->lane_position *= f;

        ++p;
        if(p >= static_cast<int>(road_memberships.entries.size()))
            break;
        rom = &(road_memberships.entries[p].data);
    }
}

lane* lane::left_adjacency(float &t) const
{
    const adjacency &adj = left.get_rescale(t);
    if(adj.neighbor.dp)
        t = t*(adj.neighbor_interval[1]-adj.neighbor_interval[0]) + adj.neighbor_interval[0];

    return adj.neighbor.dp;
}

lane* lane::right_adjacency(float &t) const
{
    const adjacency &adj = right.get_rescale(t);
    if(adj.neighbor.dp)
        t = t*(adj.neighbor_interval[1]-adj.neighbor_interval[0]) + adj.neighbor_interval[0];

    return adj.neighbor.dp;
}

lane* lane::upstream_lane() const
{
    switch(start.end_type)
    {
    case lane_end::DEAD_END:
    case lane_end::TAPER:
        return 0;
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
    mat[ 4]=n.y;  mat[ 5]=-n.x; mat[ 6]=0.0f;mat[ 7]=0.0f;
    mat[ 8]=0.0f; mat[ 9]= 0.0f;mat[10]=1.0f;mat[11]=0.0f;
    mat[12]=p.x;  mat[13]= p.y; mat[14]=0.0f;mat[15]=1.0f;
}

void lane::get_point_and_normal(const float &t, point &p, point &n) const
{
    float x = t;
    const road_membership *rom = &(road_memberships.get_rescale(x));

    rom->parent_road.dp->rep.locate_vec(&p, &n, x*(rom->interval[1]-rom->interval[0])+rom->interval[0], rom->lane_position);
    if(rom->interval[0] > rom->interval[1])
    {
        n.x *= -1.0f;
        n.y *= -1.0f;
    }
}

void lane::get_point(const float &t, point &pt) const
{
    float x = t;
    const road_membership *rom = &(road_memberships.get_rescale(x));
    x = x*(rom->interval[1]-rom->interval[0])+rom->interval[0];
    rom->parent_road.dp->rep.locate(&pt, x, rom->lane_position);
}

void lane::fill_from_carticles()
{
    float inv_h = 1.0f/h;

    for(size_t i = 0; i < carticles[0].size(); ++i)
    {
        float front_pos = carticles[0][i].x*(ncells-1);
        float back_pos  = front_pos - CAR_LENGTH*inv_h;

        int front_cell = std::floor(front_pos);
        int back_cell  = std::floor(back_pos);

        assert(front_cell - back_cell <= 1);
        assert(front_cell < static_cast<int>(ncells));

        float portion = front_pos-front_cell;
        data[front_cell].rho += portion;
        data[front_cell].y   += portion*carticles[0][i].u;

        if(back_cell < 0)
        {
            lane* up = upstream_lane();
            if(up)
            {
                portion = (back_cell-back_pos)*inv_h*up->h;
                up->data[up->ncells-1].rho += portion;
                up->data[up->ncells-1].y   += portion*carticles[0][i].u;
            }
        }
        else
        {
            portion = back_cell-back_pos;
            data[back_cell].rho -= portion;
            data[back_cell].y   -= portion*carticles[0][i].u;
        }
    }
}

void lane::reset_data()
{
    memset(data, 0, sizeof(q)*ncells);
}

void lane::fill_y(float gamma)
{
    for(size_t i = 0; i < ncells; ++i)
    {
        if(data[i].rho < FLT_EPSILON)
            data[i].rho = FLT_EPSILON;
        data[i].y = to_y(data[i].rho, data[i].y, speedlimit, gamma);
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

        // we know that maxspeed previously was 0, and
        // that starvation_riemann has a nonnegative speed in speeds[1]
        // and nothing in speeds[0]
        maxspeed = rs[0].speeds[1];
        assert(std::isfinite(maxspeed));
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

        maxspeed = std::max(maxspeed, std::max(std::abs(rs[i].speeds[0]), std::abs(rs[i].speeds[1])));
        assert(std::isfinite(rs[i].speeds[0]));
        assert(std::isfinite(rs[i].speeds[1]));

        std::swap(fq[0], fq[1]);
    }

    if(downstream_lane() == 0)
    {
        stop_riemann(rs+ncells,
                     fq[0],
                     speedlimit,
                     inv_speedlimit,
                     gamma_c,
                     inv_gamma);

        // we know that stop_riemann has a speed in speeds[0]
        // and nothing in speeds[1]
        maxspeed = std::max(maxspeed, std::abs(rs[ncells].speeds[0]));
        assert(std::isfinite(rs[ncells].speeds[0]));
    }
    else
        memset(rs+ncells, 0, sizeof(riemann_solution));

    return maxspeed;
}

void lane::update(float dt)
{
    float coeff = dt/h;

    q limited_base[2];
    q *limited[2] = {limited_base, limited_base+1};

    memset(limited[0], 0, sizeof(q));

    (*limited[1]) = flux_correction(rs,
                                    rs + 1,
                                    rs + 2,
                                    coeff);

    size_t i = 0;

    data[i].rho -= coeff*(rs[i].fluct_r.rho + rs[i+1].fluct_l.rho + (*limited[1]).rho - (*limited[0]).rho);
    data[i].y   -= coeff*(rs[i].fluct_r.y   + rs[i+1].fluct_l.y   + (*limited[1]).y   - (*limited[0]).y);

    assert(std::isfinite(data[i].rho) && std::isfinite(data[i].y));

    if(data[i].rho < FLT_EPSILON)
        data[i].rho  = FLT_EPSILON;
    if(data[i].y > FLT_EPSILON)
        data[i].y = FLT_EPSILON;

    std::swap(limited[0], limited[1]);

    for(i = 1; i < ncells-1; ++i)
    {
        (*limited[1]) = flux_correction(rs + i,
                                        rs + i + 1,
                                        rs + i + 2,
                                        coeff);

        data[i].rho -= coeff*(rs[i].fluct_r.rho + rs[i+1].fluct_l.rho + (*limited[1]).rho - (*limited[0]).rho);
        data[i].y   -= coeff*(rs[i].fluct_r.y   + rs[i+1].fluct_l.y   + (*limited[1]).y   - (*limited[0]).y);

        assert(std::isfinite(data[i].rho) && std::isfinite(data[i].y));

        if(data[i].rho < FLT_EPSILON)
            data[i].rho  = FLT_EPSILON;
        if(data[i].y > FLT_EPSILON)
            data[i].y = FLT_EPSILON;

        std::swap(limited[0], limited[1]);
    }

    memset(limited[1], 0, sizeof(q));

    data[i].rho -= coeff*(rs[i].fluct_r.rho + rs[i+1].fluct_l.rho + (*limited[1]).rho - (*limited[0]).rho);
    data[i].y   -= coeff*(rs[i].fluct_r.y   + rs[i+1].fluct_l.y   + (*limited[1]).y   - (*limited[0]).y);

    assert(std::isfinite(data[i].rho) && std::isfinite(data[i].y));

    if(data[i].rho < FLT_EPSILON)
        data[i].rho  = FLT_EPSILON;
    if(data[i].y > FLT_EPSILON)
        data[i].y  = FLT_EPSILON;
}

void lane::advance_carticles(float dt, float gamma_c)
{
    float inv_len = 1.0f/(ncells*h);

    for(int i = 0; i < static_cast<int>(carticles[0].size()); ++i)
    {
        float pos = carticles[0][i].x*(ncells-1);
        int cell = std::floor(pos);
        float cell_u = to_u(data[cell].rho, data[cell].y, speedlimit, gamma_c);
        float u[2];
        if(pos - cell < 0.5)
        {
            pos = pos - cell + 0.5;
            u[1] = cell_u;
            if(cell > 0)
                u[0] = to_u(data[cell-1].rho, data[cell-1].y, speedlimit, gamma_c);
            else
            {
                lane *prev;
                if((prev = upstream_lane()))
                    u[0] = to_u(prev->data[prev->ncells-1].rho, prev->data[prev->ncells-1].y, prev->speedlimit, gamma_c);
                else
                    u[0] = u[1];
            }
        }
        else
        {
            pos = pos - cell - 0.5;
            u[0] = cell_u;
            if(cell < static_cast<int>(ncells-1))
                u[1] = to_u(data[cell+1].rho, data[cell+1].y, speedlimit, gamma_c);
            else
            {
                lane *next;
                if((next = downstream_lane()))
                    u[1] = to_u(next->data[0].rho, next->data[0].y, next->speedlimit, gamma_c);
                else
                    u[1] = u[0];
            }
        }

        carticles[0][i].x += dt*(u[0]*pos + u[1]*(1.0f-pos))*inv_len;

        if(carticles[0][i].x > 1.0)
        {
            lane *next;
            if(end.end_type == lane_end::INTERSECTION)
            {
                if((next = downstream_lane()))
                {
                    carticles[0][i].x = (carticles[0][i].x - 1.0) * ncells*h/(next->ncells*next->h);
                    next->carticles[1].push_back(carticles[0][i]);
                }
                else
                {
                    carticles[0][i].x = 1.0f;
                    carticles[1].push_back(carticles[0][i]);
                }
            }
        }
        else
            carticles[1].push_back(carticles[0][i]);
    }
}

void lane::swap_carticles()
{
    carticles[0].swap(carticles[1]);
    carticles[1].clear();
}

