#include "macro-sim.hpp"

namespace macro
{
    simulator::simulator(hwm::network *net, const float g, const float h, const float r) : q_base(0), rs_base(0), time(0),
                                                                                           gamma(g), h_suggest(h), min_h(std::numeric_limits<float>::max()),
                                                                                           relaxation_factor(r), hnet(net)
    {}

    simulator::~simulator()
    {
        free(q_base);
        free(rs_base);
    }

    void simulator::initialize()
    {
        assert(hnet);

        // figure out how many lanes to create
        size_t lane_count = hnet->lanes.size();
        BOOST_FOREACH(const hwm::intersection_pair &ip, hnet->intersections)
        {
            BOOST_FOREACH(const hwm::intersection::state &current_state, ip.second.states)
            {
                lane_count += current_state.fict_lanes.size();
            }
        }

        // create them
        lanes.resize(lane_count);

        // initialize new lanes, compute how many cells to allocate
        size_t                      cell_count  = 0;
        std::vector<lane>::iterator current     = lanes.begin();
        for(hwm::lane_map::iterator hwm_current = hnet->lanes.begin();
            hwm_current != hnet->lanes.end() && current != lanes.end();
            ++current, ++hwm_current)
        {
            current->initialize(&(hwm_current->second), h_suggest);
            cell_count += current->N;
            min_h = std::min(min_h, current->h);
        }

        BOOST_FOREACH(hwm::intersection_pair &ip, hnet->intersections)
        {
            BOOST_FOREACH(hwm::intersection::state &current_state, ip.second.states)
            {
                BOOST_FOREACH(hwm::lane_pair &lp, current_state.fict_lanes)
                {
                    assert(current != lanes.end());
                    current->initialize(&(lp.second), h_suggest);
                    cell_count += current->N;
                    min_h = std::min(min_h, current->h);
                    ++current;
                }
            }
        }

        std::cout << "Allocating " << sizeof(arz<float>::q)*cell_count <<  " bytes for " << cell_count << " cells...";
        q_base = (arz<float>::q *) malloc(sizeof(arz<float>::q)*cell_count);
        if(!q_base)
            throw std::exception();
        std::cout << "Done." << std::endl;

        std::cout << "Allocating " << sizeof(arz<float>::riemann_solution)*(cell_count+lanes.size()) <<  " bytes for " << cell_count+lanes.size() << " riemann solutions...";
        rs_base = (arz<float>::riemann_solution *) malloc(sizeof(arz<float>::riemann_solution)*(cell_count+lanes.size()));
        if(!rs_base)
            throw std::exception();
        std::cout << "Done." << std::endl;

        size_t q_count  = 0;
        size_t rs_count = 0;
        BOOST_FOREACH(lane &l, lanes)
        {
            l.q       = q_base + q_count;
            l.rs      = rs_base + rs_count;
            q_count  += l.N;
            rs_count += l.N + 1;
        }
        std::cout << "min_h is " << min_h << std::endl;

        memset(q_base, 0, sizeof(arz<float>::q)*cell_count);
        memset(rs_base, 0, sizeof(arz<float>::riemann_solution)*(cell_count+lanes.size()));
    }

    lane::lane() : parent(0), length(0), inv_length(0), h(0), N(0), q(0), rs(0)
    {}

    void lane::initialize(hwm::lane *in_parent, const float h_suggest)
    {
        parent                = in_parent;
        in_parent->user_datum = this;
        length                = parent->length();
        inv_length            = 1.0/length;
        N = static_cast<size_t>(std::ceil(length/h_suggest));
        assert(N > 0);
        h = length/N;
    }

    float simulator::step(const float cfl)
    {
        float maxspeed = 0.0f;
        BOOST_FOREACH(lane &l, lanes)
        {
            if(l.parent->active)
                maxspeed = std::max(l.collect_riemann(gamma, 1.0f/gamma),
                                    maxspeed);
        }

        if(maxspeed < arz<float>::epsilon())
            maxspeed = min_h;

        const float dt = cfl*min_h/maxspeed;

        BOOST_FOREACH(lane &l, lanes)
        {
            if(l.parent->active)
                l.update(dt, relaxation_factor);
        }

        time += dt;
        return dt;
    }

    float lane::collect_riemann(const float gamma, const float inv_gamma)
    {
        arz<float>::full_q  full_q_buff[2];
        arz<float>::full_q *fq[2] = { full_q_buff, full_q_buff + 1 };

        *fq[0] = arz<float>::full_q(q[0],
                                    parent->speedlimit,
                                    gamma);

        float maxspeed = 0.0f;
        if(!parent->upstream_lane())
        {
            rs[0].starvation_riemann(*fq[0],
                                     parent->speedlimit,
                                     1.0f/parent->speedlimit,
                                     gamma,
                                     inv_gamma);
            maxspeed = std::max(rs[0].speeds[1], maxspeed);
        }
        else
            rs[0].clear();

        assert(rs[0].check());

        for(size_t i = 1; i < N; ++i)
        {
            *fq[1] = arz<float>::full_q(q[i],
                                        parent->speedlimit,
                                        gamma);

            rs[i].riemann(*fq[0],
                          *fq[1],
                          parent->speedlimit,
                          1.0f/parent->speedlimit,
                          gamma,
                          inv_gamma);

            assert(rs[i].check());

            maxspeed = std::max(std::max(std::abs(rs[i].speeds[0]), std::abs(rs[i].speeds[1])),
                                maxspeed);
            std::swap(fq[0], fq[1]);
        }

        if(!parent->downstream_lane())
        {
            rs[N].stop_riemann(*fq[0],
                               parent->speedlimit,
                               1.0f/parent->speedlimit,
                               gamma,
                               inv_gamma);
            maxspeed = std::max(std::abs(rs[N].speeds[0]), maxspeed);
        }
        else
            rs[N].clear();

        assert(rs[N].check());

        return maxspeed;
    }

    void lane::update(const float dt, const float relaxation_factor)
    {
        const float coefficient = dt/h;

        for(size_t i = 0; i < N; ++i)
        {
            q[i]     -= coefficient*(rs[i].right_fluctuation + rs[i+1].left_fluctuation);
            q[i].y() -= q[i].y()*coefficient*relaxation_factor;
            q[i].fix();
        }
    }
};
