#include "hybrid-sim.hpp"

namespace hybrid
{
    void lane::macro_initialize(const float h_suggest)
    {
        N = static_cast<size_t>(std::ceil(length/h_suggest));
        assert(N > 0);
        h = length/N;
        inv_h = 1.0f/h;
    }

    int lane::which_cell(const float pos) const
    {
        return static_cast<int>(std::floor(pos*N));
    }

    float lane::collect_riemann(const float gamma, const float inv_gamma)
    {
        arz<float>::full_q  full_q_buff[2];
        arz<float>::full_q *fq[2] = { full_q_buff, full_q_buff + 1 };

        *fq[0] = arz<float>::full_q(q[0],
                                    parent->speedlimit,
                                    gamma);

        float maxspeed = 0.0f;
        const hwm::lane *upstream = parent->upstream_lane();
        if(!upstream)
        {
            rs[0].starvation_riemann(*fq[0],
                                     parent->speedlimit,
                                     1.0f/parent->speedlimit,
                                     gamma,
                                     inv_gamma);
            maxspeed = std::max(rs[0].speeds[1], maxspeed);
        }
        else
        {
            const lane &sim_upstream = *(upstream->user_data<lane>());
            rs[0].riemann(arz<float>::full_q(sim_upstream.q[sim_upstream.N-1],
                                             upstream->speedlimit,
                                             gamma),
                          *fq[0],
                          parent->speedlimit,
                          1.0f/parent->speedlimit,
                          gamma,
                          inv_gamma);
            maxspeed = std::max(std::max(std::abs(rs[0].speeds[0]), std::abs(rs[0].speeds[1])),
                                maxspeed);
        }

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

        const hwm::lane *downstream = parent->downstream_lane();
        if(!downstream)
        {
            rs[N].stop_riemann(*fq[0],
                               parent->speedlimit,
                               1.0f/parent->speedlimit,
                               gamma,
                               inv_gamma);
            maxspeed = std::max(std::abs(rs[N].speeds[0]), maxspeed);
        }
        else
        {
            const lane &sim_downstream = *(downstream->user_data<lane>());

            rs[N].riemann(*fq[0],
                          arz<float>::full_q(sim_downstream.q[0],
                                             downstream->speedlimit,
                                             gamma),
                          parent->speedlimit,
                          1.0f/parent->speedlimit,
                          gamma,
                          inv_gamma);
            maxspeed = std::max(std::max(std::abs(rs[N].speeds[0]), std::abs(rs[N].speeds[1])),
                                maxspeed);
        }

        assert(rs[N].check());

        return maxspeed;
    }

    void lane::update(const float dt, const float relaxation_factor)
    {
        const float coefficient = dt*inv_h;

        for(size_t i = 0; i < N; ++i)
        {
            q[i]     -= coefficient*(rs[i].right_fluctuation + rs[i+1].left_fluctuation);
            q[i].y() -= q[i].y()*coefficient*relaxation_factor;
            q[i].fix();

        }
    }

    void lane::clear_macro()
    {
        memset(q, 0, sizeof(arz<float>::q)*N);
    }

    void lane::convert_cars(const simulator &sim)
    {
        BOOST_FOREACH(car &c, current_cars())
        {
            const float car_front = c.position+sim.rear_bumper_offset()*inv_length;
            const float car_back  = c.position+sim.front_bumper_offset()*inv_length;

            const int front_cell = which_cell(car_front);
            const int back_cell  = which_cell(car_back);

            if(front_cell >= 0)
            {
                float front_coverage = car_front*N - front_cell;
                q[front_cell].rho() += front_coverage;
                q[front_cell].y()   += front_coverage*c.velocity;
            }

            for(int i = front_cell+1; i < back_cell-1; ++i)
            {
                q[i].rho() = 1.0f;
                q[i].y()   = c.velocity;
            }

            if(back_cell < N)
            {
                float back_coverage  = (back_cell+1) - car_back*N;
                q[back_cell].rho()  += back_coverage;
                q[back_cell].y()    += back_coverage*c.velocity;
            }
        }
    }

    void lane::fill_y(const float gamma)
    {
        for(size_t i = 0; i < N; ++i)
        {
            if(q[i].rho() > 0.0)
                q[i].y() /= q[i].rho();
            q[i].y() = arz<float>::eq::y(q[i].rho(),
                                         q[i].y(),
                                         parent->speedlimit,
                                         gamma);
            q[i].fix();
        }
    }

    void simulator::macro_initialize(const float in_gamma, const float h_suggest, const float rf)
    {
        gamma             = in_gamma;
        relaxation_factor = rf;
        min_h             = std::numeric_limits<float>::max();

        // initialize new lanes, compute how many cells to allocate
        size_t cell_count = 0;
        BOOST_FOREACH(lane &l, lanes)
        {
            l.macro_initialize(h_suggest);
            cell_count += l.N;
            min_h = std::min(min_h, l.h);
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

        memset(q_base, 0, sizeof(arz<float>::q)*cell_count);
        memset(rs_base, 0, sizeof(arz<float>::riemann_solution)*(cell_count+lanes.size()));

        size_t q_count  = 0;
        size_t rs_count = 0;
        BOOST_FOREACH(lane &l, lanes)
        {
            l.q       = q_base + q_count;
            l.rs      = rs_base + rs_count;
            q_count  += l.N;
            rs_count += l.N + 1;

            if(l.N > 5)
            {
                l.q[2].rho() = 0.5;
                l.q[2].y()   = 1.5;
            }
            l.fill_y(gamma);
        }
        std::cout << "min_h is " << min_h << std::endl;
    }

    void simulator::macro_cleanup()
    {
        free(q_base);
        free(rs_base);
    }

    float simulator::macro_step(const float cfl)
    {
        float maxspeed = 0.0f;
        BOOST_FOREACH(lane *l, macro_lanes)
        {
            if(l->parent->active)
                maxspeed = std::max(l->collect_riemann(gamma, 1.0f/gamma),
                                    maxspeed);
        }

        if(maxspeed < arz<float>::epsilon())
            maxspeed = min_h;

        const float dt = cfl*min_h/maxspeed;

        BOOST_FOREACH(lane *l, macro_lanes)
        {
            if(l->parent->active)
                l->update(dt, relaxation_factor);
        }

        return dt;
    }
};
