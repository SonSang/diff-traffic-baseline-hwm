#include "hybrid-sim.hpp"

namespace hybrid
{
    flux_capacitor::flux_capacitor() : rho_accum(0.0), u_accum(0.0)
    {}

    void flux_capacitor::update(const arz<float>::riemann_solution &rs, const float dt)
    {
        const float delta_rho  = dt*rs.q_0.rho();
        rho_accum             += delta_rho;
        u_accum               += delta_rho*rs.speeds[1];
    }

    bool flux_capacitor::check() const
    {
        return rho_accum >= 1.0f-arz<float>::epsilon();
    }

    car flux_capacitor::emit()
    {
        rho_accum    -= 1.0f;
        car res;
        res.position  = 0.0f;
        res.velocity  = u_accum;
        u_accum      *= rho_accum;
        return res;
    }

    void lane::macro_initialize(const float h_suggest)
    {
        N = static_cast<size_t>(std::ceil(length/h_suggest));
        assert(N > 0);
        h = length/N;
        inv_h = 1.0f/h;
    }

    void lane::macro_distance_to_car(float &distance, float &velocity, const float distance_max, const simulator &sim) const
    {
        float rho_accum = 0.0f;
        float u_accum = 0.0f;

        int i = 0;
        int start_cell = -1;
        while(i < static_cast<int>(N) && rho_accum < 1.0)
        {
            if(q[i].rho() > arz<float>::epsilon() && start_cell == -1)
            {
                start_cell = i;
                rho_accum += q[i].rho();
                float u    = arz<float>::eq::u(q[i].rho(), q[i].y(), parent->speedlimit, sim.gamma);
                u_accum   += q[i].rho()*u;
            }
            ++i;
        }

        if(rho_accum + arz<float>::epsilon() >= 1.0f)
        {
            distance += i*h;
            velocity = u_accum;
        }
        else
        {
            distance += length;
            if(distance >= distance_max)
            {
                velocity = 0.0f;
                distance = distance_max;
            }
            else
            {
                hwm::lane *hwm_downstream = parent->downstream_lane();
                if(hwm_downstream)
                    hwm_downstream->user_data<lane>()->distance_to_car(distance, velocity, distance_max, sim);
                else
                    velocity = 0.0f;
            }
        }

        return;
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
            rs[0].inhomogeneous_riemann(arz<float>::full_q(sim_upstream.q[sim_upstream.N-1],
                                                           upstream->speedlimit,
                                                           gamma),
                                        *fq[0],
                                        upstream->speedlimit,
                                        parent->speedlimit,
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

            rs[N].inhomogeneous_riemann(*fq[0],
                                        arz<float>::full_q(sim_downstream.q[0],
                                                           downstream->speedlimit,
                                                           gamma),
                                        parent->speedlimit,
                                        downstream->speedlimit,
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
        hwm::lane *downstream = parent->downstream_lane();
        if(downstream)
        {
            capacitor.update(rs[N], dt);
            if(capacitor.check())
            {
                lane &sim_downstream = *(downstream->user_data<lane>());
                sim_downstream.next_cars().push_back(capacitor.emit());
            }
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
            const float car_back  = c.position+sim.rear_bumper_offset()*inv_length;
            const float car_front = c.position+sim.front_bumper_offset()*inv_length;

            const int start_cell = which_cell(car_back);
            const int end_cell   = which_cell(car_front);

            if(start_cell == end_cell)
            {
                float coverage       = sim.car_length*inv_h;
                q[start_cell].rho() += coverage;
                q[start_cell].y()   += coverage*c.velocity;
                continue;
            }

            if(start_cell >= 0)
            {
                float start_coverage = (start_cell+1) - car_back*N;
                q[start_cell].rho() += start_coverage;
                q[start_cell].y()   += start_coverage*c.velocity;
            }

            for(int i = start_cell+1; i < end_cell-1; ++i)
            {
                q[i].rho() = 1.0f;
                q[i].y()   = c.velocity;
            }

            if(end_cell < static_cast<int>(N))
            {
                float end_coverage  = car_front*N - end_cell;
                q[end_cell].rho()  += end_coverage;
                assert(q[end_cell].rho() <= 1.0f);
                q[end_cell].y()    += end_coverage*c.velocity;
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

    void simulator::convert_cars(const sim_t sim_mask)
    {
        BOOST_FOREACH(lane &l, lanes)
        {
            if(l.sim_type & sim_mask)
                l.clear_macro();
        }
        BOOST_FOREACH(lane &l, lanes)
        {
            if(l.sim_type & sim_mask)
                l.convert_cars(*this);
        }
        BOOST_FOREACH(lane &l, lanes)
        {
            if(l.sim_type & sim_mask)
                l.fill_y(gamma);
        }
    }

    float simulator::macro_step(const float cfl)
    {
        float maxspeed = 0.0f;
        BOOST_FOREACH(lane &l, lanes)
        {
            if(l.is_macro() && l.parent->active)
                maxspeed = std::max(l.collect_riemann(gamma, 1.0f/gamma),
                                    maxspeed);
        }

        if(maxspeed < arz<float>::epsilon())
            maxspeed = min_h;

        const float dt = cfl*min_h/maxspeed;

        BOOST_FOREACH(lane &l, lanes)
        {
            if(l.is_macro() && l.parent->active)
                l.update(dt, relaxation_factor);
        }

        return dt;
    }
};
