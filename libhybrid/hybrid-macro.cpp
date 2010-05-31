#include "libhybrid/hybrid-sim.hpp"

#include "xmmintrin.h"
#include "pmmintrin.h"

namespace hybrid
{
    void lane::macro_initialize(const float h_suggest)
    {
        N = static_cast<size_t>(std::ceil(length/h_suggest));
        assert(N > 0);
        h = length/N;
        inv_h = 1.0f/h;
    }

    struct lane_poisson_helper
    {
        typedef float real_t;

        lane_poisson_helper(const lane &l, const float scale) : dx_(l.h), n_(l.N), q_(l.q), scale_(scale)
        {}

        float operator[](size_t idx) const
        {
            return q_[idx].rho()*scale_;
        }

        float dx() const
        {
            return dx_;
        }

        float inf() const
        {
            return 0;
        }

        float end() const
        {
            return n()*dx();
        }

        size_t n() const
        {
            return n_;
        }

        float          dx_;
        size_t         n_;
        arz<float>::q *q_;
        float          scale_;
    };

    struct lane_poisson_helper_reverse
    {
        typedef float real_t;

        lane_poisson_helper_reverse(const lane &l, const float scale) : dx_(l.h), n_(l.N), q_(l.q), scale_(scale)
        {}

        float operator[](size_t idx) const
        {
            return q_[n_-1-idx].rho()*scale_;
        }

        float dx() const
        {
            return dx_;
        }

        float inf() const
        {
            return 0;
        }

        float end() const
        {
            return n()*dx();
        }

        size_t n() const
        {
            return n_;
        }

        float          dx_;
        size_t         n_;
        arz<float>::q *q_;
        float          scale_;
    };

    void lane::macro_instantiate(simulator &sim)
    {
        typedef pproc::inhomogeneous_poisson<simulator::rand_gen_t, lane_poisson_helper> ih_poisson_t;

        current_cars().clear();

        lane_poisson_helper helper(*this, 1.0f/sim.car_length);
        ih_poisson_t        ip(-sim.rear_bumper_offset(), helper, sim.uni);

        float candidate = ip.next();

        while(candidate < helper.end()-sim.front_bumper_offset())
        {
            const float position = candidate/helper.end();
            current_cars().push_back(sim.make_car(position,
                                                  velocity(position),
                                                  0.0f));

            while(1)
            {
                ih_poisson_t ip_copy(ip);
                candidate = ip.next();
                if(candidate - current_cars().back().position*helper.end() >= sim.car_length)
                    break;
                ip = ip_copy;
            }
        }
    }

    bool lane::macro_find_first(float &param, const simulator &sim) const
    {
        typedef pproc::inhomogeneous_poisson<simulator::rand_gen_t, lane_poisson_helper> ih_poisson_t;

        lane_poisson_helper helper(*this, 1.0f/sim.car_length);
        ih_poisson_t        ip(-sim.rear_bumper_offset(), helper, sim.uni);

        const float candidate = ip.next();
        if(candidate < helper.end()-sim.front_bumper_offset())
        {
            param = candidate/helper.end();
            return true;
        }
        else
            return false;
    }

    bool lane::macro_find_last(float &param, const simulator &sim) const
    {
        typedef pproc::inhomogeneous_poisson<simulator::rand_gen_t, lane_poisson_helper_reverse> ih_poisson_t;

        lane_poisson_helper_reverse helper(*this, 1.0f/sim.car_length);
        ih_poisson_t                ip(sim.front_bumper_offset(), helper, sim.uni);

        const float candidate = helper.end() - ip.next();
        if(candidate + sim.rear_bumper_offset() > 0)
        {
            param = candidate/helper.end();
            return true;
        }
        else
            return false;
    }

    void lane::macro_distance_to_car(float &distance, float &vel, const float distance_max, const simulator &sim) const
    {
        float param;
        if(!fictitious && macro_find_first(param, sim))
        {
            distance += param*length;
            vel       = velocity(param);
        }
        else
        {
            distance += length;
            if(distance >= distance_max || parent->end->network_boundary())
            {
                vel      = 0.0f;
                distance = distance_max;
            }
            else
            {
                const lane *downstream = downstream_lane();
                if(downstream)
                    downstream->distance_to_car(distance, vel, distance_max, sim);
                else
                    vel = 0.0f;
            }
        }

        return;
    }

    int lane::which_cell(const float pos) const
    {
        return static_cast<int>(std::floor(pos*N));
    }

    float lane::velocity(const float pos) const
    {
        const int   cell  = which_cell(pos);
        const float local = pos*N - cell;
        assert(cell >= 0);
        assert(cell < static_cast<int>(N));

        const arz<float>::full_q q_c(q[cell], speedlimit());
        if(local < 0.5f)
        {
            if(cell == 0)
                return q_c.u();

            const arz<float>::full_q q_c_m(q[cell-1], speedlimit());
            return (local+0.5f) * q_c.u() + (0.5f-local) * q_c_m.u();
        }
        //local >= 0.5f

        if(cell == static_cast<int>(N)-1)
            return q_c.u();

        const arz<float>::full_q q_c_p(q[cell+1], speedlimit());
        return (1.5f-local) * q_c.u() + (local-0.5f) * q_c_p.u();
    }

    float lane::collect_riemann()
    {
        arz<float>::full_q  full_q_buff[2];
        arz<float>::full_q *fq[2] = { full_q_buff, full_q_buff + 1 };

        const float my_speedlimit  = speedlimit();
        const float inv_speedlimit = 1.0f/my_speedlimit;

        *fq[0] = arz<float>::full_q(q[0],
                                    my_speedlimit);

        float maxspeed = 0.0f;
        lane *upstream = upstream_lane();
        if(!upstream)
        {
            rs[0].starvation_riemann(*fq[0],
                                     my_speedlimit,
                                     inv_speedlimit);
            maxspeed = std::max(rs[0].speeds[1], maxspeed);
        }
        else
        {
            const arz<float>::full_q us_end(*up_aux,
                                            my_speedlimit);

            if(upstream->speedlimit() == my_speedlimit)
                rs[0].riemann(us_end,
                              *fq[0],
                              my_speedlimit,
                              inv_speedlimit);
            else
                rs[0].lebaque_inhomogeneous_riemann(us_end,
                                                    *fq[0],
                                                    upstream->speedlimit(),
                                                    my_speedlimit);

            maxspeed = std::max(std::max(std::abs(rs[0].speeds[0]), std::abs(rs[0].speeds[1])),
                                maxspeed);
        }

        assert(rs[0].check());

        for(size_t i = 1; i < N; ++i)
        {
            *fq[1] = arz<float>::full_q(q[i],
                                        my_speedlimit);

            rs[i].riemann(*fq[0],
                          *fq[1],
                          my_speedlimit,
                          inv_speedlimit);

            assert(rs[i].check());

            maxspeed = std::max(std::max(std::abs(rs[i].speeds[0]), std::abs(rs[i].speeds[1])),
                                maxspeed);
            std::swap(fq[0], fq[1]);
        }

        if(parent->end->network_boundary())
        {
            rs[N].clear();
        }
        else
        {
            lane *downstream = downstream_lane();
            if(!downstream)
            {
                rs[N].stop_riemann(*fq[0],
                                   my_speedlimit,
                                   inv_speedlimit);
                maxspeed = std::max(std::abs(rs[N].speeds[0]), maxspeed);
            }
            else
            {
                const arz<float>::full_q ds_start(*down_aux,
                                                  downstream->speedlimit());

                if(my_speedlimit == downstream->speedlimit())
                    rs[N].riemann(*fq[0],
                                  ds_start,
                                  my_speedlimit,
                                  inv_speedlimit);
                else
                    rs[N].lebaque_inhomogeneous_riemann(*fq[0],
                                                        ds_start,
                                                        my_speedlimit,
                                                        downstream->speedlimit());

                maxspeed = std::max(std::max(std::abs(rs[N].speeds[0]), std::abs(rs[N].speeds[1])),
                                    maxspeed);
            }
        }

        assert(rs[N].check());

        return maxspeed;
    }

    void lane::update(const float dt, simulator &sim)
    {
        const float coefficient = dt*inv_h;

        for(size_t i = 0; i < N; ++i)
        {
            q[i]     -= coefficient*(rs[i].right_fluctuation + rs[i+1].left_fluctuation);
            q[i].y() -= q[i].y()*coefficient*sim.relaxation_factor;
            q[i].fix();
        }

        lane *upstream = upstream_lane();
        if(upstream && upstream->is_macro())
        {
            if(upstream->fictitious)
            {
                lane* next_upstream = upstream->upstream_lane();
                assert(next_upstream);
                assert(!next_upstream->fictitious);
                upstream = next_upstream;
            }
            if(upstream->is_macro())
                *upstream->down_aux = q[N-1];
        }

        lane *downstream = downstream_lane();
        if(downstream)
        {
            if(downstream->is_macro())
            {
                if(downstream->fictitious)
                {
                    lane* next_downstream = downstream->downstream_lane();
                    assert(next_downstream);
                    assert(!next_downstream->fictitious);
                    downstream = next_downstream;
                }
                if(downstream->is_macro())
                    *downstream->up_aux = q[0];
            }
            else
            {
                float param;
                if(macro_find_last(param, sim))
                {
                    car c(0, param, velocity(param), 0.0f);
                    c.compute_intersection_acceleration(sim, *this);
                    c.integrate(dt, *this, sim.hnet->lane_width);

                    const int cell(std::min(which_cell(c.position), static_cast<int>(N)-1));
                    assert(cell >=0);
                    q[cell] = arz<float>::q(q[cell].rho(), c.velocity, speedlimit());
                    if(c.position >= 1.0)
                    {
                        c.position = length*downstream->inv_length*(c.position-1.0f);
                        downstream->next_cars().push_back(sim.make_car(c.position, c.velocity, c.acceleration));
                    }
                }
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

    void lane::fill_y()
    {
        for(size_t i = 0; i < N; ++i)
        {
            if(q[i].rho() > 0.0)
                q[i].y() /= q[i].rho();
            q[i].y() = arz<float>::eq::y(q[i].rho(),
                                         q[i].y(),
                                         speedlimit());
            q[i].fix();
        }
    }

    void worker::macro_initialize()
    {
        std::cout << "Allocating " << sizeof(arz<float>::q)*N <<  " bytes for " << N << " cells...";
        q_base = (arz<float>::q *) xmalloc(sizeof(arz<float>::q)*N);
        if(!q_base)
            throw std::exception();
        std::cout << "Done." << std::endl;

        std::cout << "Allocating " << sizeof(arz<float>::riemann_solution)*(N+macro_lanes.size()) <<  " bytes for " << N+macro_lanes.size() << " riemann solutions...";
        rs_base = (arz<float>::riemann_solution *) xmalloc(sizeof(arz<float>::riemann_solution)*(N+macro_lanes.size()));
        if(!rs_base)
            throw std::exception();
        std::cout << "Done." << std::endl;

        memset(q_base, 0, sizeof(arz<float>::q)*N);
        memset(rs_base, 0, sizeof(arz<float>::riemann_solution)*(N+macro_lanes.size()));

        size_t q_count   = 0;
        size_t aux_count = 0;
        size_t rs_count  = 0;
        BOOST_FOREACH(lane *l, macro_lanes)
        {
            l->q         = q_base + q_count;
            l->up_aux    = (arz<float>::q *) xmalloc(sizeof(arz<float>::q));
            std::memset(l->up_aux, 0,sizeof(arz<float>::q));
            l->down_aux  = (arz<float>::q *) xmalloc(sizeof(arz<float>::q));
            std::memset(l->down_aux, 0,sizeof(arz<float>::q));
            l->rs        = rs_base + rs_count;
            q_count     += l->N;
            aux_count   += 2;
            rs_count    += l->N + 1;

            l->fill_y();
        }
    }

    void simulator::macro_initialize(const float h_suggest, const float rf)
    {
        const size_t max_thr = omp_get_max_threads();
        assert(workers.size() == max_thr);

        // Try to stay out of FP_ASSIST - enable DAZ and FTZ
        _MM_SET_FLUSH_ZERO_MODE(_MM_FLUSH_ZERO_ON);
        _MM_SET_DENORMALS_ZERO_MODE(_MM_DENORMALS_ZERO_ON);

        relaxation_factor = rf;
        min_h             = std::numeric_limits<float>::max();

        // initialize new lanes, compute how many cells to allocate
        BOOST_FOREACH(lane &l, lanes)
        {
            if(l.fictitious)
                continue;
            l.macro_initialize(h_suggest);

            int worker_no = 0;
            for(size_t i = 1; i < workers.size(); ++i)
                if(workers[i].N < workers[worker_no].N)
                    worker_no = i;

            workers[worker_no].N += l.N;
            min_h                = std::min(min_h, l.h);
            workers[worker_no].macro_lanes.push_back(&l);
        }

        std::cout << "min_h is " << min_h << std::endl;

        int worker_no = 0;
        BOOST_FOREACH(worker &w, workers)
        {
            w.macro_initialize();
            std::cout << "Worker " << worker_no << " has " << w.N << " cells " << std::endl;
            ++worker_no;
        }

        maxes = (float*)xmalloc(max_thr*MAXES_STRIDE*sizeof(float));
    }

    void simulator::macro_cleanup()
    {
        free(maxes);
    }

    void simulator::convert_cars(const sim_t sim_mask)
    {
        std::vector<lane*> &thelanes = sim_mask == MICRO ? micro_lanes : macro_lanes;
        BOOST_FOREACH(lane *l, thelanes)
        {
            assert(l->sim_type & sim_mask);
            if(!l->fictitious)
                l->clear_macro();
        }
        BOOST_FOREACH(lane *l, thelanes)
        {
            assert(l->sim_type & sim_mask);
            if(!l->fictitious)
                l->convert_cars(*this);
        }
        BOOST_FOREACH(lane *l, thelanes)
        {
            assert(l->sim_type & sim_mask);
            if(!l->fictitious)
                l->fill_y();
        }
    }

    float simulator::macro_step(const float cfl)
    {
        const size_t max_thr   = omp_get_max_threads();
        assert(max_thr == workers.size());
        const int    num_procs = omp_get_num_procs();
        float        maxspeed  = 0.0f;
        float        dt;
#pragma omp parallel
        {
            const int thr_id           = omp_get_thread_num();
            maxes[thr_id*MAXES_STRIDE] = 0.0f;

            cpu_set_t mask;
            CPU_ZERO(&mask);
            CPU_SET((thr_id % num_procs), &mask);

            if(sched_setaffinity(0, sizeof(mask), &mask) == -1)
                fprintf(stderr, "Couldn't set affinity for thread %d\n", thr_id);

            struct sched_param sp;
            sp.sched_priority = 10;
            sched_setscheduler(0, SCHED_FIFO, &sp);

            worker &work = workers[thr_id];
            for(size_t i = 0; i < work.macro_lanes.size(); ++i)
            {
                lane        *l             = macro_lanes[i];
                assert(l->is_macro());
                assert(l->active());
                assert(!l->fictitious);
                const float  max           = l->collect_riemann();
                maxes[thr_id*MAXES_STRIDE] = std::max(max, maxes[thr_id*MAXES_STRIDE]);
            }

#pragma omp barrier
#pragma omp single
            {
                maxspeed = 0.0f;
                for(size_t t = 0; t < max_thr; ++t)
                    maxspeed = std::max(maxspeed, maxes[t*MAXES_STRIDE]);

                if(maxspeed < arz<float>::epsilon())
                    maxspeed = min_h;

                dt = std::min(cfl*min_h/maxspeed, 0.5f);
            }

            for(size_t i = 0; i < work.macro_lanes.size(); ++i)
            {
                lane *l = macro_lanes[i];
                assert(l->is_macro());
                assert(l->active());
                assert(!l->fictitious);
                l->update(dt, *this);
            }
        }

        return dt;
    }
};
