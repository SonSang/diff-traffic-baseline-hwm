#include "libhybrid/hybrid-sim.hpp"
#include "libhybrid/timer.hpp"

namespace hybrid
{
    void simulator::parallel_hybrid_run(int nsteps)
    {
        const float cfl      = 1.0f;
        const int   max_thr  = omp_get_max_threads();
        float       maxspeed = 0.0f;
        float       dt;

        float convert_time     = 0.0f;
        float riemann_time     = 0.0f;
        float max_compute_time = 0.0f;
        float update_time      = 0.0f;
        float micro_time       = 0.0f;

        timer step_timer;
#pragma omp parallel
        {
            const int thr_id = omp_get_thread_num();

            for(int i = 0; i < nsteps; ++i)
            {
                // fill in micro
#pragma omp barrier
#pragma omp single
                {
                    step_timer.reset();
                    step_timer.start();

                    convert_cars(MICRO);

                    step_timer.stop();
                    convert_time += step_timer.interval_S();
                }

                // macro step (also emit cars)
                {
#pragma omp barrier
#pragma omp single
                    {
                        step_timer.reset();
                        step_timer.start();
                    }

                    maxes[thr_id*MAXES_STRIDE] = 0.0f;
#pragma omp for
                    for(size_t i = 0; i < macro_lanes.size(); ++i)
                    {
                        lane *l = macro_lanes[i];
                        assert(l->is_macro());
                        assert(l->active());
                        assert(!l->fictitious);
                        const float max = l->collect_riemann();
                        maxes[thr_id*MAXES_STRIDE] = std::max(max, maxes[thr_id*MAXES_STRIDE]);
                    }

#pragma omp barrier
#pragma omp single
                    {
                        step_timer.stop();
                        riemann_time += step_timer.interval_S();
                    }

#pragma omp barrier
#pragma omp single
                    {
                        step_timer.reset();
                        step_timer.start();

                        maxspeed = 0.0f;
                        for(int t = 0; t < max_thr; ++t)
                            maxspeed = std::max(maxspeed, maxes[t*MAXES_STRIDE]);

                        if(maxspeed < arz<float>::epsilon())
                            maxspeed = min_h;

                        dt = std::min(cfl*min_h/maxspeed, 0.5f);

                        step_timer.stop();
                        max_compute_time += step_timer.interval_S();
                    }

#pragma omp barrier
#pragma omp single
                    {
                        step_timer.reset();
                        step_timer.start();
                    }

#pragma omp for
                    for(size_t i = 0; i < macro_lanes.size(); ++i)
                    {
                        lane *l = macro_lanes[i];
                        assert(l->is_macro());
                        assert(l->active());
                        assert(!l->fictitious);
                        l->update(dt, *this);
                    }

#pragma omp barrier
#pragma omp single
                    {
                        step_timer.stop();
                        update_time += step_timer.interval_S();
                    }
                }

#pragma omp barrier
#pragma omp single
                {
                    step_timer.reset();
                    step_timer.start();

                    // micro step
                    update(dt);

                    time += dt;
                    apply_incoming_bc(dt, time);

                    car_swap();
                    advance_intersections(dt);

                    step_timer.stop();
                    micro_time += step_timer.interval_S();
                }
            }
        }
        std::cout << "Convert   time: " << convert_time << std::endl
                  << "Riemann   time: " << riemann_time << std::endl
                  << "Max comp. time: " << max_compute_time << std::endl
                  << "Update    time: " << update_time << std::endl
                  << "Micro     time: " << micro_time << std::endl;
    }
}
