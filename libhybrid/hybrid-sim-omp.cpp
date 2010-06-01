#include "libhybrid/hybrid-sim.hpp"
#include "libhybrid/timer.hpp"

#ifdef _MSC_VER
#include <windows.h>
#undef max
#undef min
#endif

namespace hybrid
{
    void simulator::parallel_hybrid_run(int nsteps)
    {
        const float  cfl       = 1.0f;
        const size_t max_thr   = omp_get_max_threads();
        const size_t num_procs = omp_get_num_procs();
        float        maxspeed;
        float        dt;

        float convert_time     = 0.0f;
        float riemann_time     = 0.0f;
        float max_compute_time = 0.0f;
        float update_time      = 0.0f;
        float micro_time       = 0.0f;

        timer step_timer;
        timer overall_timer;
#pragma omp parallel
        {
            const size_t thr_id = omp_get_thread_num();

            worker &work = workers[thr_id];

#ifdef _MSC_VER
            DWORD_PTR mask = (1 << (thr_id % num_procs));
            if(SetThreadAffinityMask( GetCurrentThread(), mask) == 0)
                fprintf(stderr, "Couldn't set affinity for thread %d\n", thr_id);

            if(SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_TIME_CRITICAL) == 0)
                fprintf(stderr, "Couldn't set realtime priority for thread %d\n", thr_id);
#else
            cpu_set_t mask;
            CPU_ZERO(&mask);
            CPU_SET((thr_id % num_procs), &mask);

            if(sched_setaffinity(0, sizeof(mask), &mask) == -1)
                std::cerr << "Couldn't set affinity for thread: " <<  thr_id << std::endl;
            else
                std::cerr << "Set affinity for thread: " <<  thr_id << std::endl;

            struct sched_param sp;
            sp.sched_priority = 10;
            if(sched_setscheduler(0, SCHED_FIFO, &sp) == 0)
                std::cerr << "Running with real-time priority (SCHED_FIFO)" << std::endl;
            else
                std::cerr << "Can't set SCHED_FIFO" << std::endl;
#endif

#pragma omp barrier
#pragma omp single
                {
                    overall_timer.reset();
                    overall_timer.start();
                }

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
#pragma omp barrier
#pragma omp single
                {
                    step_timer.reset();
                    step_timer.start();
                }

                maxes[thr_id*MAXES_STRIDE] = 0.0f;

                BOOST_FOREACH(lane *l, work.macro_lanes)
                {
                    assert(l->is_macro());
                    assert(l->active());
                    assert(!l->fictitious);
                    const float max            = l->collect_riemann();
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

                    maxspeed     = 0.0f;
                    for(size_t t = 0; t < max_thr; ++t)
                        maxspeed = std::max(maxspeed, maxes[t*MAXES_STRIDE]);

                    if(maxspeed < arz<float>::epsilon())
                        maxspeed = min_h;

                    dt                = std::min(cfl*min_h/maxspeed, 1.0f);
                    step_timer.stop();
                    max_compute_time += step_timer.interval_S();
                }

#pragma omp barrier
#pragma omp single
                {
                    step_timer.reset();
                    step_timer.start();
                }

                BOOST_FOREACH(lane *l, work.macro_lanes)
                {
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

#pragma omp barrier
#pragma omp single
                {
                    overall_timer.stop();
                }

        }
        printf("convert = %015.10lf\n", convert_time);
        printf("riemann = %015.10lf\n", riemann_time);
        printf("max     = %015.10lf\n", max_compute_time);
        printf("update  = %015.10lf\n", update_time);
        printf("micro   = %015.10lf\n", micro_time);
        printf("--------------------\n");
        printf("tot. time = %015.10lf\n", overall_timer.interval_S());
    }
}
