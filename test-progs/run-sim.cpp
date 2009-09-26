#include "network.hpp"
#include <omp.h>

#include "xmmintrin.h"
#include "pmmintrin.h"

int main(int argc, char * argv[])
{
    if(argc < 3)
    {
        fprintf(stderr, "Usage: %s <input network> <input flow>\n", argv[0]);
        exit(1);
    }

    _MM_SET_FLUSH_ZERO_MODE(_MM_FLUSH_ZERO_ON);
    _MM_SET_DENORMALS_ZERO_MODE(_MM_DENORMALS_ZERO_ON);

    printf("%s version: %s\n", argv[0], hwm_version_string());

    network net;

    if(!net.load_from_xml(argv[1]))
    {
        fprintf(stderr, "Couldn't load %s\n", argv[1]);
        exit(1);
    }

    std::deque<input_car> icars = read_sumo_routing(argv[2]);

    net.calc_bounding_box();

    point pt;
    pt.x = -(net.bb[0]+net.bb[1])*0.5f;
    pt.y = -(net.bb[2]+net.bb[3])*0.5f;
    pt.z = 0.0f;

    net.translate(pt);
    net.prepare(H);

    net.fill_from_carticles();

    timer t;

    t.start();
    size_t steps = 0;
    float end_time = 8000.0f;
    while(net.global_time < end_time)
    {
        if(!icars.empty() && net.global_time > icars.front().time)
        {
            size_t lno = icars.front().lane;
            net.add_carticle(lno, CAR_LENGTH/(net.lanes[lno].h*net.lanes[lno].ncells), icars.front().speed);
            net.lanes[lno].data[0].rho += CAR_LENGTH/net.lanes[lno].h;
            if(net.lanes[lno].data[0].rho >= 0.95)
                net.lanes[lno].data[0].rho = 0.95;
            net.lanes[lno].data[0].y = to_y(CAR_LENGTH/net.lanes[lno].h, icars.front().speed, net.lanes[lno].speedlimit, net.gamma_c);

            icars.pop_front();
        }

        net.sim_step();
        ++steps;
    };
    t.stop();

    printf("%zu steps; end time = %f took %f seconds\n", steps, net.global_time, t.interval_S());

    return 0;
}
