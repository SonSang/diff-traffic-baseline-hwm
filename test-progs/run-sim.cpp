#include "network.hpp"

int main(int argc, char * argv[])
{
#define OUTPUT_RATE 0.3

    if(argc < 4)
    {
        fprintf(stderr, "Usage: %s <input network> <output file> <end time>\n", argv[0]);
        exit(1);
    }

    size_t frameno = 0;
    float last_dump;
    float end_time;

    printf("%s version: %s\n", argv[0], hwm_version_string());

    network net;

    if(!net.load_from_xml(argv[1]))
    {
        fprintf(stderr, "Couldn't load %s\n", argv[1]);
        exit(1);
    }

    net.calc_bounding_box();
    point pt;
    pt.x = -(net.bb[0]+net.bb[1])*0.5f;
    pt.y = -(net.bb[2]+net.bb[3])*0.5f;
    pt.z = 0.0f;
    net.translate(pt);
    net.prepare(H);

    int lno = 0;
    foreach(lane &la, net.lanes)
    {
        if(la.h > 0.8*H)
        {
            for(int i = 0; i < static_cast<int>(la.ncells/4); ++i)
            {
                if(drand48() > 0.5)
                    net.add_carticle(lno, drand48(), drand48()*la.speedlimit);
            }
        }
        ++lno;
    }

    net.fill_from_carticles();

    FILE *out_file = fopen(argv[2], "w");
    assert(out_file);

    sscanf(argv[3], "%f", &end_time);

    printf("Running for %f seconds\n", end_time);

    net.dump_carticles(out_file);
    last_dump = net.global_time;
    frameno = 1;

    while(net.global_time < end_time)
    {
        float dt =  net.sim_step();
        printf("t = %11.6f, dt = %11.6f\n", net.global_time, dt);
        if(net.global_time - last_dump > OUTPUT_RATE)
        {
            printf("frameno = %zu\n", frameno);
            net.dump_carticles(out_file);
            frameno++;
            last_dump = net.global_time;
        }
    };

    return 0;
}
