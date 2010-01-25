#include "libhybrid/hybrid-sim.hpp"

int main(int argc, char *argv[])
{
    hwm::network net(hwm::load_xml_network(argv[1], vec3f(1.0, 1.0, 15.0f)));
    net.build_intersections();
    net.build_fictitious_lanes();
    net.center();
    std::cerr << "HWM net loaded successfully" << std::endl;

    if(net.check())
        std::cerr << "HWM net checks out" << std::endl;
    else
    {
        std::cerr << "HWM net doesn't check out" << std::endl;
        exit(1);
    }

    hybrid::simulator s(&net);
    s.micro_initialize(0.73,
                       1.67,
                       33,
                       4,
                       5,
                       1.0);
    BOOST_FOREACH(hybrid::lane &l, s.lanes)
    {
        if(l.parent->active)
            s.micro_lanes.push_back(&l);
    }

    static const int cars_per_lane = 1;
    BOOST_FOREACH(hybrid::lane *l, s.micro_lanes)
    {
        if(!l->parent->active)
            continue;

        double p = s.rear_bumper_offset()*l->inv_length;
        for (int i = 0; i < cars_per_lane; i++)
        {
            //TODO Just creating some cars here...
            hybrid::car tmp;
            tmp.position = p;
            tmp.velocity = 33;
            l->current_cars().push_back(tmp);
            //Cars need a minimal distance spacing
            p += (15.0 / l->length);
        }
    }

    s.settle(0.033);

    s.update(0.033);

    return 0;
}
