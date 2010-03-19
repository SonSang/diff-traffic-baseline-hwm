#include "libhybrid/hybrid-sim.hpp"

int main(int argc, char *argv[])
{
    hwm::network net(hwm::load_xml_network(argv[1], vec3f(1.0, 1.0, 15.0f)));
    net.build_intersections();
    net.build_fictitious_lanes();
    net.center();
    std::cerr << "HWM net loaded successfully" << std::endl;

    try
    {
        net.check();
        std::cerr << "HWM net checks out" << std::endl;
    }
    catch(std::runtime_error &e)
    {
        std::cerr << "HWM net doesn't check out: " << e.what() << std::endl;
        exit(1);
    }

    hybrid::simulator s(&net,
                        5.0f,
                        1.0);
    s.micro_initialize(0.73,
                       1.67,
                       33,
                       4);

    static const int cars_per_lane = 1;
    BOOST_FOREACH(hybrid::lane &l, s.lanes)
    {
        if(!l.parent->active)
            continue;

        double p = s.rear_bumper_offset()*l.inv_length;
        for (int i = 0; i < cars_per_lane; i++)
        {
            //TODO Just creating some cars here...
            l.current_cars().push_back(s.make_car(p, 33, 0));
            //Cars need a minimal distance spacing
            p += (15.0 / l.length);
        }
    }

    s.settle(0.033);

    s.update(0.033);
    s.car_swap();

    return 0;
}
