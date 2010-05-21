#include "libhybrid/hybrid-sim.hpp"

int main(int argc, char *argv[])
{
    std::cout << libroad_package_string() << std::endl;
    std::cerr << libhybrid_package_string() << std::endl;
    if(argc < 2)
    {
        std::cerr << "Usage: " << argv[0] << " <network file>" << std::endl;
        return 1;
    }

    hwm::network net(hwm::load_xml_network(argv[1], vec3f(1.0, 1.0, 1.0f)));

    net.build_intersections();
    net.build_fictitious_lanes();
    net.auto_scale_memberships();
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
                        4.5f,
                        1.0);
    s.micro_initialize(0.73,
                       1.67,
                       33,
                       4);
    s.macro_initialize(0.5, 4.1*4.5, 0.0f);

    BOOST_FOREACH(hybrid::lane &l, s.lanes)
    {
        l.sim_type = hybrid::MICRO;
        l.populate(0.05/s.car_length, s);
        l.convert_to_macro(s);
    }

    int   num_steps = 0;
    while(1)
    {
        float dt = s.hybrid_step();
        s.advance_intersections(dt);
        ++num_steps;
        std::cout << "\r" << num_steps << " " << dt;
        std::cout.flush();
    }
    return 0;
}
