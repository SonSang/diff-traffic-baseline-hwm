#include "libarz/macro-sim.hpp"

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

    macro::simulator s(&net, 0.5, 10.0f, 0.4f);
    s.initialize();

    s.step();

    return 0;
}
