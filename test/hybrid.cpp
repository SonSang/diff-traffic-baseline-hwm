#include "libhybrid/hybrid-sim.hpp"
#include "libhybrid/timer.hpp"

int main(int argc, char *argv[])
{
    std::cout << libroad_package_string() << std::endl;
    std::cerr << libhybrid_package_string() << std::endl;
    if(argc < 2)
    {
        std::cerr << "Usage: " << argv[0] << " <network file> [number of threads]" << std::endl;
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

    int n_threads = 1;
    if(argc == 3)
        n_threads = boost::lexical_cast<int>(argv[2]);
    omp_set_num_threads(n_threads);
    std::cout << "OpenMP using " << n_threads << " threads" << std::endl;

    hybrid::simulator s(&net,
                        4.5f,
                        1.0);
    s.micro_initialize(0.73,
                       1.67,
                       33,
                       4);
    s.macro_initialize(4.1*4.5, 0.0f);

    BOOST_FOREACH(hybrid::lane &l, s.lanes)
    {
        l.sim_type = hybrid::MICRO;
        l.populate(0.25/s.car_length, s);
        s.convert_to_macro(l);
    }

    int   num_steps = 100;
    timer step_timer;
    step_timer.reset();
    step_timer.start();
    s.parallel_hybrid_run(num_steps);
    step_timer.stop();
    std::cout << num_steps << " " << " " << step_timer.interval_S() << " " << step_timer.interval_S()/num_steps << " " << s.ncars() << std::endl;

    return 0;
}
