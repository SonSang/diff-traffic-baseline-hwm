#include "libhybrid/pc-poisson.hpp"
#include <iostream>

const float car_length = 4.5;

struct identity
{
    float operator()(const float x) const
    {
        return 1.0/car_length*(std::cos(x)+1);
    }
};

int main(int argc, char **argv)
{
    std::cerr << libhybrid_package_string() << std::endl;

    srand48(10);

    pproc::pc_data<float> pcd = pproc::pc_from_func(identity(), 0.05f, 1000);
    pcd.write(std::cout);

    const float sep      = car_length;
    const float interval = 100.0;

    for(int j = 0; j < 1000; ++j)
    {
        float last = -sep;
        pproc::pc_integrator<pproc::pc_data<float> > pci(&pcd);
        size_t i = 0;
        for(; last + sep < 50.0; ++i)
        {
            const float c = pproc::texp(last+sep, last+sep+interval, pci);
            last = c;
        }
        std::cout << i << " ";
    }
    std::cout << std::endl;

    return 0;
};
