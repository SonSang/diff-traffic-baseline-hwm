#include "pc-poisson.hpp"
#include <iostream>

struct identity
{
    float operator()(const float x) const
    {
        return 0.1*(std::cos(x)+1);
    }
};

int main(int argc, char **argv)
{
    srand48(10);

    pc_data<float> pcd = pc_from_func(identity(), 0.05f, 1000);
    pcd.write(std::cout);

    const float sep      = 0.5;
    const float interval = 100.0;

    float last = 0.0;
    pc_integrator<pc_data<float> > pci(&pcd);
    for(size_t i = 0; i < 50; ++i)
    {
        const float c = texp(last+sep, std::min(last+sep+interval, 50.0f), pci);
        std::cout << c << " ";
        last = c;
    }
    std::cout << std::endl;

    return 0;
};
