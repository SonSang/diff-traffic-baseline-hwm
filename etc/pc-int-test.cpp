#include "pc-poisson.hpp"
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
    srand48(10);

    pc_data<float> pcd = pc_from_func(identity(), 0.05f, 1000);
    pcd.write(std::cout);

    const float sep      = car_length;
    const float interval = 100.0;

    for(int j = 0; j < 1000; ++j)
    {
        float last = -sep;
        pc_integrator<pc_data<float> > pci(&pcd);
        size_t i = 0;
        for(; last + sep < 50.0; ++i)
        {
            const float c = texp(last+sep, last+sep+interval, pci);
            last = c;
        }
        std::cout << i << " ";
    }
    std::cout << std::endl;

    return 0;
};
