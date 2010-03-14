#include "pc-poisson.hpp"
#include <iostream>

struct identity
{
    float operator()(const float x) const
    {
        return x;
    }
};

int main(int argc, char **argv)
{
    srand48(10);

    pc_data<float> pcd = pc_from_func(identity(), 0.05f, 100);
    pcd.write(std::cout);

    inhomogeneous_poisson<float> ihp(0.0f, pcd);

    for(size_t i = 0; i < 20; ++i)
        std::cout << ihp.next() << " ";
    std::cout << std::endl;

    return 0;
};
