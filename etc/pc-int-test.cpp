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

    BOOST_FOREACH(const float &p, poisson_points(0.0f, pcd.end(), 1000, 0.0f, pcd))
    {
        std::cout << p << " ";
    }
    std::cout << std::endl;

    return 0;
};
