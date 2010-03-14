#include "pc-integrate.hpp"
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
    pc_data<float> pcd = pc_from_func(identity(), 0.5f, 10);
    for(size_t i = 1; i < pcd.integration.size(); ++i)
        std::cout << pcd[i-1] << " ";
    std::cout << std::endl;

    std::cout << pcd.integrate(3.4) << std::endl;
    std::cout << pcd.inv_integrate(1.0) << std::endl;
    std::cout << pcd.inv_integrate(pcd.integrate(3.4)) << std::endl;
    std::cout << pcd.integrate(pcd.inv_integrate(1.0)) << std::endl;
    return 0;
};
