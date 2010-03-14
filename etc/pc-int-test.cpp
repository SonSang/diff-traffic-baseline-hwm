#include "pc-integrate.hpp"
#include <iostream>

int main(int argc, char **argv)
{
    const float dx = 0.5f;

    std::vector<float> d(10);
    for(size_t i = 0; i < d.size(); ++i)
    {
        const float x = i*dx;
        d[i] = x;
    }

    pc_data<float> pcd(dx, d, 0.0);

    for(size_t i = 0; i < pcd.data.size(); ++i)
    {
        std::cout << pcd.data[i] << " ";
    }
    std::cout << std::endl;

    std::cout << pcd.integrate(3.4) << std::endl;
    std::cout << pcd.inv_integrate(1.0) << std::endl;
    std::cout << pcd.inv_integrate(pcd.integrate(3.4)) << std::endl;
    std::cout << pcd.integrate(pcd.inv_integrate(1.0)) << std::endl;
    return 0;
};
