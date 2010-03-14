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
    pc_data<float> pcd = pc_from_func(identity(), 0.5f, 10);
    for(size_t i = 1; i < pcd.integration.size(); ++i)
        std::cout << pcd[i-1] << " ";
    std::cout << std::endl;

    std::cout << pcd.integrate(3.4) << std::endl;
    std::cout << pcd.inv_integrate(1.0) << std::endl;
    std::cout << pcd.inv_integrate(pcd.integrate(3.4)) << std::endl;
    std::cout << pcd.integrate(pcd.inv_integrate(1.0)) << std::endl;

    std::vector<float> obs;
    obs.push_back(1.0);
    obs.push_back(1.1);
    obs.push_back(4.3);

    pc_data<float> pca = pc_from_avg(obs, 0.5f, 10);
    for(size_t i = 1; i < pca.integration.size(); ++i)
        std::cout << pca[i-1] << " ";
    std::cout << std::endl;

    inhomogeneous_poisson<float> ihp(0.0f, pcd);

    for(size_t i = 0; i < 4; ++i)
        std::cout << ihp.next() << std::endl;

    return 0;
};
