#include "libhybrid/arz.hpp"

int main(int argc, char *argv[])
{
    arz<float>::q left(0.00131,
                       -0.12273);

    arz<float>::q right(0.00168,
                        0.00000);

    arz<float>::full_q fq_left(left,
                               33.33333,
                               0.5);

    arz<float>::full_q fq_right(right,
                                33.33333,
                                0.5);

    arz<float>::riemann_solution rs;
    rs.lebaque_inhomogeneous_riemann(fq_left,
                                     fq_right,
                                     33.33333,
                                     33.33333,
                                     0.5,
                                     1.0/0.5);


    std::cout << rs << std::endl;
    return 0;
}
