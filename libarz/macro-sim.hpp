#include "libroad/hwm_network.hpp"
#include "arz.hpp"

namespace macro
{
    struct lane;

    struct simulator
    {
        simulator(hwm::network *net) : hnet(net)
        {}

        ~simulator();

        void initialize();

        std::vector<lane> lanes;

        arz<float>::q                *base;
        arz<float>::riemann_solution *rs_base;
        float                         time;
        float                         gamma;
        hwm::network                 *hnet;
    };

    struct lane
    {
        hwm::lane                    *parent;
        float                         length;
        float                         inv_length;
        float                         h;
        size_t                        N;
        arz<float>::q                *q;
        arz<float>::riemann_solution *rs;
    };
}
