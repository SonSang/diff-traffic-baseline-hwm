#include "libroad/hwm_network.hpp"
#include "arz.hpp"

namespace macro
{
    struct lane;

    struct simulator
    {
        simulator(hwm::network *net, float gamma, float h_suggest, float g);
        ~simulator();

        void  initialize();
        float step(float cfl=1.0f);

        std::vector<lane>             lanes;
        arz<float>::q                *q_base;
        arz<float>::riemann_solution *rs_base;
        float                         time;
        float                         gamma;
        float                         h_suggest;
        float                         min_h;
        float                         relaxation_factor;
        hwm::network                 *hnet;
    };

    struct lane
    {
        lane();
        void initialize(hwm::lane *parent, const float h_suggest);

        float collect_riemann(const float gamma, const float inv_gamma);
        void  update         (const float dt,    const float relaxation_factor);

        hwm::lane                    *parent;
        float                         length;
        float                         inv_length;
        float                         h;
        size_t                        N;
        arz<float>::q                *q;
        arz<float>::riemann_solution *rs;
    };
}
