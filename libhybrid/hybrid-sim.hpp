#include "libroad/hwm_network.hpp"
#include "arz.hpp"

namespace hybrid
{
    struct car
    {
        //common data
        size_t id;
        float  position;
        float  velocity;
        float  acceleration;

        // micro data

        // macro data
    };

    struct lane
    {
        // common data
        void              initialize(hwm::lane *parent);
        hwm::lane        *parent;
        float             length;
        std::vector<car>  cars[2];

        // micro data

        // macro data
        void  macro_initialize(const float h_suggest);
        float collect_riemann(const float gamma, const float inv_gamma);
        void  update         (const float dt,    const float relaxation_factor);
        void  fill_y(const float gamma);

        float                         h;
        size_t                        N;
        arz<float>::q                *q;
        arz<float>::riemann_solution *rs;
    };

    struct simulator
    {
        // common
        simulator(hwm::network *net);

        ~simulator();
        void initialize();

        hwm::network      *hnet;
        std::vector<lane>  lanes;
        float              time;

        // micro
        void  micro_initialize();
        void  micro_cleanup();
        float micro_step();
        std::vector<lane*> micro_lanes;

        // macro
        void  macro_initialize(float gamma, float h_suggest, float relaxation);
        void  macro_cleanup();
        float macro_step(const float cfl=1.0f);

        std::vector<lane*>            macro_lanes;
        arz<float>::q                *q_base;
        arz<float>::riemann_solution *rs_base;
        float                         gamma;
        float                         h_suggest;
        float                         min_h;
        float                         relaxation_factor;
    };
}
