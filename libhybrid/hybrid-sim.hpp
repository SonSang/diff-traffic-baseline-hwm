#include "libroad/hwm_network.hpp"
#include "arz.hpp"

namespace hybrid
{
    struct simulator;
    struct lane;

    struct car
    {
        //common data
        size_t id;
        double position;
        double velocity;
        double acceleration;

        // micro data
        void compute_acceleration(const car &f, const float distance, const simulator &sim);
        void compute_intersection_acceleration(const simulator &sim, const lane &l);
        void integrate(double timestep, const lane &l);

        // macro data
    };

    struct lane
    {
        lane();

        // common data
        void                     initialize(hwm::lane *parent);

        const std::vector<car>  &current_cars() const { return cars[0]; }
        std::vector<car>        &current_cars()       { return cars[0]; }
        const car               &current_car(size_t i) const { return cars[0][i]; }
        car                     &current_car(size_t i)       { return cars[0][i]; };

        const std::vector<car>  &next_cars() const { return cars[1]; };
        std::vector<car>        &next_cars()       { return cars[1]; };

        void                    car_swap();

        hwm::lane        *parent;
        float             length;
        float             inv_length;
        std::vector<car>  cars[2];

        // micro data
        void   compute_lane_accelerations(double timestep, const simulator &sim);
        double settle_pass(const double timestep, const double epsilon, const double epsilon_2, const simulator &sim);

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
        void   micro_initialize(const double a_max, const double a_pref, const double v_pref,
                                const double delte, const double length);
        void   micro_cleanup();
        void   settle(const double timestep);
        double acceleration(const double leader_velocity, const double follower_velocity, const double distance) const;
        void   compute_accelerations(double timestep);
        void   update(double timestep);

        std::vector<lane*> micro_lanes;
        double a_max;
        double a_pref;
        double v_pref;
        double delta;
        double car_length;

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
