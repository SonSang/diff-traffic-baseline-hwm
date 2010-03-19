#include "libroad/hwm_network.hpp"
#include "arz.hpp"
#include <boost/random.hpp>

namespace hybrid
{
    typedef enum {MACRO=1, MICRO=2} sim_t;

    struct simulator;
    struct lane;

    struct car
    {
        car() : id(0) {}
        car(const size_t in_id, const double in_position,
            const double in_velocity, const double in_acceleration)
            : id(in_id), position(in_position),
              velocity(in_velocity), acceleration(in_acceleration)
        {}

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

    struct flux_capacitor
    {
        flux_capacitor();

        void update(const arz<float>::riemann_solution &rs, const float coefficient);
        bool check() const;
        car  emit(simulator &sim);

        float rho_accum;
        float u_accum;
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
        bool                    is_micro() const;
        bool                    is_macro() const;
        bool                    occupied() const;

        hwm::lane        *parent;
        float             length;
        float             inv_length;
        std::vector<car>  cars[2];
        sim_t             sim_type;

        void distance_to_car(float &distance, float &velocity, const float distance_max, const simulator &sim) const;

        // micro data
        void   micro_distance_to_car(float &distance, float &velocity, const float distance_max, const simulator &sim) const;
        void   compute_lane_accelerations(double timestep, const simulator &sim);
        double settle_pass(const double timestep, const double epsilon, const double epsilon_2, const simulator &sim);

        // macro data
        void  macro_initialize(const float h_suggest);
        void  macro_instantiate(simulator &sim);
        bool  macro_find_first(float &param, const simulator &sim) const;
        bool  macro_find_last(float &param, const simulator &sim) const;
        void  macro_distance_to_car(float &distance, float &velocity, const float distance_max, const simulator &sim) const;
        int   which_cell(float pos) const;
        float velocity(float pos, float gamma) const;
        float collect_riemann(const float gamma, const float inv_gamma);
        void  update         (const float dt,    simulator  &sim);
        void  clear_macro();
        void  convert_cars(const simulator &sim);
        void  fill_y(const float gamma);

        flux_capacitor                capacitor;
        float                         h;
        float                         inv_h;
        size_t                        N;
        arz<float>::q                *q;
        arz<float>::riemann_solution *rs;
    };

    struct simulator
    {
        // common
        simulator(hwm::network *net, float length, float rear_axle);

        ~simulator();
        void  initialize();
        float rear_bumper_offset()  const;
        float front_bumper_offset() const;
        void  car_swap();

        car make_car(const double position, const double velocity, const double acceleration);

        void hybrid_step();

        void advance_intersections(float dt);

        hwm::network          *hnet;
        std::vector<lane>      lanes;
        float                  car_length;
        float                  rear_bumper_rear_axle;
        float                  time;
        typedef boost::rand48  base_generator_type;
        base_generator_type   *generator;
        boost::uniform_real<> *uni_dist;
        typedef boost::variate_generator<base_generator_type&,
            boost::uniform_real<> > rand_gen_t;
        rand_gen_t            *uni;
        size_t                 car_id_counter;


        // micro
        void   micro_initialize(const double a_max, const double a_pref, const double v_pref,
                                const double delta);
        void   micro_cleanup();
        void   settle(const double timestep);
        double acceleration(const double leader_velocity, const double follower_velocity, const double distance) const;
        void   compute_accelerations(double timestep);
        void   update(double timestep);

        double a_max;
        double a_pref;
        double v_pref;
        double delta;

        // macro
        void  macro_initialize(float gamma, float h_suggest, float relaxation);
        void  macro_cleanup();
        void  convert_cars(sim_t sim_mask);
        float macro_step(const float cfl=1.0f);

        arz<float>::q                *q_base;
        arz<float>::riemann_solution *rs_base;
        float                         gamma;
        float                         h_suggest;
        float                         min_h;
        float                         relaxation_factor;
    };
}
