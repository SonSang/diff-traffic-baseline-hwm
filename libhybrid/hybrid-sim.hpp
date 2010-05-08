#include "libroad/hwm_network.hpp"
#include "libhybrid/libhybrid-common.hpp"
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
        {
            other_lane_membership.other_lane = 0;
            other_lane_membership.merge_param = 0;
        }

        //common data
        size_t id;
        double position;
        double velocity;
        double acceleration;

        // micro data
        struct Other_lane_membership
        {
            bool   is_left;
            lane  *other_lane;
            float  merge_param;
            float  position;
            float  theta;
            float  phi_max;
        } other_lane_membership;

        void compute_acceleration(const car &f, const float distance, const simulator &sim);
        void find_free_dist_and_vel(const lane& l, float& next_velocity, float& distance, const simulator& sim);
        void compute_intersection_acceleration(const simulator &sim, const lane &l);
        void integrate(double timestep, const lane &l);
        void check_if_valid_acceleration(lane& l, double timestep);
        float check_lane(const lane* l, const float param, const double timestep, const simulator& sim);
        mat4x4f point_frame(const lane* l) const;

        // macro data
    };

    struct lane
    {
        struct serial_state
        {
            serial_state();
            serial_state(const lane &l);

            void apply(lane &l) const;

            std::vector<car> cars;
            sim_t            sim_type;
        };

        lane();

        // common data
        void                     initialize(hwm::lane *parent);

        const std::vector<car>  &current_cars() const { return cars[0]; }
        std::vector<car>        &current_cars()       { return cars[0]; }
        const car               &current_car(size_t i) const { return cars[0][i]; }
        car                     &current_car(size_t i)       { return cars[0][i]; };

        const std::vector<car>  &next_cars() const { return cars[1]; };
        std::vector<car>        &next_cars()       { return cars[1]; };
        const car               &next_car(size_t i) const { return cars[1][i]; }
        car                     &next_car(size_t i)       { return cars[1][i]; };

        void                    car_swap();
        bool                    is_micro() const;
        bool                    is_macro() const;
        bool                    occupied() const;
        void                    convert(sim_t dest_type, simulator &sim);
        void                    convert_to_micro(simulator &sim);
        void                    convert_to_macro(simulator &sim);

        serial_state serial() const;

        hwm::lane        *parent;
        float             length;
        float             inv_length;
        std::vector<car>  cars[2];
        sim_t             sim_type;
        bool              updated_flag;

        void distance_to_car(float &distance, float &velocity, const float distance_max, const simulator &sim) const;

        // micro data
        void  micro_distance_to_car(float &distance, float &velocity, const float distance_max, const simulator &sim) const;
        void  compute_lane_accelerations(double timestep, const simulator &sim);
        float settle_pass(const double timestep, const double epsilon, const double epsilon_2, const simulator &sim);
        void  compute_merges(const double timestep, const simulator& sim);
        car&  find_next_car(float param);
        car   get_merging_leader(float param, const lane* other_lane);

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

        float                         h;
        float                         inv_h;
        size_t                        N;
        arz<float>::q                *q;
        arz<float>::riemann_solution *rs;
    };

    struct simulator
    {
        typedef boost::rand48  base_generator_type;

        struct serial_state
        {
            serial_state();
            serial_state(const simulator &s);

            void apply(simulator &s) const;

            size_t               car_id_counter;
            arz<float>::q       *q_base;
            base_generator_type  generator;

            hwm::network::serial_state      network_state;
            std::vector<lane::serial_state> lane_states;
        };

        // common
        simulator(hwm::network *net, float length, float rear_axle);

        ~simulator();
        void  initialize();
        float rear_bumper_offset()  const;
        float front_bumper_offset() const;
        void  car_swap();

        car   make_car(const double position, const double velocity, const double acceleration);

        lane       &get_lane_by_name(const str &s);
        const lane &get_lane_by_name(const str &s) const;

        void hybrid_step();
        void advance_intersections(float dt);

        serial_state serial() const;

        hwm::network          *hnet;
        std::vector<lane>      lanes;
        float                  car_length;
        float                  rear_bumper_rear_axle;
        float                  time;
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
