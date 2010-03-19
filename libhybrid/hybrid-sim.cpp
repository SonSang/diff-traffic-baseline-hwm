#include "hybrid-sim.hpp"

namespace hybrid
{
    lane::lane() : parent(0), N(0), q(0), rs(0)
    {}

    void lane::initialize(hwm::lane *in_parent)
    {
        parent             = in_parent;
        parent->user_datum = this;
        length             = parent->length();
        inv_length         = 1.0f/length;
    }

    struct car_sort
    {
        inline bool operator()(const car &l, const car &r)
        {
            return l.position < r.position;
        }
    };

    void lane::car_swap()
    {
        cars[0].swap(cars[1]);
        cars[1].clear();
        std::sort(cars[0].begin(), cars[0].end(), car_sort());
    }

    bool lane::is_micro() const
    {
        return sim_type == MICRO;
    }

    bool lane::is_macro() const
    {
        return sim_type == MACRO;
    }

    bool lane::occupied() const
    {
        switch(sim_type)
        {
        case MICRO:
            return !current_cars().empty() || !next_cars().empty();
        case MACRO:
            for(size_t i = 0; i < N; ++i)
            {
                if(q[i].rho() > 3*arz<float>::epsilon())
                    return true;
            }
            return false;
        default:
            assert(0);
            return false;
        }
    }

    void lane::distance_to_car(float &distance, float &velocity, const float distance_max, const simulator &sim) const
    {
        switch(sim_type)
        {
        case MICRO:
            return micro_distance_to_car(distance, velocity, distance_max, sim);
        case MACRO:
            return macro_distance_to_car(distance, velocity, distance_max, sim);
        default:
            assert(0);
            return;
        }
    }

    simulator::simulator(hwm::network *net) : hnet(net),
                                              time(0.0f),
                                              car_id_counter(1),
                                              q_base(0),
                                              rs_base(0)
    {
        generator = new base_generator_type(42u);
        uni_dist  = new boost::uniform_real<>(0,1);
        uni       = new boost::variate_generator<base_generator_type&, boost::uniform_real<> >(*generator, *uni_dist);

        assert(hnet);

        // figure out how many lanes to create
        size_t lane_count = hnet->lanes.size();
        BOOST_FOREACH(const hwm::intersection_pair &ip, hnet->intersections)
        {
            BOOST_FOREACH(const hwm::intersection::state &current_state, ip.second.states)
            {
                lane_count += current_state.fict_lanes.size();
            }
        }

        // create them
        lanes.resize(lane_count);

        std::vector<lane>::iterator current     = lanes.begin();
        for(hwm::lane_map::iterator hwm_current = hnet->lanes.begin();
            hwm_current != hnet->lanes.end() && current != lanes.end();
            ++current, ++hwm_current)
        {
            current->initialize(&(hwm_current->second));
        }

        BOOST_FOREACH(hwm::intersection_pair &ip, hnet->intersections)
        {
            BOOST_FOREACH(hwm::intersection::state &current_state, ip.second.states)
            {
                BOOST_FOREACH(hwm::lane_pair &lp, current_state.fict_lanes)
                {
                    assert(current != lanes.end());
                    current->initialize(&(lp.second));
                    ++current;
                }
            }
        }
    }

    simulator::~simulator()
    {
        delete generator;
        delete uni_dist;
        delete uni;

        micro_cleanup();
        macro_cleanup();
    }

    float simulator::rear_bumper_offset() const
    {
        return -rear_bumper_rear_axle;
    }

    float simulator::front_bumper_offset() const
    {
        return car_length-rear_bumper_rear_axle;
    }

    void simulator::car_swap()
    {
        BOOST_FOREACH(lane &l, lanes)
        {
            l.car_swap();
        }
    }

    car simulator::make_car(const double position, const double velocity,
                            const double acceleration)
    {
        return car(car_id_counter++, position, velocity, acceleration);
    }

    void simulator::hybrid_step()
    {
        // fill in micro
        convert_cars(MICRO);

        // macro step (also emit cars)
        const float dt = macro_step(1.0f);

        // micro step
        update(dt);

        car_swap();
    }

    void simulator::advance_intersections(float dt)
    {
        BOOST_FOREACH(hwm::intersection_pair &ip, hnet->intersections)
        {
            hwm::intersection &i = ip.second;

            i.state_time += dt;
            if(i.locked || i.state_time > i.states[i.current_state].duration)
            {
                bool free = true;
                BOOST_FOREACH(const hwm::lane_pair &lp, i.states[i.current_state].fict_lanes)
                {
                    if(lp.second.user_data<lane>()->occupied())
                    {
                        free = false;
                        break;
                    }
                }

                if(free)
                {
                    i.unlock();
                    i.advance_state();
                }
                else
                    i.lock();
            }
        }
    }
}
