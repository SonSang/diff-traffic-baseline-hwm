#include "hybrid-sim.hpp"

namespace hybrid
{
    lane::serial_state::serial_state()
    {
    }

    lane::serial_state::serial_state(const lane &l) : cars(l.current_cars()),
                                                      sim_type(l.sim_type)
    {
        assert(l.next_cars().empty());
    }

    void lane::serial_state::apply(lane &l) const
    {
        l.sim_type       = sim_type;
        l.current_cars() = cars;
    }

    lane::lane() : parent(0), N(0), q(0), rs(0)
    {
    }

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

    bool lane::active() const
    {
        return parent->active;
    }

    void lane::convert(const sim_t dest_type, simulator &sim)
    {
        switch(dest_type)
        {
        case MICRO:
            convert_to_micro(sim);
            break;
        case MACRO:
            convert_to_macro(sim);
            break;
        default:
            assert(0);
            return;
        }
    };

    void lane::convert_to_micro(simulator &sim)
    {
        if(sim_type == MICRO)
            return;

        macro_instantiate(sim);

        sim_type = MICRO;
    }

    void lane::convert_to_macro(simulator &sim)
    {
        if(sim_type == MACRO)
            return;

        clear_macro();
        convert_cars(sim);
        fill_y(sim.gamma);

        current_cars().clear();
        next_cars().clear();

        sim_type = MACRO;
    }

    lane *lane::left_adjacency(float &param)
    {
        hwm::lane *l(parent->left_adjacency(param));
        return l ? l->user_data<lane>() : 0;
    }

    lane *lane::right_adjacency(float &param)
    {
        hwm::lane *l(parent->right_adjacency(param));
        return l ? l->user_data<lane>() : 0;
    }

    lane *lane::upstream_lane()
    {
        hwm::lane *l(parent->upstream_lane());
        return l ? l->user_data<lane>() : 0;
    }

    lane *lane::downstream_lane()
    {
        hwm::lane *l(parent->downstream_lane());
        return l ? l->user_data<lane>() : 0;
    }

    lane::serial_state lane::serial() const
    {
        return serial_state(*this);
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

    simulator::serial_state::serial_state() : q_base(0)
    {
    }

    simulator::serial_state::~serial_state()
    {
        if(q_base)
            delete[] q_base;
    }

    simulator::serial_state::serial_state(const simulator &s) : car_id_counter(s.car_id_counter),
                                                                generator(*s.generator),
                                                                network_state(s.hnet->serial())
    {
        lane_states.reserve(s.lanes.size());
        BOOST_FOREACH(const lane &l, s.lanes)
        {
            lane_states.push_back(l.serial());
        }

        q_base = new arz<float>::q[s.N];
        std::memcpy(q_base, s.q_base, s.N*sizeof(sizeof(arz<float>::q)));
    }

    car_interp::car_hash simulator::get_car_hash() const
    {
        car_interp::car_hash res;
        BOOST_FOREACH(const lane &l, lanes)
        {
            if(!l.parent->active || !l.is_micro())
                continue;

            BOOST_FOREACH(const car &c, l.current_cars())
            {
                res.insert(car_interp::car_spatial(c, l.parent));
            }
        }
        return res;
    }

    void simulator::serial_state::apply(simulator &s) const
    {
        s.car_id_counter = car_id_counter;
        *s.generator     = generator;
        network_state.apply(*s.hnet);
        assert(q_base);
        std::memcpy(s.q_base, q_base, s.N*sizeof(sizeof(arz<float>::q)));
    }

    simulator::simulator(hwm::network *net, float length, float rear_axle)
        : hnet(net),
          car_length(length),
          rear_bumper_rear_axle(rear_axle),
          time(0.0f),
          car_id_counter(1),
          q_base(0),
          rs_base(0)
    {
        generator = new base_generator_type(42ul);
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

    car simulator::make_car(const float position, const float velocity,
                            const float acceleration)
    {
        return car(car_id_counter++, position, velocity, acceleration);
    }

    lane &simulator::get_lane_by_name(const str &s)
    {
        const hwm::lane_map::iterator res(hnet->lanes.find(s));
        if(res == hnet->lanes.end())
            throw std::runtime_error("No such lane!");
        return *(res->second.user_data<lane>());
    }

    const lane &simulator::get_lane_by_name(const str &s) const
    {
        const hwm::lane_map::iterator res(hnet->lanes.find(s));
        if(res == hnet->lanes.end())
            throw std::runtime_error("No such lane!");
        return *(res->second.user_data<const lane>());
    }

    float simulator::hybrid_step()
    {
        // fill in micro
        convert_cars(MICRO);

        // // macro step (also emit cars)
        float dt = macro_step(1.0f);

        // micro step
        update(dt);

        car_swap();

        time += dt;

	return dt;
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

    simulator::serial_state simulator::serial() const
    {
        return serial_state(*this);
    }
}
