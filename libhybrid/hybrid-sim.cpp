#include "libhybrid/hybrid-sim.hpp"

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

    const lane *lane::upstream_lane() const
    {
        const hwm::lane *l(parent->upstream_lane());
        return l ? l->user_data<const lane>() : 0;
    }

    lane *lane::downstream_lane()
    {
        hwm::lane *l(parent->downstream_lane());
        return l ? l->user_data<lane>() : 0;
    }

    const lane *lane::downstream_lane() const
    {
        const hwm::lane *l(parent->downstream_lane());
        return l ? l->user_data<const lane>() : 0;
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
        BOOST_FOREACH(const lane *l, micro_lanes)
        {
            assert(l->is_micro());
            if(!l->active())
                continue;

            BOOST_FOREACH(const car &c, l->current_cars())
            {
                res.insert(car_interp::car_spatial(c, l->parent));
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

        float                       min_len         = std::numeric_limits<float>::max();
        float                       max_speedlimit  = -std::numeric_limits<float>::max();
        std::vector<lane>::iterator current         = lanes.begin();
        for(hwm::lane_map::iterator hwm_current = hnet->lanes.begin();
            hwm_current                            != hnet->lanes.end() && current != lanes.end();
            ++current, ++hwm_current)
        {
            current->initialize(&(hwm_current->second));

            current->fictitious = false;
            min_len             = std::min(current->length, min_len);
            max_speedlimit      = std::max(current->speedlimit(), max_speedlimit);
        }

        std::cout << "Min length of regular lane is: " << min_len << std::endl;
        std::cout << "Max speedlimit of regular lane is: " << max_speedlimit << std::endl;

        min_len = std::numeric_limits<float>::max();
        BOOST_FOREACH(hwm::intersection_pair &ip, hnet->intersections)
        {
            BOOST_FOREACH(hwm::intersection::state &current_state, ip.second.states)
            {
                BOOST_FOREACH(hwm::lane_pair &lp, current_state.fict_lanes)
                {
                    assert(current != lanes.end());
                    current->initialize(&(lp.second));
                    current->fictitious = true;
                    min_len             = std::min(current->length, min_len);
                    ++current;
                }
            }
        }
        std::cout << "Min length of fict lane  is: " << min_len << std::endl;
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
        BOOST_FOREACH(lane *l, micro_lanes)
        {
            l->car_swap();
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

    size_t simulator::ncars() const
    {
        size_t res = 0;
        BOOST_FOREACH(const lane *l, micro_lanes)
        {
            res += l->ncars();
        }
        return res;
    }

    void simulator::mass_reassign(std::vector<hwm::network_aux::road_spatial::entry> &qr)
    {
        BOOST_FOREACH(lane &l, lanes)
        {
            l.updated_flag = false;
        }
        // micro_lanes.clear();
        // macro_lanes.clear();
        BOOST_FOREACH(hwm::network_aux::road_spatial::entry &e, qr)
        {
            BOOST_FOREACH(hwm::network_aux::road_rev_map::lane_cont::value_type &lcv, *e.lc)
            {
                hwm::lane    &hwm_l = *(lcv.second.lane);
                hybrid::lane &hyb_l = *(hwm_l.user_data<lane>());
                if(!hyb_l.updated_flag && hyb_l.active())
                {
                    convert_to_micro(hyb_l);
                    hyb_l.updated_flag = true;
                }
            }
        }
        BOOST_FOREACH(hwm::intersection_pair &ip, hnet->intersections)
        {
            BOOST_FOREACH(hwm::intersection::state &s, ip.second.states)
            {
                BOOST_FOREACH(hwm::lane_pair &lp, s.fict_lanes)
                {
                    lane &hyb_l = *(lp.second.user_data<lane>());
                    if(!hyb_l.updated_flag && hyb_l.active())
                    {
                        lane *up_l(hyb_l.upstream_lane());
                        lane *dn_l(hyb_l.downstream_lane());
                        if((up_l && up_l->updated_flag && up_l->is_micro()) ||
                           (dn_l && dn_l->updated_flag && dn_l->is_micro()))
                        {
                            convert_to_micro(hyb_l);
                            hyb_l.updated_flag = true;
                        }
                    }
                }
            }
        }

        BOOST_FOREACH(lane &l, lanes)
        {
            if(!l.updated_flag && l.active())
            {
                convert_to_macro(l);
                l.updated_flag = true;
            }
        }
    }

    float simulator::hybrid_step()
    {
        // fill in micro
        convert_cars(MICRO);

        // // macro step (also emit cars)
        float dt = macro_step(1.0f);

        // micro step
        update(dt);

        time += dt;
        apply_incoming_bc(dt, time);

        car_swap();

        return dt;
    }

    void simulator::convert_to_micro(lane &l)
    {
        if(l.sim_type == MICRO)
            return;

        if(!l.fictitious)
            l.macro_instantiate(*this);

        std::vector<lane*>::iterator loc = std::find(macro_lanes.begin(),
                                                     macro_lanes.end(),
                                                     &l);
        if(loc != macro_lanes.end())
        {
            std::swap(*loc, *boost::prior(macro_lanes.end()));
            macro_lanes.pop_back();
        }

        micro_lanes.push_back(&l);
        l.sim_type = MICRO;
    }

    void simulator::convert_to_macro(lane &l)
    {
        if(l.sim_type == MACRO)
            return;

        l.sim_type = MACRO;

        std::vector<lane*>::iterator loc = std::find(micro_lanes.begin(),
                                                     micro_lanes.end(),
                                                     &l);
        if(loc != micro_lanes.end())
        {
            std::swap(*loc, *boost::prior(micro_lanes.end()));
            micro_lanes.pop_back();
        }

        if(l.fictitious)
            return;

        macro_lanes.push_back(&l);

        l.clear_macro();
        l.convert_cars(*this);
        l.fill_y();

        l.current_cars().clear();
        l.next_cars().clear();
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

    void simulator::apply_incoming_bc(float dt, float t)
    {
        static const float MIN_SPEED_FRACTION = 0.7;
        static const float rate               = 0.05;
        BOOST_FOREACH(lane *l, micro_lanes)
        {
            assert(l->is_micro());
            if(!l->parent->start->network_boundary())
                continue;

            car new_car;
            if(l->current_cars().empty())
            {
                const float add_prob     = (*uni)();
                const float prob_of_none = std::exp(-rate*dt);
                if(add_prob <= prob_of_none)
                    continue;

                new_car = make_car(0, std::max((float)(*uni)(), MIN_SPEED_FRACTION)*l->speedlimit(), 0);
                new_car.compute_intersection_acceleration(*this, *l);
            }
            else
            {
                const car &leader = l->current_car(0);
                if(!(leader.position * l->length > 2*car_length))
                    continue;

                const float add_prob     = (*uni)();
                const float prob_of_none = std::exp(-rate*dt);
                if(add_prob <= prob_of_none)
                    continue;

                new_car           = make_car(0, leader.velocity, 0);
                new_car.compute_acceleration(leader, (leader.position - new_car.position)*l->length, *this);
            }
            l->next_cars().push_back(new_car);
        }
    }

    simulator::serial_state simulator::serial() const
    {
        return serial_state(*this);
    }
}
