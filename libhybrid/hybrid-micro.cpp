#include "hybrid-sim.hpp"

namespace hybrid
{
    void car::compute_acceleration(const car &next, const float distance, const simulator &sim)
    {
        acceleration = sim.acceleration(next.velocity, velocity, distance);
    }

    void car::compute_intersection_acceleration(const simulator &sim, const lane &l)
    {
        const double min_for_free_movement = 1000;

        hwm::lane *hwm_downstream = l.parent->downstream_lane();

        float next_velocity = 0.0f;
        float distance      = (1.0 - position) * l.length - sim.rear_bumper_offset();
        if(hwm_downstream)
            hwm_downstream->user_data<lane>()->distance_to_car(distance, next_velocity, min_for_free_movement, sim);

        std::cout << "distance: " << distance << " velocity: " << next_velocity << std::endl;
        acceleration = sim.acceleration(next_velocity, velocity, distance);
    }

    void car::integrate(const double timestep, const lane& l)
    {
        position += (velocity     * timestep)*l.inv_length;
        velocity = std::max(0.0, velocity + acceleration * timestep);
    }

    void lane::micro_distance_to_car(float &distance, float &velocity, const float distance_max, const simulator &sim) const
    {
        if(current_cars().empty())
        {
            distance += length;
            if(distance >= distance_max)
            {
                velocity = 0.0f;
                distance = distance_max;
            }
            else
            {
                hwm::lane *hwm_downstream = parent->downstream_lane();
                if(hwm_downstream)
                    hwm_downstream->user_data<lane>()->distance_to_car(distance, velocity, distance_max, sim);
                else
                    velocity = 0.0f;
            }
        }
        else
        {
            velocity  = current_cars().front().velocity;
            distance += current_cars().front().position*length;
        }

        return;
    }

    void lane::compute_lane_accelerations(const double timestep, const simulator &sim)
    {
        if(current_cars().empty())
            return;

        for(size_t i = 0; i < current_cars().size()-1; ++i)
            current_car(i).compute_acceleration(current_car(i+1), (current_car(i+1).position - current_car(i).position)*length, sim);

        current_cars().back().compute_intersection_acceleration(sim, *this);
    }

    double lane::settle_pass(const double timestep, const double epsilon, const double epsilon_2,
                             const simulator &sim)
    {
        double max_acceleration = epsilon;
        int i = static_cast<int>(current_cars().size())-1;
        while(!current_cars().empty() && i >= 0)
        {
            car &c = current_car(i);

            double last_acceleration = std::numeric_limits<double>::max();
            while(true)
            {
                compute_lane_accelerations(timestep, sim);

                c.velocity = std::max(c.velocity + c.acceleration * timestep,
                                       0.0);

                max_acceleration = std::max(std::abs(c.acceleration), max_acceleration);

                if( std::abs(c.acceleration) > std::abs(last_acceleration)
                    or c.position+sim.front_bumper_offset()*inv_length >= 1.0
                    or ((std::abs(c.acceleration - last_acceleration) < epsilon_2) and (std::abs(c.acceleration) > epsilon)))
                {
                    for(int j = i; j < static_cast<int>(current_cars().size())-1; ++j)
                        current_car(j) = current_car(j+1);
                    current_cars().pop_back();
                    break;
                }
                else if(std::abs(c.acceleration) < epsilon)
                {
                    --i;
                    break;
                }

                last_acceleration = c.acceleration;
            }

        }

        return max_acceleration;
    }

    void simulator::micro_initialize(const double in_a_max, const double in_a_pref, const double in_v_pref,
                                     const double in_delta)
    {
        a_max                 = in_a_max;
        a_pref                = in_a_pref;
        v_pref                = in_v_pref;
        delta                 = in_delta;
    }

    void simulator::settle(const double timestep)
    {
        static const double EPSILON   = 1;
        static const double EPSILON_2 = 0.01;

        double max_acceleration;
        do
        {
            max_acceleration = EPSILON;
            BOOST_FOREACH(lane &l, lanes)
            {
                if(l.is_micro() && l.parent->active)
                    max_acceleration = std::max(l.settle_pass(timestep, EPSILON, EPSILON_2, *this),
                                                max_acceleration);
            }

            std::cout << "Max acceleration in settle: " << max_acceleration << std::endl;
        }
        while(max_acceleration > EPSILON);
    }

    double simulator::acceleration(const double leader_velocity, const double follower_velocity, const double distance) const
    {
        static const double s1 = 2;
        static const double T  = 1.6;

        //if (f.vel < 0) f.vel = 0; //TODO Should this be taken into account?

        const double optimal_spacing = s1 + T*follower_velocity + (follower_velocity*(follower_velocity - leader_velocity))/2*(std::sqrt(a_max*a_pref));
        const double t               = a_max*(1 - std::pow((follower_velocity / v_pref), delta) - std::pow((optimal_spacing/(distance - car_length)), 2));

        //assert(l.dist - f.dist > 0); //A leader needs to lead.
        return t;
    }

    void simulator::compute_accelerations(const double timestep)
    {
        BOOST_FOREACH(lane &l, lanes)
        {
            if(l.is_micro() && l.parent->active)
                l.compute_lane_accelerations(timestep, *this);
        }
    }

    void simulator::update(const double timestep)
    {
        compute_accelerations(timestep);

        BOOST_FOREACH(lane &l, lanes)
        {
            if(l.is_micro() && l.parent->active)
            {
                BOOST_FOREACH(car &c, l.current_cars())
                {
                    c.integrate(timestep, l);
                }
            }
        }

        BOOST_FOREACH(lane &l, lanes)
        {
            if(l.is_micro() && l.parent->active)
            {
                BOOST_FOREACH(car &c, l.current_cars())
                {
                    if(c.position >= 1.0)
                    {
                        hwm::lane *hwm_downstream = l.parent->downstream_lane();
                        assert(hwm_downstream);
                        assert(hwm_downstream->active);

                        lane *downstream = hwm_downstream->user_data<lane>();

                        switch(downstream->sim_type)
                        {
                        case MICRO:
                            c.position = (c.position - 1.0f)*l.length * downstream->inv_length;
                            downstream->next_cars().push_back(c);
                            break;
                        case MACRO:
                            downstream->q[0].rho() = std::min(1.0f, downstream->q[0].rho() + car_length/downstream->h);
                            downstream->q[0].y()   = std::min(0.0f, arz<float>::eq::y(downstream->q[0].rho(), c.velocity,
                                                                                      hwm_downstream->speedlimit,
                                                                                      gamma));
                            assert(downstream->q[0].check());
                            break;
                        }
                    }
                    else
                        l.next_cars().push_back(c);
                }
            }
        }
    }

    void simulator::micro_cleanup()
    {
    }
}
