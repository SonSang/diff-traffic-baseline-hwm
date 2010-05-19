#include "hybrid-sim.hpp"

#include <iostream>

namespace hybrid
{
    struct lc_curve
    {
        lc_curve(float target_y) : y_(target_y) {}

        inline float operator()(float t) const
        {
            return y(t) - y_;
        }

        static inline float y(float t)
        {
            return ((((8.7025182813*t+
                       -21.7562980512)*t +
                      15.5458393998)*t +
                     -1.56245957436)*t +
                    0.0709664576132)*t +
            -0.000283090356282;
        }

        static inline float end(float speed)
        {
            return 11.50731f*std::pow(speed, -0.5f);
        }

        float y_;
    };

    mat4x4f car::point_frame(const hwm::lane* l, const float lane_width) const
    {
        mat4x4f trans;
        float pos[4];
        if (other_lane_membership.other_lane != 0)
        {
            float offset  = std::min(lc_curve::y(other_lane_membership.merge_param), (float)1.0);
            offset       *= lane_width;
            if (!other_lane_membership.is_left)
            {
                offset *= -1;
            }

            mat4x4f rotation;
            rotation(0, 0) = std::cos(other_lane_membership.theta);  rotation(0, 1) = -1*std::sin(other_lane_membership.theta);  rotation(0, 2) = 0;    rotation(0, 3) = 0;
            rotation(1, 0) = std::sin(other_lane_membership.theta);  rotation(1, 1) = std::cos(other_lane_membership.theta);     rotation(1, 2) = 0;    rotation(1, 3) = 0;
            rotation(2, 0) = 0;                rotation(2, 1) = 0;                   rotation(2, 2) = 1.0f; rotation(2, 3) = 0;
            rotation(3, 0) = 0.0f;             rotation(3, 1) = 0.0f;                rotation(3, 2) = 0.0f; rotation(3, 3) = 1.0f;

            trans = mat4x4f(other_lane_membership.other_lane->parent->point_frame(other_lane_membership.position, offset));
            pos[0] = trans(0,3); pos[1] = trans(1,3); pos[2] = trans(2,3); pos[3] = trans(3,3);

            trans = rotation*trans;
            trans(0,3) = pos[0]; trans(1,3) = pos[1]; trans(2,3) = pos[2]; trans(3,3) = pos[3];

        }
        else
            trans = mat4x4f(l->point_frame(position));

        return trans;
    }

    vec3f car::point_theta(float &theta, const hwm::lane *l, const float lane_width) const
    {
        vec3f pos;
        if (other_lane_membership.other_lane != 0)
        {
            float offset  = std::min(lc_curve::y(other_lane_membership.merge_param), (float)1.0);
            offset       *= lane_width;
            if (!other_lane_membership.is_left)
            {
                offset *= -1;
            }

            theta = other_lane_membership.theta;

            float lane_theta;
            pos = other_lane_membership.other_lane->parent->point_theta(lane_theta, other_lane_membership.position, offset);
            theta += lane_theta;
        }
        else
            pos = l->point_theta(theta, position);

        return pos;
    }

    float car::check_lane(const lane* l, const float param, const float timestep, const simulator &sim)
    {
        float polite           = 0;
        car   potential_follower(0, 0, 0, 0);
        car   potential_leader(0, 0, 0, 0);
        bool  found_a_leader   = false;
        bool  found_a_follower = false;

        //Find the cars ahead and behind of where the car will merge
        ///TODO should do a binary search for positon
        for (int i = 0; i < (int)l->current_cars().size(); i++)
        {
            potential_leader   = l->current_car(i);

            if (potential_leader.position > param)
            {
                found_a_leader = true;
                break;
            }

            potential_follower = l->current_car(i);

            if (potential_follower.position <= param)
            {
                found_a_follower = true;
            }
        }

        if (found_a_leader == false)
        {
            float vel, pos;
            find_free_dist_and_vel(*l, vel, pos, sim);
            potential_leader = car(0, (pos / l->length)+position, vel, 0);
        }

        //Calc what the acceleration would be for this car were the lane change made.
        float a_hat = sim.acceleration(potential_leader.velocity, velocity, std::abs(potential_leader.position*l->length - param*l->length));

        bool found_a_next_leader = false;
        car  potential_next_follower(0, 0, 0, 0);
        car  potential_next_leader(0, 0, 0, 0);

        //Check for next cars too
        for (int i = 0; i < (int)l->next_cars().size(); i++)
        {
            potential_next_leader   = l->next_car(i);

            if (potential_next_leader.position > param)
            {
                found_a_next_leader = true;
                break;
            }

        }

        if (found_a_next_leader)
        {
            float a_hat_foo = sim.acceleration(potential_next_leader.velocity, velocity, std::abs(potential_next_leader.position*l->length - param*l->length));

            a_hat = std::min(a_hat, a_hat_foo);
        }

        //Check for cars that are still merging out of this lane
        float      a_hat_bar      = std::numeric_limits<float>::max();
        ///Check left lane if it's there
        float      left_param     = param;
        hwm::lane* potential_left = l->parent->left_adjacency(left_param);
        lane*      left_lane      = 0;
        float      a_hat_bar_l;
        if (potential_left)
        {
            left_lane = potential_left->user_data<lane>();
            car c = left_lane->get_merging_leader(left_param, l);
            if (c.position > -1)
            {
                a_hat_bar_l = sim.acceleration(c.velocity,
                                               velocity,
                                               std::abs(c.position*l->length - param*l->length));
                a_hat_bar = std::min(a_hat_bar, a_hat_bar_l);
            }
        }

        ///Check right lane if it's there
        float      right_param     = param;
        hwm::lane* potential_right = l->parent->right_adjacency(right_param);
        lane*      right_lane      = 0;
        float      a_hat_bar_r;
        if (potential_right)
        {
            right_lane = potential_right->user_data<lane>();
            car c = right_lane->get_merging_leader(right_param, l);
            if (c.position > -1)
            {
                a_hat_bar_r = sim.acceleration(c.velocity,
                                               velocity,
                                               std::abs(c.position*l->length - param*l->length));
                a_hat_bar   = std::min(a_hat_bar, a_hat_bar_r);
            }
        }

        a_hat = std::min(a_hat, a_hat_bar);

        //TODO need to search backward for follower if not found.

        //Calc what the deceleration would be for the new follower car.
        float f_hat            = 0;
        if (found_a_follower)
            f_hat              = sim.acceleration(velocity, potential_follower.velocity, std::abs(param*l->length - potential_follower.position*l->length));
        else
            potential_follower = car(0, 0, 0, 0);


        //Limit on how much deceleration is possible.
        float politeness = 0.6f;
        float max_deceleration = 10.0f;
        if (f_hat < -politeness*max_deceleration)//sim.a_max)
            return std::numeric_limits<int>::min();
        else
            return (a_hat - acceleration);
    }

    void car::check_if_valid_acceleration(lane& l, float timestep)
    {
        float tmp_position = position + (velocity * timestep) * l.inv_length;

        lane* curr = &l;
        lane* destination_lane = &l;

        while(tmp_position >= 1.0)
        {

            hwm::lane *hwm_downstream = curr->parent->downstream_lane();

            if (!hwm_downstream)
            {
                std::cout << "error" << std::endl;
                assert(0);
            }

            if (!hwm_downstream->active)
            {
                std::cout << "error" << std::endl;
                assert(0);
            }

            lane* downstream = hwm_downstream->user_data<lane>();

            tmp_position = (tmp_position - 1.0f) * curr->length * downstream->inv_length;

            destination_lane = downstream;

            if (tmp_position >= 1.0)
            {
                curr = downstream;
            }
        }
    }

    void car::compute_acceleration(const car &next, const float distance, const simulator &sim)
    {
        acceleration = sim.acceleration(next.velocity, velocity, distance);
    }

    void car::find_free_dist_and_vel(const lane &l, float& next_velocity, float& distance, const simulator& sim)
    {
        const float  min_for_free_movement = 1000;

        if(l.parent->end->network_boundary())
        {
            next_velocity = l.parent->speedlimit;
            distance      = min_for_free_movement;
            return;
        }

        hwm::lane *hwm_downstream = l.parent->downstream_lane();
        next_velocity             = 0.0f;
        distance                  = (1.0 - position) * l.length - sim.rear_bumper_offset();
        if(hwm_downstream)
            hwm_downstream->user_data<lane>()->distance_to_car(distance, next_velocity, min_for_free_movement, sim);
    }

    void car::compute_intersection_acceleration(const simulator &sim, const lane &l)
    {
        float next_velocity;
        float distance;
        find_free_dist_and_vel(l, next_velocity, distance, sim);

        acceleration = sim.acceleration(next_velocity, velocity, distance);
    }

    void car::integrate(const float timestep, const lane& l, const float lane_width)
    {
        //Update position and velocity
        velocity  = std::max(0.0f, velocity + acceleration * timestep);
        position += (velocity * timestep) * l.inv_length;

        //Move car that is also a member of the other lane
        if (other_lane_membership.other_lane != 0)
        {
            const float old_y                  = lc_curve::y(other_lane_membership.merge_param);
            float       needed_duration        = lc_curve::end(velocity);
            other_lane_membership.merge_param += timestep / needed_duration;
            const float new_y                  = lc_curve::y(other_lane_membership.merge_param);
            float       del_y                  = lane_width*(old_y-new_y);
            if(other_lane_membership.is_left)
                del_y                         *= -1;
            other_lane_membership.theta        = std::atan2(del_y, (velocity * timestep));
            other_lane_membership.position    += (velocity * timestep) * other_lane_membership.other_lane->inv_length;

            //TODO L is wheelbase -- length to rear axel
            // float L = 4.5;
            // float u_s = 1.0f/sin(other_lane_membership.theta)*velocity;
            // float d_x = u_s*cos(theta);
            // float d_theta = (u_s/L)*tan(other_lane_membership.merge_param*other_lane_membership.phi_max)*timestep;

            if (other_lane_membership.merge_param > 1)
            {
                other_lane_membership.merge_param = 0;
                other_lane_membership.other_lane  = 0;
                other_lane_membership.position    = 0;
                other_lane_membership.theta       = 0;
            }
        }


    }

    car lane::get_merging_leader(float param, const lane* other_lane)
    {
        car cur_car(-1, -1, -1, -1);
        for (size_t i = 0; i < current_cars().size(); i++)
        {
            if (current_car(i).other_lane_membership.other_lane != 0)
            {
                if (current_car(i).other_lane_membership.other_lane == other_lane)
                {
                    if (current_car(i).position >= param)
                    {
                        cur_car = current_car(i);
                        cur_car.position = current_car(i).other_lane_membership.position;
                    }
                }
            }
        }

        bool nxt_car_found = false;
        car nxt_car(-1, -1, -1, -1);
        for (size_t i = 0; i < next_cars().size(); i++)
        {
            if (next_car(i).other_lane_membership.other_lane != 0)
            {
                if (next_car(i).other_lane_membership.other_lane == other_lane)
                {
                    if (next_car(i).position >= param)
                    {
                        nxt_car = next_car(i);
                        nxt_car.position = next_car(i).other_lane_membership.position;
                        nxt_car_found =true;
                    }
                }
            }
        }

        if (nxt_car_found and (nxt_car.position < cur_car.position))
            return nxt_car;
        else
            return cur_car;
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
                if(parent->end->network_boundary())
                {
                    velocity = 0.0f;
                    distance = distance_max;
                    return;
                }

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

    void lane::compute_merges(const float timestep, const simulator& sim)
    {
        float threshold = 0.2;
        for(int i = (int)current_cars().size() - 1; i >= 0; --i)
        {
            //Don't consider merging again if still merging.
            if (current_car(i).other_lane_membership.other_lane == 0)
            {
                //Check if car decides to merge
                float right_param = current_car(i).position;

                hwm::lane* potential_right = parent->right_adjacency(right_param);
                float right_accel = std::numeric_limits<int>::min();
                lane* right_lane = 0;
                if (potential_right)
                {
                    right_lane = potential_right->user_data<lane>();
                    right_accel = current_car(i).check_lane(right_lane, right_param, timestep, sim);
                }

                float      left_param     = current_car(i).position;
                hwm::lane* potential_left = parent->left_adjacency(left_param);
                float     left_accel     = std::numeric_limits<int>::min();
                lane*      left_lane      = 0;
                if (potential_left)
                {
                    left_lane = potential_left->user_data<lane>();
                    left_accel = current_car(i).check_lane(left_lane, left_param, timestep, sim);
                }

                //Merge the cars
                if (right_accel  > threshold
                    or left_accel > threshold)
                {
                    if (right_accel > left_accel)
                    {
                        current_car(i).other_lane_membership.other_lane  = this;
                        current_car(i).other_lane_membership.merge_param = 0;
                        current_car(i).other_lane_membership.position    = current_car(i).position;
                        current_car(i).other_lane_membership.is_left     = false;
                        current_car(i).position                          = right_param;

                        right_lane->next_cars().push_back(current_car(i));
                    }
                    else
                    {
                        current_car(i).other_lane_membership.other_lane  = this;
                        current_car(i).other_lane_membership.merge_param = 0;
                        current_car(i).other_lane_membership.position    = current_car(i).position;
                        current_car(i).other_lane_membership.is_left     = true;
                        current_car(i).position                          = left_param;
                        left_lane->next_cars().push_back(current_car(i));
                    }
                }
                else
                {
                    next_cars().push_back(current_car(i));
                }
            }
            else
            {
                next_cars().push_back(current_car(i));
            }
        }
    }

    car& lane::find_next_car(float param)
    {
        //TODO bisect search
        for (size_t i = 0; i < current_cars().size(); i++)
        {
            if (current_car(i).position >= param)
            {
                return current_car(i);
            }
        }
        assert(0);
        return current_cars()[0];
    }

    void lane::compute_lane_accelerations(const float timestep, const simulator &sim)
    {
         if(current_cars().empty())
             return;

         for(size_t i = 0; i < current_cars().size(); ++i)
         {
             if (i < current_cars().size() - 1)
                 current_car(i).compute_acceleration(current_car(i+1), (current_car(i+1).position - current_car(i).position)*length, sim);
             else
                 current_cars().back().compute_intersection_acceleration(sim, *this);

             // //Check if there are cars still merging out of this lane.
             float right_param = current_car(i).position;
             hwm::lane* potential_right = parent->right_adjacency(right_param);
             lane* right_lane;
             float right_accel = 0;
             car next_r;
             if (potential_right)
             {
                right_lane = potential_right->user_data<lane>();

                next_r  = right_lane->get_merging_leader(right_param, this);

                if (next_r.position > -1)
                    right_accel = sim.acceleration(next_r.velocity, current_car(i).velocity, std::abs(next_r.position - right_param)*right_lane->length);
             }

             float left_param = current_car(i).position;
             hwm::lane* potential_left = parent->left_adjacency(left_param);
             lane* left_lane;
             float left_accel = 0;
             car next_l;
             if (potential_left)
             {
                left_lane = potential_left->user_data<lane>();

                next_l  = left_lane->get_merging_leader(left_param, this);
                if (next_l.position > -1)
                {
                    left_accel = sim.acceleration(next_l.velocity, current_car(i).velocity, std::abs(next_l.position - left_param)*left_lane->length);
                }
             }

             if (potential_right and next_r.position > -1)
                 current_car(i).acceleration = std::min(current_car(i).acceleration, (float)right_accel);

             if (potential_left and next_l.position > -1)
                 current_car(i).acceleration = std::min(current_car(i).acceleration, (float)left_accel);
         }
     }

     float lane::settle_pass(const float timestep, const float epsilon, const float epsilon_2,
                              const simulator &sim)
     {
         float max_acceleration = epsilon;
         int i = static_cast<int>(current_cars().size())-1;
         while(!current_cars().empty() && i >= 0)
         {
             car &c = current_car(i);

             float last_acceleration = std::numeric_limits<float>::max();
             while(true)
             {
                 compute_lane_accelerations(timestep, sim);

                 c.velocity = std::max(c.velocity + c.acceleration * timestep,
                                        0.0f);

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

     void simulator::micro_initialize(const float in_a_max, const float in_a_pref, const float in_v_pref,
                                      const float in_delta)
     {
         a_max                 = in_a_max;
         a_pref                = in_a_pref;
         v_pref                = in_v_pref;
         delta                 = in_delta;
     }

     void simulator::settle(const float timestep)
     {
         static const float EPSILON   = 1;
         static const float EPSILON_2 = 0.01;

         float max_acceleration;
         do
         {
             max_acceleration = EPSILON;
             BOOST_FOREACH(lane &l, lanes)
             {
                 if(l.is_micro() && l.parent->active)
                     max_acceleration = std::max(l.settle_pass(timestep, EPSILON, EPSILON_2, *this),
                                                 (float) max_acceleration);
             }

             std::cout << "Max acceleration in settle: " << max_acceleration << std::endl;
         }
         while(max_acceleration > EPSILON);
     }

     float simulator::acceleration(const float leader_velocity, const float follower_velocity, float distance) const
     {
         static const float s1 = 2;
         static const float T  = 1.6;
         const float EPSILON = 10e-7;

         //if (f.vel < 0) f.vel = 0; //TODO Should this be taken into account?

         const float optimal_spacing = s1 + T*follower_velocity + (follower_velocity*(follower_velocity - leader_velocity))/2*(std::sqrt(a_max*a_pref));

         if (distance > car_length)
             distance -= car_length;
         if (distance == 0)
             distance = EPSILON;


         //         std::cout << "leader vel " << leader_velocity << " dist " << distance << std::endl;
         const float t               = a_max*(1 - std::pow((follower_velocity / v_pref), delta) - std::pow((optimal_spacing/(distance)), 2));

         //assert(l.dist - f.dist > 0); //A leader needs to lead.
         return t;
     }

     void simulator::compute_accelerations(const float timestep)
     {
         BOOST_FOREACH(lane &l, lanes)
         {
             if (l.is_micro() && l.parent->active)
                 l.compute_lane_accelerations(timestep, *this);
         }

         BOOST_FOREACH(lane& l, lanes)
         {
             if (l.is_micro() and l.parent->active)
                 l.compute_merges(timestep, *this);
         }

         BOOST_FOREACH(lane& l, lanes)
         {
             l.car_swap();
         }
     }

     void simulator::update(const float timestep)
     {
         compute_accelerations(timestep);

         BOOST_FOREACH(lane &l, lanes)
         {
             if(l.is_micro() && l.parent->active)
             {
                 BOOST_FOREACH(car &c, l.current_cars())
                 {
                     c.integrate(timestep, l, hnet->lane_width);
                 }
             }
         }

         BOOST_FOREACH(lane &l, lanes)
         {
             if(l.is_micro() && l.parent->active)
             {
                 BOOST_FOREACH(car &c, l.current_cars())
                 {
                     lane* destination_lane = &l;
                     lane* curr = &l;

                     while(c.position >= 1.0)
                     {
                         if(curr->parent->end->network_boundary())
                             goto next_car;

                         hwm::lane *hwm_downstream = curr->parent->downstream_lane();
                         assert(hwm_downstream);
                         assert(hwm_downstream->active);

                         lane* downstream = hwm_downstream->user_data<lane>();

                         switch(downstream->sim_type)
                         {
                         case MICRO:
                             c.position = (c.position - 1.0f) * curr->length * downstream->inv_length;
                             // downstream->next_cars().push_back(c);
                             destination_lane = downstream;
                             break;
                        case MACRO: //TODO update for correctness
                            downstream->q[0].rho() = std::min(1.0f, downstream->q[0].rho() + car_length/downstream->h);
                            downstream->q[0].y()   = std::min(0.0f, arz<float>::eq::y(downstream->q[0].rho(), c.velocity,
                                                                                      hwm_downstream->speedlimit,
                                                                                      gamma));
                            assert(downstream->q[0].check());
                            goto next_car;
                        }

                        if (c.position >= 1.0)
                        {
                            curr = downstream;
                        }
                    }

                    assert(c.position < 1.0);

                    destination_lane->next_cars().push_back(c);

                 next_car:;
                }
            }
        }
    }

    void simulator::micro_cleanup()
    {
    }
}
