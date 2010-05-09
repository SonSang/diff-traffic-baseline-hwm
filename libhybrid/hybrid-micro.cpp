#include "hybrid-sim.hpp"

#include <iostream>

namespace hybrid
{
    struct turn_curve
    {
        turn_curve(float in_x) : x_(in_x) {}

        float operator()(float t) const
        {
            return x(t) - x_;
        }

        float x_;

        static float x(float t)
        {
            return (((0.952735372896*t+
                      -0.999804060107*t) +
                     -1.72981115096*t) +
                    2.77923310042*t) +
            0.000979602079628;
        }

        static float y(float t)
        {
            return ((((4.95688785514*t+
                       -12.4993312165*t) +
                      9.28482561644*t) +
                     -0.8082796784*t) +
                    0.0691053639752*t) +
            -0.00106975213459;
        }

        static float theta(float t)
        {
            const float orientation = M_PI/2.0f;
            const float kmax = 1.1172f;
            const float m = 2.0f*orientation/kmax;
            const float inv_m = 1.0f/m;


            t *= m;

            if(t <= m/2)
                return kmax*t*t*inv_m;
            else if(t <= m)
                return -kmax*(t*t*inv_m - 2*t + m/2);
            else
                return kmax*m/2;
        }

        static float rotate(float t)
        {
            float max_rotation = M_PI/12.0;
            float factor = 1.5;

            if (t < 0.5)
            {
                return pow((t/0.5f), factor)*max_rotation;
            }
            else if (t == 0.5) //really just for illustrartion.
            {
                return max_rotation;
            }
            else  // (t > 0.5)
            {
                return pow(1 - t,factor)*max_rotation;
            }
        }

        static float x_end(float speed)
        {
            return  0.843112757647*speed+
            2.45445917647;
        }

        static float y_end(float speed)
        {
            return 0.244331745882*speed+
            4.11774005882;
        }
    };

    mat4x4f car::point_frame(const lane* l) const
    {
        mat4x4f trans;
        float pos[4];
        if (other_lane_membership.other_lane != 0)
        {
            float offset  = std::min(hybrid::turn_curve::y(other_lane_membership.merge_param), (float)1.0);
            offset       *= 2.5;
            float theta   = hybrid::turn_curve::rotate(other_lane_membership.merge_param);
            if (!other_lane_membership.is_left)
            {
                offset *= -1;
                theta  *= -1;
            }

            mat4x4f rotation;
            rotation(0, 0) = std::cos(theta);  rotation(0, 1) = -1*std::sin(theta);  rotation(0, 2) = 0;    rotation(0, 3) = 0;
            rotation(1, 0) = std::sin(theta);  rotation(1, 1) = std::cos(theta);     rotation(1, 2) = 0;    rotation(1, 3) = 0;
            rotation(2, 0) = 0;                rotation(2, 1) = 0;                   rotation(2, 2) = 1.0f; rotation(2, 3) = 0;
            rotation(3, 0) = 0.0f;             rotation(3, 1) = 0.0f;                rotation(3, 2) = 0.0f; rotation(3, 3) = 1.0f;

            trans = mat4x4f(other_lane_membership.other_lane->parent->point_frame(other_lane_membership.position, offset));
            pos[0] = trans(0,3); pos[1] = trans(1,3); pos[2] = trans(2,3); pos[3] = trans(3,3);

            trans = rotation*trans;
            trans(0,3) = pos[0]; trans(1,3) = pos[1]; trans(2,3) = pos[2]; trans(3,3) = pos[3];

        }
        else
            trans = mat4x4f(l->parent->point_frame(position));

        return trans;
    }

    float car::check_lane(const lane* l, const float param, const double timestep, const simulator &sim)
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
        if (f_hat < -0.5*sim.a_max)
            return std::numeric_limits<int>::min();
        else
            return (a_hat - acceleration) + polite*(f_hat - potential_follower.acceleration);
    }

    void car::check_if_valid_acceleration(lane& l, double timestep)
    {
        double tmp_position = position + (velocity * timestep) * l.inv_length;

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
        const double min_for_free_movement = 1000;

        hwm::lane *hwm_downstream = l.parent->downstream_lane();

        next_velocity = 0.0f;
        distance      = (1.0 - position) * l.length - sim.rear_bumper_offset();
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

    void car::integrate(const double timestep, const lane& l)
    {
        //Update position and velocity
        position += (velocity * timestep) * l.inv_length;

        //Move car that is also a member of the other lane
        float seconds_for_merge = 5.0f;
        if (other_lane_membership.other_lane != 0)
        {
            // TODO USE LANE LENGTH -- 0.2 = 1/5
            if (velocity > (2.5*0.2f))
            {
                other_lane_membership.merge_param += timestep / seconds_for_merge;
            }
            else
            {
                other_lane_membership.merge_param += (velocity*timestep*(1.0/2.5));
            }

            other_lane_membership.position += (velocity * timestep) * other_lane_membership.other_lane->inv_length;

            //TODO L is wheelbase -- length to rear axel
            // float L = 4.5;
            // float u_s = 1.0f/sin(other_lane_membership.theta)*velocity;
            // float d_x = u_s*cos(theta);
            // float d_theta = (u_s/L)*tan(other_lane_membership.merge_param*other_lane_membership.phi_max)*timestep;



            if (other_lane_membership.merge_param > 1)
            {
                other_lane_membership.merge_param = 0;
                other_lane_membership.other_lane = 0;
                other_lane_membership.position = 0;
            }
        }

        velocity = std::max(0.0, velocity + acceleration * timestep);
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

    void lane::compute_merges(const double timestep, const simulator& sim)
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
                double right_accel = std::numeric_limits<int>::min();
                lane* right_lane = 0;
                if (potential_right)
                {
                    right_lane = potential_right->user_data<lane>();
                    right_accel = current_car(i).check_lane(right_lane, right_param, timestep, sim);
                }

                float      left_param     = current_car(i).position;
                hwm::lane* potential_left = parent->left_adjacency(left_param);
                double     left_accel     = std::numeric_limits<int>::min();
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

    void lane::compute_lane_accelerations(const double timestep, const simulator &sim)
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
                 current_car(i).acceleration = std::min(current_car(i).acceleration, (double)right_accel);

             if (potential_left and next_l.position > -1)
                 current_car(i).acceleration = std::min(current_car(i).acceleration, (double)left_accel);

             //            current_car(i).check_if_valid_acceleration(*this, timestep);  For debugging only
         }
     }

     float lane::settle_pass(const double timestep, const double epsilon, const double epsilon_2,
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
                                                 (float) max_acceleration);
             }

             std::cout << "Max acceleration in settle: " << max_acceleration << std::endl;
         }
         while(max_acceleration > EPSILON);
     }

     double simulator::acceleration(const double leader_velocity, const double follower_velocity, double distance) const
     {
         static const double s1 = 2;
         static const double T  = 1.6;
         const float EPSILON = 10e-7;

         //if (f.vel < 0) f.vel = 0; //TODO Should this be taken into account?

         const double optimal_spacing = s1 + T*follower_velocity + (follower_velocity*(follower_velocity - leader_velocity))/2*(std::sqrt(a_max*a_pref));

         if (distance > car_length)
             distance -= car_length;
         if (distance == 0)
             distance = EPSILON;


         //         std::cout << "leader vel " << leader_velocity << " dist " << distance << std::endl;
         const double t               = a_max*(1 - std::pow((follower_velocity / v_pref), delta) - std::pow((optimal_spacing/(distance)), 2));

         //assert(l.dist - f.dist > 0); //A leader needs to lead.
         return t;
     }

     void simulator::compute_accelerations(const double timestep)
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
                     lane* destination_lane = &l;
                     lane* curr = &l;

                     while(c.position >= 1.0)
                     {
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
                            break;
                        }

                        if (c.position >= 1.0)
                        {
                            curr = downstream;
                        }
                    }

                    assert(c.position < 1.0);

                    destination_lane->next_cars().push_back(c);
                }
            }
        }
    }

    void simulator::micro_cleanup()
    {
    }
}
