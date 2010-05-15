#include "hybrid-sim.hpp"

namespace hybrid
{
    car_interp::car_spatial::car_spatial()
    {
    }

    car_interp::car_spatial::car_spatial(const car &in_c, const hwm::lane *l)
        : c(in_c), la(l)
    {
    }

    car_interp::car_interp(simulator &s)
    {
        times[1]    = s.time;
        car_data[1] = s.get_car_hash();
    }

    void car_interp::capture(simulator &s)
    {
        std::swap(times[0], times[1]);
        std::swap(car_data[0], car_data[1]);

        times[1]    = s.time;
        car_data[1] = s.get_car_hash();
        inv_dt = 1.0/(times[1]-times[0]);
    }

    bool car_interp::in_second(size_t id) const
    {
        car_spatial tmp;
        tmp.c.id = id;
        const car_hash::const_iterator high (car_data[1].find(tmp));
        return high != car_data[1].end();
    }

    static bool walk_lanes(const hwm::lane                        *&res,
                           float                                   &param,
                           const car_interp::car_hash::value_type  &start,
                           const car_interp::car_hash::value_type  &end,
                           float                                    rel_time)
    {
        std::vector<std::pair<float, const hwm::lane*> > visited;
        visited.push_back(std::make_pair(0, start.la));
        float            distance = start.la->length()*(1-start.c.position);
        {
            const hwm::lane *current  = start.la->downstream_lane();
            while(current != end.la)
            {
                if(!current)
                    return false;
                visited.push_back(std::make_pair(distance, current));
                distance += current->length();
                current  = current->downstream_lane();
            }
            visited.push_back(std::make_pair(distance, current));
            distance += end.la->length()*end.c.position;
        }

        const float target_distance = rel_time*distance;
        res                         = 0;
        param                       = 0;
        for(size_t i = 1; i < visited.size(); ++i)
        {
            if(target_distance < visited[i].first)
            {
                res   = visited[i-1].second;
                param = (target_distance-visited[i-1].first)/res->length();
                break;
            }
        }
        if(!res)
        {
            res   = end.la;
            param = (target_distance-visited.back().first)/res->length();
        }
        if(res == start.la)
            param += start.c.position;
        return true;
    }

    mat4x4f car_interp::point_frame(size_t id, float time, float lane_width) const
    {
        car_spatial tmp;
        tmp.c.id = id;
        const car_hash::const_iterator low (car_data[0].find(tmp));
        const car_hash::const_iterator high(car_data[1].find(tmp));

        assert(low  != car_data[0].end());
        assert(high != car_data[1].end());

        const float w0 = 1 - (time - times[0])*inv_dt;
        const float w1 = 1 - (times[1] - time)*inv_dt;

        if(low->la == high->la)
        {
            car fake_car;
            fake_car.position                          = low->c.position*w0 + high->c.position*w1;
            fake_car.other_lane_membership.other_lane  = (low->c.other_lane_membership.other_lane == high->c.other_lane_membership.other_lane) ? low->c.other_lane_membership.other_lane : 0;
            fake_car.other_lane_membership.is_left     = low->c.other_lane_membership.is_left;
            fake_car.other_lane_membership.merge_param = low->c.other_lane_membership.merge_param*w0 +
                                                         high->c.other_lane_membership.merge_param*w1;
            fake_car.other_lane_membership.position    = low->c.other_lane_membership.position*w0 +
                                                         high->c.other_lane_membership.position*w1;
            fake_car.other_lane_membership.theta       = low->c.other_lane_membership.theta*w0 +
                                                         high->c.other_lane_membership.theta*w1;
            return fake_car.point_frame(low->la, lane_width);
        }
        else
        {
            const hwm::lane *l;
            float param;
            if(walk_lanes(l, param, *low, *high, 1-w0))
                return l->point_frame(param);
            else
            {
                mat4x4f res0(low->c.point_frame(low->la, lane_width));
                mat4x4f res1(high->c.point_frame(high->la, lane_width));
                return mat4x4f(res0*w0+res1*w1);
            }
        }
    }
}
