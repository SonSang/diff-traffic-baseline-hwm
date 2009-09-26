#ifndef _CAR_STREAM_HPP_
#define _CAR_STREAM_HPP_

struct input_car
{
    float time;
    int   lane;
    float speed;
};

std::deque<input_car> read_sumo_routing(const char *filename);

#endif
