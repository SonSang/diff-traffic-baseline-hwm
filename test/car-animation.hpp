#ifndef __CAR_ANIMATION_HPP__
#define __CAR_ANIMATION_HPP__

struct car_frame
{
    float time;
    int   id;
    float position[3];
    float direction[2];
    float velocity[2];
    float acceleration;
};

struct car
{
    int        id;
    int        frames_n;
    int        frames_n_allocd;
    car_frame *frames;
};

struct car_animation
{
    int  cars_n;
    int  cars_n_allocd;
    car *cars;
    float time_range[2];
};

void load_trajectory_data(car_animation *anim, const char *filename);
#endif
