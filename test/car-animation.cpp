#include "car-animation.hpp"
#include "array_macros.hpp"
#include <cstdio>
#include <cstdarg>
#include <cstdlib>
#include <cassert>

void die(const char *fmt, ...)
{
    va_list val;
    va_start(val, fmt);
    vfprintf(stderr, fmt, val);
    va_end(val);
    exit(EXIT_FAILURE);
}

static bool read_car_frame(car_frame *frame, float time, char *line)
{
    frame->time = time;
    int res = sscanf(line, "%d %f %f %f %f %f %f %f %f",
                     &frame->id,
                     frame->position+0,
                     frame->position+1,
                     frame->position+2,
                     frame->direction+0,
                     frame->direction+1,
                     frame->velocity+0,
                     frame->velocity+1,
                     &frame->acceleration);
    return res == 9;
}

static bool load_frame(car_animation *anim, FILE *fp)
{
    float time;
    int ncars_frame;
    int res = fscanf(fp, "%f %d\n", &time, &ncars_frame);
    if(res != 2)
        return false;

    anim->time_range[0] = std::min(time, anim->time_range[0]);
    anim->time_range[1] = std::max(time, anim->time_range[1]);

    for(int c = 0; c < ncars_frame; ++c)
    {
        char buff[2048];
        char *line_res = fgets(buff, 2047, fp);
        if(!line_res)
            die("Badly formed line in car %d of frame at time %f!\n", c, time);

        car_frame temp;
        int car_res = read_car_frame(&temp, time, buff);
        if(!car_res)
            die("Badly formed line in car %d of frame at time %f!\n", c, time);

        if(temp.id + 1 > anim->cars_n)
        {
            SIZE_ARRAY(anim->cars, temp.id + 1, anim->cars_n_allocd)
            for(int nc = anim->cars_n; nc < temp.id+1; ++nc)
            {
                car *current             = anim->cars + nc;
                current->id              = -1;
                current->frames_n        = 0;
                current->frames_n_allocd = 0;
                current->frames          = 0;
            }
            anim->cars_n = temp.id+1;
        }

        car *thiscar = anim->cars + temp.id;
        if(thiscar->frames == 0)
        {
            INIT_ARRAY(thiscar->frames, 100, thiscar->frames_n_allocd);
            thiscar->id = temp.id;
        }
        else
            assert(thiscar->id == temp.id);

        EXTEND_ARRAY(thiscar->frames, 1, thiscar->frames_n_allocd);
        thiscar->frames[thiscar->frames_n] = temp;
        ++thiscar->frames_n;
    }
    return true;
}

void load_trajectory_data(car_animation *anim, const char *filename)
{
    FILE *fp = fopen(filename, "r");
    if(!fp)
        die("Can't open %s for trajectory reading\n", filename);

    INIT_ARRAY(anim->cars, 500, anim->cars_n_allocd);

    int nframes = 0;
    while(load_frame(anim, fp))
        ++nframes;

    printf("Loaded %d frames from %f to %f. Maximum car id is %d\n", nframes,
           anim->time_range[0], anim->time_range[1],
           anim->cars_n);
    fclose(fp);
};
