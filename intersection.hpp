#ifndef _INTERSECTION_HPP_
#define _INTERSECTION_HPP_

#include <vector>
#include "lane.hpp"

struct intersection
{
    struct state
    {
        typedef int in_id;
        typedef int out_id;
        enum {STARVATION=-1, STOP=-1};

        int id;
        float duration;
        std::vector<in_id> in_states;
        std::vector<out_id> out_states;
    };

    std::vector<lane*> incoming;
    std::vector<lane*> outgoing;

    int current_state;
    std::vector<state> states;
};

#endif
