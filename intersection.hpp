#ifndef _INTERSECTION_HPP_
#define _INTERSECTION_HPP_

struct intersection
{
    struct state
    {
        typedef int in_id;
        typedef int out_id;
        enum {STARVATION=-1, STOP=-1};

        bool xml_read(xmlTextReaderPtr reader);

        int id;
        float duration;
        std::vector<in_id> in_states;
        std::vector<out_id> out_states;
    };

    bool xml_read(xmlTextReaderPtr reader);

    std::vector<lane_id> incoming;
    std::vector<lane_id> outgoing;

    int current_state;
    std::vector<state> states;
};

#endif
