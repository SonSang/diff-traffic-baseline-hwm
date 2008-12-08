#ifndef _INTERSECTION_HPP_
#define _INTERSECTION_HPP_

struct intersection
{
    typedef int id;
    enum {TAPER = -2, DEAD_END = -1};

    std::vector<lane::id> incoming;
    std::vector<lane::id> outgoing;
};

#endif
