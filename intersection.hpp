#ifndef _INTERSECTION_HPP_
#define _INTERSECTION_HPP_

void convex_hull(std::vector<point> & pts);

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
        std::vector<out_id> in_states; //< Mapping of incoming lane ids to
                                       //< out ids
        std::vector<in_id>  out_states;//< Mapping of outgoing lane ids to
    };                                 //< in ids

    bool xml_read(xmlTextReaderPtr reader);

    int next_state();

    lane* incoming_state(int intern_ref) const;
    lane* outgoing_state(int intern_ref) const;

    void build_shape(float lane_width);
    void draw() const;

    std::vector<lane_id> incoming; //< Lanes that flow
                                   //< _IN_ to intersection
    std::vector<lane_id> outgoing; //< Lanes that flow
                                   //< _OUT_ of intersection
    int current_state;
    std::vector<state> states;

    std::vector<point> shape;
    point center;
};

#endif
