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

        bool isvalid(int nincoming, int noutgoing) const;

        int id;
        float duration;
        std::vector<out_id> in_states; //< Mapping of incoming lane ids to
                                       //< out ids
        std::vector<in_id>  out_states;//< Mapping of outgoing lane ids to
                                       //< in ids

        std::vector<road> fict_roads;
        std::vector<lane> fict_lanes;
    };

    bool xml_read(xmlTextReaderPtr reader);

    int next_state();

    lane* incoming_state(int intern_ref) const; //< Returns the OUTGOING lane corresponing to
                                                //< incoming lane intern_ref, or 0 if none
    lane* outgoing_state(int intern_ref) const;  //< Returns the INCOMING lane corresponing to
                                                //< OUTGOING lane intern_ref, or 0 if none

    void build_shape(float lane_width);
    void draw() const;

    float collect_riemann(float gamma_c, float inv_gamma);

    void initialize_state_lanes();

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
