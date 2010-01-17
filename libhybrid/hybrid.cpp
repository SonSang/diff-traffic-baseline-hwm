#include "hybrid-sim.hpp"

namespace hybrid
{
    void lane::initialize(hwm::lane *in_parent)
    {
        parent             = in_parent;
        parent->user_datum = this;
        length             = parent->length();
    }

    simulator::simulator(hwm::network *net) : hnet(net), time(0.0f)
    {
        assert(hnet);

        // figure out how many lanes to create
        size_t lane_count = hnet->lanes.size();
        BOOST_FOREACH(const hwm::intersection_pair &ip, hnet->intersections)
        {
            BOOST_FOREACH(const hwm::intersection::state &current_state, ip.second.states)
            {
                lane_count += current_state.fict_lanes.size();
            }
        }

        // create them
        lanes.resize(lane_count);

        std::vector<lane>::iterator current     = lanes.begin();
        for(hwm::lane_map::iterator hwm_current = hnet->lanes.begin();
            hwm_current != hnet->lanes.end() && current != lanes.end();
            ++current, ++hwm_current)
        {
            current->initialize(&(hwm_current->second));
        }

        BOOST_FOREACH(hwm::intersection_pair &ip, hnet->intersections)
        {
            BOOST_FOREACH(hwm::intersection::state &current_state, ip.second.states)
            {
                BOOST_FOREACH(hwm::lane_pair &lp, current_state.fict_lanes)
                {
                    assert(current != lanes.end());
                    current->initialize(&(lp.second));
                    ++current;
                }
            }
        }
    }

    simulator::~simulator()
    {
        micro_cleanup();
        macro_cleanup();
    }
};

