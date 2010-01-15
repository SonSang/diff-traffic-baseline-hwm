#include "macro-sim.hpp"

namespace macro
{
    simulator::~simulator()
    {
        delete base;
        delete rs_base;
    }

    void simulator::initialize()
    {
    }
}
