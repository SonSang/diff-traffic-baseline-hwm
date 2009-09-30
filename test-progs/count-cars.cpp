#include "network.hpp"
#include <boost/lexical_cast.hpp>

int main(int argc, char *argv[])
{
    if(argc < 3)
    {
        fprintf(stderr, "Usage: %s <routing_file> <time>\n", argv[0]);
        exit(1);
    }

    std::deque<input_car> icars = read_sumo_routing(argv[1]);

    float finish_time = boost::lexical_cast<float>(argv[2]);

    size_t count = 0;
    foreach(const input_car &ic, icars)
    {
        if(ic.time > finish_time)
            break;
        ++count;
    }

    printf("Found %zu cars before time %f\n", count, finish_time);

    return 0;
};

