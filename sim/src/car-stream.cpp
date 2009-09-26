#include "network.hpp"

std::deque<input_car> read_sumo_routing(const char *filename)
{
    xmlTextReaderPtr reader = xmlReaderForFile(filename, NULL, 0);

    if(!reader)
    {
        fprintf(stderr, "Couldn't open %s for reading\n", filename);
        exit(1);
    }

    int ret = xmlTextReaderRead(reader);
    while (ret == 1 && !is_opening_element(reader, "routes"))
        ret = xmlTextReaderRead(reader);

    if(ret != 1)
    {
        if(ret == -1)
            fprintf(stderr, "Parsing error in %s\n", filename);
        else if(ret == 0)
            fprintf(stderr, "Couldn't find routes element in %s\n", filename);
        exit(1);
    }

    std::deque<input_car> res;
    ret = xmlTextReaderRead(reader);
    while (ret == 1 && !is_closing_element(reader, "routes"))
    {
        if(is_opening_element(reader, "vehicle"))
        {
            input_car ic;
            if(!(get_attribute(ic.time, reader, "depart") &&
                 get_attribute(ic.lane, reader, "departlane") &&
                 get_attribute(ic.speed, reader, "departspeed")))
            {
                fprintf(stderr, "Couldn't get needed parts of vehicle\n");
                exit(1);
            }

            res.push_back(ic);
        }
        ret = xmlTextReaderRead(reader);
    }

    xmlFreeTextReader(reader);
    xmlCleanupParser();

    return res;
}
