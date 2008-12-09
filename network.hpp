#ifndef _NETWORK_HPP_
#define _NETWORK_HPP_

#include "xml-util.hpp"
#include "lane.hpp"

struct network
{
    bool load_from_xml(const char * filename);
    bool xml_read(xmlTextReaderPtr reader);

    ~network();

    char * name;
    std::vector<road> roads;
    std::vector<lane> lanes;
    std::vector<intersection> intersections;
};
#endif
