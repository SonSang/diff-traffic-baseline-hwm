#ifndef _NETWORK_HPP_
#define _NETWORK_HPP_

struct road;
typedef union {
    road * rp;
    char * sp;
} road_id;

struct lane;
typedef union {
    lane * lp;
    char * sp;
} lane_id;

struct intersection;
typedef union {
    intersection * lp;
    char * sp;
} intersection_id;

#include <vector>
#include <map>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <ctype.h>
#include "xml-util.hpp"
#include "intervals.hpp"
#include "arz.hpp"
#include "road.hpp"
#include "lane.hpp"
#include "intersection.hpp"

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
