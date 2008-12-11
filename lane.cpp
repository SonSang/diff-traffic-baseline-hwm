#include "network.hpp"

#define read_dead_end tautology
#define read_int_ref tautology
#define read_taper tautology

static bool tautology(void *item, xmlTextReaderPtr reader)
{
    return true;
}

static bool read_startend(void *item, xmlTextReaderPtr reader)
{
    const xmlChar *name = xmlTextReaderConstName(reader);

    xml_elt read[] =
        {{0,
          BAD_CAST "dead_end",
          item,
          read_dead_end},
         {0,
          BAD_CAST "intersection_ref",
          item,
          read_int_ref},
         {0,
          BAD_CAST "taper",
          item,
          read_taper}};

    if(!read_elements(reader, sizeof(read)/sizeof(read[0]), read, name))
        return false;

    return (read[0].count + read[1].count + read[2].count) == 1;
}

bool road_membership::xml_read(xmlTextReaderPtr reader)
{
    return true;
}

static bool read_road_membership_interval(void *item, xmlTextReaderPtr reader)
{
    lane *l = reinterpret_cast<lane*>(item);
    return l->road_memberships.interval_xml_read(reader, BAD_CAST "road_membership");
}

static bool read_road_intervals(void *item, xmlTextReaderPtr reader)
{
    xml_elt read[] =
        {{0,
          BAD_CAST "interval",
          item,
          read_road_membership_interval}};

    if(!read_elements(reader, sizeof(read)/sizeof(read[0]), read, BAD_CAST "road_intervals"))
        return false;

    return read[0].count;
}

static bool read_adjacency(void *item, xmlTextReaderPtr reader)
{
    do
    {
        int ret = xmlTextReaderRead(reader);
        if(ret != 1)
            return false;
    }
    while(!is_closing_element(reader, "adjacency_intervals"));

    return true;
}

bool lane::xml_read(xmlTextReaderPtr reader)
{
    if(!get_attribute(speedlimit, reader, "speedlimit"))
        return false;

    xml_elt read[] =
        {{0,
          BAD_CAST "start",
          this,
          read_startend},
         {0,
          BAD_CAST "end",
          this,
          read_startend},
         {0,
          BAD_CAST "road_intervals",
          this,
          read_road_intervals},
         {0,
          BAD_CAST "adjacency_intervals",
          this,
          read_adjacency}};

    bool status = read_elements(reader, sizeof(read)/sizeof(read[0]), read, BAD_CAST "lane");
    if(!status)
        return false;

    return read[0].count && read[1].count && read[2].count && read[3].count;
}
