#include "network.hpp"

template <>
bool interval_xml_read<road_membership>::xml_read(road_membership & item, xmlTextReaderPtr reader)
{
    return item.xml_read(reader);
}

template <>
bool interval_xml_read<adjacency_pair*>::xml_read(adjacency_pair *& item, xmlTextReaderPtr reader)
{
    item = 0;
    return true;
}

static bool read_dead_end(void *item, xmlTextReaderPtr reader)
{
    lane_end *le = reinterpret_cast<lane_end*>(item);
    le->end_type = lane_end::DEAD_END;
    return true;
}

static bool read_int_ref(void *item, xmlTextReaderPtr reader)
{
    lane_end *le = reinterpret_cast<lane_end*>(item);
    le->end_type = lane_end::INTERSECTION;

    return get_attribute(le->inters.sp, reader, "ref");
}

static bool read_taper(void *item, xmlTextReaderPtr reader)
{
    lane_end *le = reinterpret_cast<lane_end*>(item);
    le->end_type = lane_end::TAPER;
    return true;
}

static bool read_start(void *item, xmlTextReaderPtr reader)
{
    const xmlChar *name = xmlTextReaderConstName(reader);

    lane *l = reinterpret_cast<lane*>(item);

    xml_elt read[] =
        {{0,
          BAD_CAST "dead_end",
          &(l->start),
          read_dead_end},
         {0,
          BAD_CAST "intersection_ref",
          &(l->start),
          read_int_ref},
         {0,
          BAD_CAST "taper",
          &(l->start),
          read_taper}};

    if(!read_elements(reader, sizeof(read)/sizeof(read[0]), read, name))
        return false;

    return (read[0].count + read[1].count + read[2].count) == 1;
}

static bool read_end(void *item, xmlTextReaderPtr reader)
{
    const xmlChar *name = xmlTextReaderConstName(reader);

    lane *l = reinterpret_cast<lane*>(item);

    xml_elt read[] =
        {{0,
          BAD_CAST "dead_end",
          &(l->end),
          read_dead_end},
         {0,
          BAD_CAST "intersection_ref",
          &(l->end),
          read_int_ref},
         {0,
          BAD_CAST "taper",
          &(l->end),
          read_taper}};

    if(!read_elements(reader, sizeof(read)/sizeof(read[0]), read, name))
        return false;

    return (read[0].count + read[1].count + read[2].count) == 1;
}

static bool read_road_membership_interval(void *item, xmlTextReaderPtr reader)
{
    lane *l = reinterpret_cast<lane*>(item);
    return l->road_memberships.xml_read(reader, BAD_CAST "road_membership");
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

    printf("Read: %d road intervals\n", read[0].count);

    return read[0].count == 1;
}

static bool read_left_interval(void *item, xmlTextReaderPtr reader)
{
    lane *l = reinterpret_cast<lane*>(item);
    return l->left.xml_read(reader, BAD_CAST "lane_adjacency");
}

static bool read_left_adjacency(void *item, xmlTextReaderPtr reader)
{
    xml_elt read[] =
        {{0,
          BAD_CAST "interval",
          item,
          read_left_interval}};

    if(!read_elements(reader, sizeof(read)/sizeof(read[0]), read, BAD_CAST "left"))
        return false;

    return read[0].count == 1;
}

static bool read_right_interval(void *item, xmlTextReaderPtr reader)
{
    lane *l = reinterpret_cast<lane*>(item);
    return l->right.xml_read(reader, BAD_CAST "lane_adjacency");
}

static bool read_right_adjacency(void *item, xmlTextReaderPtr reader)
{
    xml_elt read[] =
        {{0,
          BAD_CAST "interval",
          item,
          read_right_interval}};

    if(!read_elements(reader, sizeof(read)/sizeof(read[0]), read, BAD_CAST "right"))
        return false;

    return read[0].count == 1;
}

static bool read_adjacency(void *item, xmlTextReaderPtr reader)
{
    xml_elt read[] =
        {{0,
          BAD_CAST "left",
          item,
          read_left_adjacency},
         {0,
          BAD_CAST "right",
          item,
          read_right_adjacency}};

    if(!read_elements(reader, sizeof(read)/sizeof(read[0]), read, BAD_CAST "adjacency_intervals"))
        return false;

    return read[0].count == 1 && read[1].count == 1;
}

bool road_membership::xml_read(xmlTextReaderPtr reader)
{
    return get_attribute(parent_road.sp, reader, "parent_road_ref") &&
        get_attribute(interval[0], reader, "interval_start") &&
        get_attribute(interval[1], reader, "interval_end") &&
        get_attribute(lane_position, reader, "lane_position");
}

bool lane::xml_read(xmlTextReaderPtr reader)
{
    if(!get_attribute(speedlimit, reader, "speedlimit"))
        return false;

    xml_elt read[] =
        {{0,
          BAD_CAST "start",
          this,
          read_start},
         {0,
          BAD_CAST "end",
          this,
          read_end},
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

    return read[0].count == 1 && read[1].count == 1 && read[2].count == 1 && read[3].count == 1;
}
