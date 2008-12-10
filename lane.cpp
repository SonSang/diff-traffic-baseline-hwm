#include "lane.hpp"

bool lane::xml_read(xmlTextReaderPtr reader)
{
    bool have_start     = false;
    bool have_end       = false;
    bool have_road_int  = false;
    bool have_adjacency = false;

    if(!get_attribute(speedlimit, reader, "speedlimit"))
        return false;

    do
    {
        int ret = xmlTextReaderRead(reader);
        if(ret != 1)
            return false;

        if(xmlTextReaderNodeType(reader) == XML_READER_TYPE_ELEMENT)
        {
            const xmlChar *name = xmlTextReaderConstName(reader);
            if(!name)
                return false;

            if(xmlStrEqual(name, BAD_CAST "start") && !have_start)
            {
                do
                {
                    ret = xmlTextReaderRead(reader);
                    if(ret != 1)
                        return false;
                }
                while(!is_closing_element(reader, "start"));

                have_start = true;
            }
            else if(xmlStrEqual(name, BAD_CAST "end") && !have_end)
            {
                do
                {
                    ret = xmlTextReaderRead(reader);
                    if(ret != 1)
                        return false;
                }
                while(!is_closing_element(reader, "end"));

                have_end = true;
            }
            else if(xmlStrEqual(name, BAD_CAST "road_intervals") && !have_road_int)
            {
                do
                {
                    ret = xmlTextReaderRead(reader);
                    if(ret != 1)
                        return false;

                    if(xmlTextReaderNodeType(reader) == XML_READER_TYPE_ELEMENT)
                    {
                        const xmlChar *name = xmlTextReaderConstName(reader);
                        if(!name)
                            return false;
                    }

                }
                while(!is_closing_element(reader, "road_intervals"));

                have_road_int = true;
            }
            else if(xmlStrEqual(name, BAD_CAST "adjacency_intervals") && !have_adjacency)
            {
                do
                {
                    ret = xmlTextReaderRead(reader);
                    if(ret != 1)
                        return false;
                }
                while(!is_closing_element(reader, "adjacency_intervals"));

                have_adjacency = true;
            }
            else
                return false;
        }
    }
    while(!is_closing_element(reader, "lane"));

    return have_start && have_end && have_road_int && have_adjacency;
}
