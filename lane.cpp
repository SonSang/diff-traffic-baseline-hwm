#include "lane.hpp"

bool lane::xml_read(xmlTextReaderPtr reader)
{
    if(!get_attribute(speedlimit, reader, "speedlimit"))
        return false;

    do
    {
        int ret = xmlTextReaderRead(reader);
        if(ret != 1)
            return false;

        // get start
        // get end
        // get road intervals
        // get adjacency intervals
    }
    while(!is_closing_element(reader, "lane"));

    return true;
}
