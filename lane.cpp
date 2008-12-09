#include "lane.hpp"

bool lane::xml_read(xmlTextReaderPtr reader)
{
    do
    {
        int ret = xmlTextReaderRead(reader);
        if(ret != 1)
            return false;
    }
    while(!is_closing_element(reader, "lane"));

    return true;
}
