#include "intersection.hpp"

bool intersection::xml_read(xmlTextReaderPtr reader)
{
    do
    {
        int ret = xmlTextReaderRead(reader);
        if(ret != 1)
            return false;
    }
    while(!is_closing_element(reader, "intersection"));

    return true;
}
