#include <libxml/xmlreader.h>
#include <cstring>

inline bool is_opening_element(xmlTextReaderPtr reader, const xmlChar *eltname)
{
    const xmlChar *name = xmlTextReaderConstName(reader);

    return name && xmlTextReaderNodeType(reader) == XML_READER_TYPE_ELEMENT
        && xmlStrEqual(eltname, name);
}

inline bool is_closing_element(xmlTextReaderPtr reader, const xmlChar *eltname)
{
    const xmlChar *name = xmlTextReaderConstName(reader);

    return name && xmlTextReaderNodeType(reader) == XML_READER_TYPE_END_ELEMENT
        && xmlStrEqual(eltname, name);
}


inline bool is_opening_element(xmlTextReaderPtr reader, const char *eltname)
{
    return is_opening_element(reader, BAD_CAST eltname);
}

inline bool is_closing_element(xmlTextReaderPtr reader, const char *eltname)
{
    return is_closing_element(reader, BAD_CAST eltname);
}
