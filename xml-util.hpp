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

template <typename T>
inline void str_convert(T & res, const xmlChar *str);

template <>
inline void str_convert<float>(float & res, const xmlChar *str)
{
    sscanf((const char*) str, "%f", &res);
}

template <>
inline void str_convert<int>(int & res, const xmlChar *str)
{
    sscanf((const char*) str, "%d", &res);
}

template <typename T>
inline bool get_attribute(T & res, xmlTextReaderPtr reader, const xmlChar *eltname)
{
    xmlChar *att = xmlTextReaderGetAttribute(reader, eltname);
    if(!att)
        return -1;
    str_convert<T>(res, att);
    free(att);

    return 1;
}

template <>
inline bool get_attribute<char*>(char* & res, xmlTextReaderPtr reader, const xmlChar *eltname)
{
    xmlChar *att = xmlTextReaderGetAttribute(reader, eltname);
    if(!att)
        return -1;
    res = (char*)att; // Not very safe, but I don't expect much unicode...

    return 1;
}

template<typename T>
inline bool get_attribute(T & res, xmlTextReaderPtr reader, const char *eltname)
{
    return get_attribute(res, reader, BAD_CAST eltname);
}


