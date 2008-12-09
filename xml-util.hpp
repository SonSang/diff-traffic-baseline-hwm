#ifndef _XML_UTIL_HPP_
#define _XML_UTIL_HPP_

#include <libxml/xmlreader.h>
#include <libxml/xinclude.h>
#include <cstring>
#include <vector>
#include <map>

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
        return false;
    str_convert<T>(res, att);
    free(att);

    return true;
}

template <>
inline bool get_attribute<char*>(char* & res, xmlTextReaderPtr reader, const xmlChar *eltname)
{
    xmlChar *att = xmlTextReaderGetAttribute(reader, eltname);
    if(!att)
        return false;
    res = (char*)att; // Not very safe, but I don't expect much unicode...

    return true;
}

template <typename T>
inline bool get_attribute(T & res, xmlTextReaderPtr reader, const char *eltname)
{
    return get_attribute(res, reader, BAD_CAST eltname);
}

template <typename T>
inline bool read_sequence(std::vector<T> & seq, std::map<char *, int> & id_map, xmlTextReaderPtr reader, const xmlChar * item_name, const xmlChar * container_name)
{
    int ret;
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

            if(xmlStrEqual(name, item_name))
            {
                seq.push_back(T());
                char *id = 0;
                if(!get_attribute(id, reader, "id"))
                    return false;
                if(id_map.find(id) != id_map.end())
                    return false;
                id_map[id] = seq.size()-1;
                seq[seq.size()-1].xml_read(reader);
            }
            else
                return false;
        }
    }
    while(ret == 1 && !is_closing_element(reader, container_name));

    return ret == 1;
}

template <typename T>
inline bool read_sequence(std::vector<T> & seq, std::map<char *, int> & id_map, xmlTextReaderPtr reader, const char * item_name, const char * container_name)
{
    return read_sequence(seq, id_map, reader, BAD_CAST item_name, BAD_CAST container_name);
}

#endif
