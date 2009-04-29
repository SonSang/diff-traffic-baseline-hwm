#ifndef _XML_UTIL_HPP_
#define _XML_UTIL_HPP_

#include <boost/fusion/container/vector.hpp>
#include <boost/fusion/include/vector.hpp>
#include <boost/fusion/sequence/intrinsic/at.hpp>
#include <libxml/xmlreader.h>
#include <libxml/xinclude.h>
#include <cstring>
#include <vector>
#include <map>
#ifdef _MSC_VER
#define snprintf _snprintf
#define __restrict__
#define copysign _copysign
#define SIZE_T_FMT "%lu"
#else
#define SIZE_T_FMT "%zu"
#endif


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
    {
        printf("no attribute %s\n", (const char*)eltname);
        return false;
    }
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
inline bool read_sequence(std::vector<T> & seq, std::map<char *, int, ltstr> & id_map, xmlTextReaderPtr reader, const xmlChar * item_name, const xmlChar * container_name)
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
inline bool read_sequence(std::vector<T> & seq, std::map<char *, int, ltstr> & id_map, xmlTextReaderPtr reader, const char * item_name, const char * container_name)
{
    return read_sequence(seq, id_map, reader, BAD_CAST item_name, BAD_CAST container_name);
}

struct xml_elt
{
    int count;
    const xmlChar *name;
    void *item;
    bool (*read_xml)(void *, xmlTextReaderPtr reader);
};

inline bool read_elements(xmlTextReaderPtr reader, int nelt, xml_elt *elt, const xmlChar *endelt)
{
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

            bool found = false;
            for(int i = 0; i < nelt; ++i)
                if(xmlStrEqual(name, elt[i].name))
                {
                    if(!elt[i].read_xml(elt[i].item, reader))
                        return false;

                    ++elt[i].count;
                    found = true;
                    break;
                }

            if(!found)
                return false;
        }
    }
    while(!is_closing_element(reader, endelt));

    return true;
}

template <typename T>
struct converter;

template <>
struct converter<float>
{
    static inline bool from_string(float *f, const char *vstr)
    {
        return sscanf(vstr, "%f", f) == 1;
    }
};

template <>
struct converter<int>
{
    static inline bool from_string(int *f, const char *vstr)
    {
        return sscanf(vstr, "%d", f) == 1;
    }
};

template <>
struct converter<char*>
{
    static inline bool from_string(char **f, const char *vstr)
    {
        return (*f = strdup(vstr));
    }
};

template <>
struct converter<std::string>
{
    static inline bool from_string(std::string *f, const char *vstr)
    {
        *f = std::string(vstr);
        return true;
    }
};

template <typename T>
struct list_matcher
{
    list_matcher(const char *n, T *v)
        : name(n), count(0), val(v)
    {}

    const char *name;

    int count;
    T * val;

    inline bool match(const char *nstr, const char *vstr)
    {
        if(!count && strcmp(nstr, name) == 0 && converter<T>::from_string(val, vstr))
        {
            ++count;
            return true;
        }
        return false;
    }
};

template <typename T, int N>
struct walker
{
    static inline bool walk(T &v, const char *nstr, const char *vstr)
    {
        if(boost::fusion::at_c<N>(v).match(nstr, vstr))
            return true;
        else
            return walker<T, N-1>::walk(v, nstr, vstr);
    }

    static inline bool is_full(T &v)
    {
        if(boost::fusion::at_c<N>(v).count == 0)
            return false;

        return walker<T, N-1>::is_full(v);
    }
};

template <typename T>
struct walker<T, 0>
{
    static inline bool walk(T & v, const char *nstr, const char *vstr)
    {
        return boost::fusion::at_c<0>(v).match(nstr, vstr);
    }

    static inline bool is_full(T & v)
    {
        return boost::fusion::at_c<0>(v).count > 0;
    }
};

template <typename T>
bool walk(T &v, const char *nstr, const char *vstr)
{
    return walker<T, boost::fusion::result_of::size<T>::value-1>::walk(v, nstr, vstr);
}

template <typename T>
bool is_full(T &v)
{
    return walker<T, boost::fusion::result_of::size<T>::value-1>::is_full(v);
}

template <typename T>
list_matcher<T> lm(const char *name, T *val)
{
    return list_matcher<T>(name, val);
}

template <typename T>
inline bool read_attributes(T & v, xmlTextReaderPtr reader)
{
    if(xmlTextReaderMoveToFirstAttribute(reader) != 1)
        return false;

    while(!is_full(v))
    {
        walk(v, (const char*)xmlTextReaderConstName(reader), (const char*)xmlTextReaderConstValue(reader));
        if(xmlTextReaderMoveToNextAttribute(reader) != 1)
            break;
    }

    xmlTextReaderMoveToElement(reader);

    return true;
}

inline bool read_leaf_text(std::string &res, xmlTextReaderPtr reader, const char *endtag)
{
    bool read_text = false;

    do
    {
        int ret = xmlTextReaderRead(reader);
        if(ret != 1)
            return false;

        if(xmlTextReaderNodeType(reader) == XML_READER_TYPE_TEXT)
        {
            res.append((const char *) xmlTextReaderConstValue(reader));
            read_text = true;
        }
    }
    while(!is_closing_element(reader, endtag));

    return read_text;
}
#endif
