#ifndef _INTERVALS_HPP_
#define _INTERVALS_HPP_

#include <vector>
#include <cassert>
#include <cstdio>
#include "xml-util.hpp"

template<typename T>
struct interval_xml_read
{
    static bool xml_read(T & item, xmlTextReaderPtr reader);
};

//! Associates data with intervals that cover the range [0, 1].
/*!
  There are n (0+) 'dividers' that split up the range into n + 1 data items:
  \f$[0, x_0)    [x_0, ...) [..., x_{n-1}), [x_{n-1}, 1]\f$
  base_data     data_0     data_{n-2}      data_{n-1}
  Note that the last range is inclusive; them's the breaks.
  We could conceivably use a method to insert a whole sub-interval - i.e. some (a,b)
  assocated with specific data. There are interesting problems with well-posed-ness there.
  We'll hold off
*/
template <class T>
struct intervals
{
    typedef interval_xml_read<T> xml_reader;
    typedef int entry_id;

    //! Stores additional dividers + associated data.
    struct entry
    {
        float divider; //!< Has to be in (0, 1).
        T data;        //!< Data associated with interval beginning with divider.
    };

    T base_data;                //!< Data assocated with [0,..).
    std::vector<entry> entries; //!< Storage for additional entries.

    //! Search structure for entry.
    /*!
      \param x A number in (0.0, 1.0).
      \param t x rescaled to the found interval
      \return -1 if the number is covered by base_data, the number of the entry that covers it otherwise.
      This is just linear search right now; something more scalable (like binary search) is conceivably desireable.
    */
    inline entry_id find(float &x) const
    {
        if(entries.empty())
            return -1;
        else if(x < entries[0].divider)
        {
            x /= entries[0].divider;
            return -1;
        }
        else
        {
            size_t current = 0;
            while(x > entries[current].divider)
            {
                ++current;
                if(current >= entries.size())
                {
                    x = (x-entries.back().divider)/(1.0-entries.back().divider);
                    return current-1;
                }
            }
            x = (x-entries[current-1].divider)/(entries[current].divider-entries[current-1].divider);
            return current-1;
        }
    }

    //! Search structure for entry.
    /*!
      \param x A number in (0.0, 1.0).
      \return -1 if the number is covered by base_data, the number of the entry that covers it otherwise.
      This is just linear search right now; something more scalable (like binary search) is conceivably desireable.
    */
    inline entry_id find(const float &x) const
    {
        if(entries.empty() || x < entries[0].divider)
            return -1;
        else
        {
            size_t current = 0;
            while(x > entries[current].divider)
            {
                ++current;
                if(current >= entries.size())
                    return current-1;
            }
            return current-1;
        }
    }

    //! Insert new divider.
    /*!
      Inserts a new divider starting at x until the next divider.
      Note that a more sophisticated version of this would have some method to 'resize' data
      subsumed by the new interval, perhaps.
      \param x A number in (0.0, 1.0).
      \param data The associated data for [x,...).
    */
    inline void insert(const float &x, T data)
    {
        entry_id loc = find(x) + 1;
        entries.resize(entries.size()+1);

        entry_id current = entries.size()-1;
        while(current > loc)
        {
            entries[current] = entries[current-1];
            --current;
        }
        entries[loc].divider = x;
        entries[loc].data = data;
    }

    //! Modify/retrieve data entry.
    /*!
      Finds interval containing x and returns assocated data.
      \param x A number in (0.0, 1.0).
      \returns The associated data for the found interval.
    */
    inline T & operator[](const float &x)
    {
        entry_id loc = find(x);
        if(loc == -1)
            return base_data;
        else
            return entries[loc].data;
    }

    //! Retrieve data entry.
    /*!
      Finds interval containing x and returns assocated data.
      \param x A number in (0.0, 1.0).
      \returns The associated data for the found interval (as a const).
    */
    inline const T & operator[](const float &x) const
    {
        entry_id loc = find(x);
        if(loc == -1)
            return base_data;
        else
            return entries[loc].data;
    }

    //! Modify/retrieve data entry and rescale search param
    /*!
      Finds interval containing x and returns assocated data.
      Also rescales x to the found interval
      \param x A number in (0.0, 1.0).
      \returns The associated data for the found interval.
    */
    inline T & get_rescale(float &x)
    {
        entry_id loc = find(x);
        if(loc == -1)
            return base_data;
        else
            return entries[loc].data;
    }

    //! Retrieve data entry and rescale search param
    /*!
      Finds interval containing x and returns assocated data.
      Also rescales x to the found interval
      \param x A number in (0.0, 1.0).
      \returns The associated (const) data for the found interval.
    */
    inline const T & get_rescale(float &x) const
    {
        entry_id loc = find(x);
        if(loc == -1)
            return base_data;
        else
            return entries[loc].data;
    }

    inline bool xml_read(xmlTextReaderPtr reader, const xmlChar *eltname)
    {
        bool have_base = false;

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

                if(xmlStrEqual(name, BAD_CAST "base") && !have_base)
                {
                    do
                    {
                        ret = xmlTextReaderRead(reader);
                        if(ret != 1)
                            return false;

                        if(xmlTextReaderNodeType(reader) == XML_READER_TYPE_ELEMENT)
                        {
                            const xmlChar *name = xmlTextReaderConstName(reader);
                            if(!name || !xmlStrEqual(name, eltname) || !xml_reader::xml_read(base_data, reader))
                                return false;
                        }
                    }
                    while(!is_closing_element(reader, "base"));

                    have_base = true;
                }
                else if(xmlStrEqual(name, BAD_CAST "divider"))
                {
                    float div_val;
                    if(!get_attribute(div_val, reader, "value"))
                        return false;

                    do
                    {
                        ret = xmlTextReaderRead(reader);
                        if(ret != 1)
                            return false;

                        if(xmlTextReaderNodeType(reader) == XML_READER_TYPE_ELEMENT)
                        {
                            const xmlChar *name = xmlTextReaderConstName(reader);
                            if(name && xmlStrEqual(name, eltname))
                            {
                                T elt;

                                if(!xml_reader::xml_read(elt, reader))
                                    return false;

                                insert(div_val, elt);
                            }
                            else
                                return false;
                        }
                    }
                    while(!is_closing_element(reader, "divider"));
                }
                else
                    return false;
            }
        }
        while(!is_closing_element(reader, "interval"));

        return have_base;
    }

    //! Build a string rep of the structure.
    /*!
      Handy for debugging purposes. Fills buff with the first len
      characters ofa string rep.
      \param buff An array of at least len characters.
      \param len  The max. number of characters to write in buff (including trailing \0).
      \returns The number of written characters (not inluding trailing \0).
    */
    inline int to_string(char buff[], int len) const;

};

//! Implementation of to_string for T = float
template <>
inline int intervals<float>::to_string(char buff[], int len) const
{
    if(entries.empty())
        return snprintf(buff, len, "[0.0 1.0]: %f", base_data);
    else
    {
        int offs = 0;
        offs += snprintf(buff, len, "[0.0 %f): %f; ", entries[0].divider, base_data);

        int i = 0;
        for(; i < static_cast<int>(entries.size()-1) && offs < len; ++i)
            offs += snprintf(buff+offs, len-offs, "[%f %f): %f; ", entries[i].divider, entries[i+1].divider, entries[i].data);

        if(offs >= len)
            return offs;

        return offs + snprintf(buff+offs, len-offs, "[%f 1.0]: %f",  entries[i].divider, entries[i].data);
    }
}
#endif
