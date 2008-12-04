#ifndef _INTERVALS_HPP_
#define _INTERVALS_HPP_

#include <vector>
#include <cassert>
#include <cstdio>

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
      \return -1 if the number is covered by base_data, the number of the entry that covers it otherwise.
      This is just linear search right now; something more scalable (like binary search) is conceivably desireable.
    */
    inline int find(float x) const
    {
        assert(0.0 <= x && x <= 1.0);

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
    inline void insert(float x, T data)
    {
        int loc = find(x) + 1;
        printf("loc: %d/%zu\n", loc-1, entries.size());
        entries.resize(entries.size()+1);

        int current = entries.size();
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
    inline T & operator[](float x)
    {
        int loc = find(x);
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
    inline const T & operator[](float x) const
    {
        return static_cast<const T &>((*this)[x]);
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
