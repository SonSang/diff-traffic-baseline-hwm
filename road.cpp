#include "road.hpp"

void line_rep::locate(point *pt, float t, float offset) const
{
    int seg = find_segment(t);

    t = (t*lengths.back() - lengths[seg])/(lengths[seg+1]-lengths[seg]);

    pt->x = (1.0-t) * points[seg].x + t * points[seg+1].x;
    pt->y = (1.0-t) * points[seg].y + t * points[seg+1].y;
}

void line_rep::draw(float start, float stop, float offset) const
{
}

//! Find the segment containing x.
/*
  \param x A float in [0.0, 1.0].
  \returns The segment containing x
*/
int line_rep::find_segment(float x) const
{
    assert(x >= 0 && x <= 1.0);

    x *= lengths.back();

    int current = 1;
    while(current < static_cast<int>(lengths.size()))
    {
        if(x-lengths[current] < 0.0f)
            return current-1;
        ++current;
    }
    return lengths.size()-2;
}

//! Fills lengths vector.
/*!
  This assumes that points has at least 2 elements in it;
  we overwrite any existing lengths info with new calculations.
*/
void line_rep::calc_lengths()
{
    /*!
      strictly speaking, this will work with points = 1,
      but the class doesn't make much sense then.
    */
    assert(points.size() > 1);

    lengths.resize(points.size());

    lengths[0] = 0.0f;
    for(int i = 1; i < static_cast<int>(points.size()); ++i)
    {
        lengths[i] = lengths[i-1] +
            std::sqrt((points[i].x-points[i-1].x)*(points[i].x-points[i-1].x) + (points[i].y-points[i-1].y)*(points[i].y-points[i-1].y));
    }
}

//! Create a string representation of line
/*!
  Fills buff with the first len characters of rep
  \param buff An array of at least len characters.
  \param len  The max. number of characters to write in buff (including trailing \0).
  \returns The number of written characters (not inluding trailing \0).
*/
int line_rep::to_string(char buff[], int len) const
{
    int offs = 0;
    for(int i = 0; i < static_cast<int>(points.size()) && offs < len; ++i)
        offs += snprintf(buff+offs, len-offs, "%d: (%f, %f) - %f\n", i, points[i].x, points[i].y, lengths[i]);

    return offs;
}
