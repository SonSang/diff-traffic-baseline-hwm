#include "road.hpp"

void line_rep::locate(point *pt, float t, float offset) const
{
    int seg = find_segment(t);

    t = t*lengths.back() - lengths[seg];

    pt->x = t * normals[seg].x + points[seg].x;
    pt->y = t * normals[seg].y + points[seg].y;
}

void line_rep::draw(float start, float stop, float offset) const
{
}

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

void line_rep::calc_rep()
{
    /*!
      strictly speaking, this will work with points = 1,
      but the class doesn't make much sense then.
    */
    assert(points.size() > 1);

    lengths.resize(points.size());
    normals.resize(points.size()-1);
    mitres.resize(points.size()-2);

    lengths[0] = 0.0f;
    for(int i = 1; i < static_cast<int>(points.size()); ++i)
    {
        normals[i-1].x = points[i].x-points[i-1].x;
        normals[i-1].y = points[i].y-points[i-1].y;

        float len = std::sqrt(normals[i-1].x*normals[i-1].x + normals[i-1].y*normals[i-1].y);

        lengths[i] = lengths[i-1] + len;

        float invlen = 1.0f/len;
        normals[i-1].x *= invlen;
        normals[i-1].y *= invlen;
    }
    for(int i = 0; i < static_cast<int>(mitres.size()); ++i)
    {
        float dot = normals[i].x*normals[i+1].x + normals[i].y*normals[i+1].y;

        mitres[i] = (1.0f - dot)/(1.0f + dot);
    }
}

int line_rep::to_string(char buff[], int len) const
{
    int offs = 0;
    for(int i = 0; i < static_cast<int>(points.size()) && offs < len; ++i)
        offs += snprintf(buff+offs, len-offs, "%d: (%f, %f) - %f\n", i, points[i].x, points[i].y, lengths[i]);

    return offs;
}
